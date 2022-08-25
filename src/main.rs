#![no_std]
#![no_main]


#[macro_use]
mod audio_read;
#[allow(dead_code)]
mod dma;
mod logger;
#[allow(dead_code)]
mod pdm;

use cortex_m_rt::{entry, exception};
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU32, Ordering};
use log::info;
use stm32h7xx_hal as hal;
use stm32h7xx_hal::{
    prelude::*,
    rcc::ResetEnable,
    device,
    device::interrupt,
    ethernet,
    ethernet::PHY
};
use smoltcp::iface::{ InterfaceBuilder, SocketStorage, Neighbor, NeighborCache, Route, Routes};
use smoltcp::wire::{HardwareAddress, IpCidr, Ipv4Cidr, IpAddress, Ipv4Address, IpEndpoint};
use smoltcp::socket::{UdpSocket, UdpSocketBuffer, UdpPacketMetadata};
use smoltcp::time::Instant;

/// TIME is an atomic u32 that counts milliseconds. Although not used
/// here, it is very useful to have for network protocols
static TIME: AtomicU32 = AtomicU32::new(0);

fn systick_init(syst: &mut device::SYST, clocks: hal::rcc::CoreClocks) {
    let c_ck_mhz = clocks.c_ck().to_MHz();

    let syst_calib = 0x3E8;

    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    syst.set_reload((syst_calib * c_ck_mhz) - 1);
    syst.enable_interrupt();
    syst.enable_counter();
}

/// Ethernet descriptor rings are a global singleton
#[link_section = ".sram3.eth"]
static mut DES_RING: ethernet::DesRing<4, 4> = ethernet::DesRing::new();

/// Net storage with static initialisation - another global singleton
pub struct NetStorageStatic<'a> {
    //ip_addrs: [IpCidr; 1],
    socket_storage: [SocketStorage<'a>; 8],
    neighbor_cache_storage: [Option<(IpAddress, Neighbor)>; 8],
    routes_storage: [Option<(IpCidr, Route)>; 1],
}

// Storage for the socket queue
// This is intermediate storage, it gets copied again for transmission so it doesn't
// need to be accessible by the ethernet DMA. However, if we want to save CPU cycles
// we could look into sending packets straight to the PHY layer and skipping this copy.
static mut PAYLOAD_STORAGE: [u8; 4096] = [0u8; 4096];
static mut METADATA_STORAGE: [UdpPacketMetadata; 10] = [UdpPacketMetadata::EMPTY; 10];

// Allocate storage for smoltcp
static mut STORE: NetStorageStatic = NetStorageStatic {
    socket_storage: [SocketStorage::EMPTY; 8],
    neighbor_cache_storage: [None; 8],
    routes_storage: [None; 1],
};

// static storage for the IP addrs for smoltcp
static mut IP_ADDRS : MaybeUninit<[IpCidr; 1]> = MaybeUninit::uninit();

#[entry]
fn main() -> ! {
    let dp = device::Peripherals::take().unwrap();
    let mut cp = device::CorePeripherals::take().unwrap();
    
    logger::init();
    
    /// Locally administered, arbitrary MAC address
    /// TODO: Could randomize this based on device serial...
    const MAC_ADDRESS: [u8; 6] = [0x02, 0xab, 0xab, 0xab, 0xab, 0xab];

    let pwr = dp.PWR.constrain().freeze();

    // Initialise SRAM3
    dp.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(400.MHz())
        .hclk(200.MHz())
        .pll1_r_ck(100.MHz()) // for TRACECK
        .pll2_p_ck(18432.kHz()) // for SAI1
        .freeze(pwr, &dp.SYSCFG);


    // Turn on clocks for SAI and the DMA we'll use to transfer from it
    ccdr.peripheral.SAI1.enable()
    .kernel_clk_mux(device::rcc::d2ccip1r::SAI1SEL_A::PLL2_P);
    ccdr.peripheral.DMA1.enable();

    // Initialise system...
    cp.SCB.enable_icache();
    // TODO: ETH DMA coherence issues
    // cp.SCB.enable_dcache(&mut cp.CPUID);
    cp.DWT.enable_cycle_counter();

    // Initialise IO...
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);
    let mut link_led = gpiob.pb0.into_push_pull_output(); // LED1, green
    link_led.set_low();

    let rmii_ref_clk = gpioa.pa1.into_alternate();
    let rmii_mdio = gpioa.pa2.into_alternate();
    let rmii_mdc = gpioc.pc1.into_alternate();
    let rmii_crs_dv = gpioa.pa7.into_alternate();
    let rmii_rxd0 = gpioc.pc4.into_alternate();
    let rmii_rxd1 = gpioc.pc5.into_alternate();
    let rmii_tx_en = gpiog.pg11.into_alternate();
    let rmii_txd0 = gpiog.pg13.into_alternate();
    let rmii_txd1 = gpiob.pb13.into_alternate();

    // Configure pdm pins
    let _pdm_d0 = gpiob.pb2.into_alternate::<2>();
    let _pdm_d1 = gpioe.pe4.into_alternate::<2>();
    let _pdm_d2 = gpiof.pf10.into_alternate::<2>().internal_pull_up(true);
    let _pdm_clk = gpioe.pe2.into_alternate::<2>();

    assert_eq!(ccdr.clocks.hclk().raw(), 200_000_000); // HCLK 200MHz
    assert_eq!(ccdr.clocks.pclk1().raw(), 100_000_000); // PCLK 100MHz
    assert_eq!(ccdr.clocks.pclk2().raw(), 100_000_000); // PCLK 100MHz
    assert_eq!(ccdr.clocks.pclk4().raw(), 100_000_000); // PCLK 100MHz

    // Initialise ethernet...
    let mac_addr = smoltcp::wire::EthernetAddress::from_bytes(&MAC_ADDRESS);
    let (eth_dma, eth_mac) = unsafe {
        ethernet::new(
            dp.ETHERNET_MAC,
            dp.ETHERNET_MTL,
            dp.ETHERNET_DMA,
            (
                rmii_ref_clk,
                rmii_mdio,
                rmii_mdc,
                rmii_crs_dv,
                rmii_rxd0,
                rmii_rxd1,
                rmii_tx_en,
                rmii_txd0,
                rmii_txd1,
            ),
            &mut DES_RING,
            mac_addr,
            ccdr.peripheral.ETH1MAC,
            &ccdr.clocks,
        )
    };

    // Initialise ethernet PHY...
    let mut lan8742a = ethernet::phy::LAN8742A::new(eth_mac.set_phy_addr(0));
    lan8742a.phy_reset();
    lan8742a.phy_init();

    let store = unsafe { &mut STORE };
    let neighbor_cache =
        NeighborCache::new(&mut store.neighbor_cache_storage[..]);
    let routes = Routes::new(&mut store.routes_storage[..]);
    
    unsafe { IP_ADDRS.write([IpCidr::Ipv4(Ipv4Cidr::new(Ipv4Address([192, 168, 1, 58]), 24))]) };
    let ip_addrs = unsafe { IP_ADDRS.assume_init_mut() };

    let mut interface = InterfaceBuilder::new(eth_dma, &mut store.socket_storage[..])
        .hardware_addr(HardwareAddress::Ethernet(mac_addr))
        .neighbor_cache(neighbor_cache)
        .routes(routes)
        .ip_addrs(&mut ip_addrs[..])
        .finalize();
    
    let payload_storage = unsafe { PAYLOAD_STORAGE.as_mut() };
    let metadata_storage = unsafe { METADATA_STORAGE.as_mut() };
    let tx_buffer = UdpSocketBuffer::new(&mut metadata_storage[..], &mut payload_storage[..]);
    let rx_buffer = UdpSocketBuffer::new(&mut [][..], &mut [][..]);
    let mut socket = UdpSocket::new(rx_buffer, tx_buffer);
    // Broadcast src port; it's pretty much arbitrary.
    socket.bind(IpEndpoint::new(IpAddress::Ipv4(Ipv4Address::UNSPECIFIED), 10000)).unwrap();
    
    let socket_handle = interface.add_socket(socket);

    // Enable the ethernet IRQ
    unsafe {
        ethernet::enable_interrupt();
        cp.NVIC.set_priority(device::Interrupt::ETH, 196); // Mid prio
        cortex_m::peripheral::NVIC::unmask(device::Interrupt::ETH);
    }

    // Enable the DMA IRQ
    unsafe {
        cp.NVIC.set_priority(device::Interrupt::DMA1_STR0, 100);
        cortex_m::peripheral::NVIC::unmask(device::Interrupt::DMA1_STR0);
    }

    let delay = cp.SYST.delay(ccdr.clocks);
    systick_init(&mut delay.free(), ccdr.clocks);
    unsafe {
        cp.SCB.shpr[15 - 4].write(128);
    } // systick exception priority

    let sai1 = unsafe {&*device::SAI1::PTR};
    let dma1 = unsafe {&*device::DMA1::PTR};
    let audio = audio_read::AudioReader::init(sai1, dma1);

    let mut current_data_buf: Option<audio_read::Buffer> = None;
    let mut current_data_pos: usize = 0;
    
    let data_ep = IpEndpoint::new(IpAddress::Ipv4(Ipv4Address::BROADCAST), 10200);
    let mut packet_seq_counter = 0u8;
    loop {
        let time = TIME.load(Ordering::Relaxed);

        // Run smoltcp processing
        interface.poll(Instant::from_millis(time)).unwrap();
        
        
        // READ DATA
        if current_data_buf.is_none() {
            current_data_buf = audio.poll_for_data();
            current_data_pos = 0;
        }
        // SEND DATA
        if current_data_buf.is_some() {
            let buffer = current_data_buf.as_mut().unwrap();
            let socket = interface.get_socket::<UdpSocket>(socket_handle);
            match socket.send(audio_read::UDP_PACKET_SIZE+2, data_ep) {
                Ok(packet_buf) => {
                    let data = buffer.as_mut();
                    let end = current_data_pos + audio_read::UDP_PACKET_SIZE;
                    packet_buf[0..audio_read::UDP_PACKET_SIZE].copy_from_slice(&data[current_data_pos..end]);
                    current_data_pos += audio_read::UDP_PACKET_SIZE;
                    if current_data_pos >= audio_read::DMA_BUFFER_SIZE {
                        audio.return_buffer(current_data_buf.take().unwrap());
                    }
                    packet_seq_counter = packet_seq_counter.overflowing_add(1).0;
                    packet_buf[audio_read::UDP_PACKET_SIZE] = packet_seq_counter;
                    let ovf_flag = audio_read::read_dma_overflow_flag();
                    packet_buf[audio_read::UDP_PACKET_SIZE+1] = 0;
                    if ovf_flag {
                        packet_buf[audio_read::UDP_PACKET_SIZE+1] |= 1;
                    }
                },
                Err(_) => {
                    // We will try again next time through
                }
            }
        }

        // Ethernet
        // let eth_last = eth_up;
        // eth_up = lan8742a.poll_link();
        // match eth_up {
        //     true => link_led.set_high(),
        //     _ => link_led.set_low(),
        // }

        // if eth_up != eth_last {
        //     // Interface state change
        //     match eth_up {
        //         true => info!("Ethernet UP"),
        //         _ => info!("Ethernet DOWN"),
        //     }
        // }
    }
}

#[interrupt]
fn ETH() {
    unsafe { ethernet::interrupt_handler() }
}

#[exception]
fn SysTick() {
    TIME.fetch_add(1, Ordering::Relaxed);
}

#[exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[exception]
unsafe fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}