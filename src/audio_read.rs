use heapless::spsc::Queue;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};
use crate::{device, dma};
use crate::device::interrupt;
use crate::pdm::PdmInput;
use crate::hal::dma::traits::TargetAddress;
use log::info;

// Amount of data to be sent in each UDP packet
pub const UDP_PACKET_SIZE: usize = 6 * 80;
// Amount of data in each DMA transfer from the SAI
pub const DMA_BUFFER_SIZE: usize = UDP_PACKET_SIZE * 3;
// Number of audio buffers to allocate At any given time, two can be assigned to the DMA, and at
// least one other will be actively being copied to an ethernet buffer
const NUM_AUDIO_BUFFERS: usize = 4;
// DMA1/2 cannot access the TCM memory (e.g. where stack/heap is)
// Data needs to go in one of the other SRAMs
#[link_section = ".axisram.audio"]
static mut AUDIO_BUFFERS: [[u8; DMA_BUFFER_SIZE]; NUM_AUDIO_BUFFERS] = [[0u8; DMA_BUFFER_SIZE]; NUM_AUDIO_BUFFERS];

// Flag to be set by DMA IRQ if it does not execute, or cannot aquire a free buffer in time
pub static mut DMA_OVERRUN: AtomicBool = AtomicBool::new(false);


#[derive(Debug)]
pub struct Buffer {
    buf: *mut [u8; DMA_BUFFER_SIZE]
}

impl Buffer{
    pub fn new(buf: *mut [u8; DMA_BUFFER_SIZE]) -> Self {
        Self { buf: buf }
    }

    pub fn as_mut(&mut self) -> &mut [u8] {
        let buf = self.buf;
        let b2 = unsafe { &mut *buf };
        b2
    }

    pub fn ptr(&self) -> *mut [u8; DMA_BUFFER_SIZE] {
        self.buf
    }
}

// Buffer scheme for passing around ownership of a finite number of audio buffers.
// Two are always owned by the DMA (SLOT0, SLOT1), one is held by the transmitter 
// in MAIN, and remainder are in the queues.
static mut READY_TO_FILL: Queue<Buffer, {NUM_AUDIO_BUFFERS + 1}> = Queue::new();
static mut READY_TO_SEND: Queue<Buffer, {NUM_AUDIO_BUFFERS + 1}> = Queue::new();
static mut SLOT0_BUF: Option<Buffer> = None;
static mut SLOT1_BUF: Option<Buffer> = None;

static mut DMA_STREAM: MaybeUninit<dma::DmaStream<0>> = MaybeUninit::uninit();

pub struct AudioReader {
    pdm: Option<PdmInput>
}

// Just a way to enforce a singleton, so only one AudioReader can be created.
static mut THE_ONE_TRUE_READER: Option<AudioReader> = Some(AudioReader { pdm: None });

// Setup audio buffers by pushing them all into the ready to fill queue! 
fn initialize_audio_buffers() {
    let raw_buffers = unsafe { AUDIO_BUFFERS.as_mut() };
    let mut fill_producer = unsafe { READY_TO_FILL.split().0 };
    for buf in raw_buffers {
        info!("Initializing {:x}", core::ptr::addr_of_mut!(*buf) as *mut () as u32);
        fill_producer.enqueue(Buffer::new(core::ptr::addr_of_mut!(*buf))).unwrap();
    }
}

/// Read the overflow flag and clear it at the same time
pub fn read_dma_overflow_flag() -> bool {
    unsafe { DMA_OVERRUN.swap(false, Ordering::Relaxed) }
}

impl AudioReader {
    pub fn init(
        sai1: &'static device::sai4::RegisterBlock,
        dma1: &'static device::dma1::RegisterBlock,
    ) -> AudioReader {
        let reader = unsafe { THE_ONE_TRUE_READER.take() };
        // Panic now, befure re-initializing
        let mut reader = reader.unwrap();

        let mut pdm = PdmInput::new(sai1);
        pdm.init();
        pdm.enable_dma();
        
        let buf_addr = unsafe { core::ptr::addr_of!(AUDIO_BUFFERS) as *const () as u32 };
        info!("DMA_BUFFERS: {:x}", buf_addr);

        initialize_audio_buffers();

        dma::set_dma_request_mux(0, 87);
        let dmastream = dma::DmaStream::<0>::new(dma1);
        dmastream.set_psize(dma::DataSize::HalfWord);
        // Get two buffers to kick off DMA
        let mut rtf_consumer = unsafe { READY_TO_FILL.split().1 };
        let buf1 = rtf_consumer.dequeue().unwrap();
        let buf2 = rtf_consumer.dequeue().unwrap();

        dmastream.start_p2m_transfer(pdm.address(), buf1.ptr(), Some(buf2.ptr()), DMA_BUFFER_SIZE);
        dmastream.enable_interrupt();
        unsafe { SLOT0_BUF = Some(buf1); SLOT1_BUF = Some(buf2); };
        unsafe { DMA_STREAM.write(dmastream) };
        
        reader.pdm = Some(pdm);
        reader
    }

    /// Check if a filled buffer is available and return it if so
    /// Buffers must be returned via `return_buffer`.
    pub fn poll_for_data(&self) -> Option<Buffer> {
        let mut rts_consumer = unsafe { READY_TO_SEND.split().1 };
        rts_consumer.dequeue()
    }

    /// Return a buffer previously received via `poll_for_data`
    pub fn return_buffer(&self, buf: Buffer) {
        let mut rtf_producer = unsafe { READY_TO_FILL.split().0 };
        rtf_producer.enqueue(buf).unwrap();
    }
}

// IRQ goes off on completion of a transfer. We then swap out the inactive buffer
// for a for a fresh one, and pass the completed one off to the ready-to-send 
// queue for transmission
#[interrupt]
fn DMA1_STR0() {
    static mut FIRST_BUFFER_ACTIVE: bool = true;

    let stream = unsafe {DMA_STREAM.assume_init_mut()};
    stream.clear_interrupts();
    let mut rtf_consumer = unsafe { READY_TO_FILL.split().1 };
    let mut rts_producer = unsafe { READY_TO_SEND.split().0 };
    let next_buf = rtf_consumer.peek();
    if next_buf.is_none() {
        unsafe { DMA_OVERRUN.store(true, Ordering::Relaxed); }
        return;
    }
    if *FIRST_BUFFER_ACTIVE {
        if stream.get_current_target() == dma::TargetBuffer::Buffer0 {
            unsafe { DMA_OVERRUN.store(true, Ordering::Relaxed); }
            return
        }

        let completed_buffer = unsafe { SLOT0_BUF.take().unwrap() };
        rts_producer.enqueue(completed_buffer).unwrap();
        let next_buf = rtf_consumer.dequeue().unwrap();
        stream.load_memory0(next_buf.ptr());
        unsafe { SLOT0_BUF = Some(next_buf) };

        *FIRST_BUFFER_ACTIVE = false;
    } else {
        if stream.get_current_target() == dma::TargetBuffer::Buffer1 {
            unsafe { DMA_OVERRUN.store(true, Ordering::Relaxed); }
            return
        }

        let completed_buffer = unsafe { SLOT1_BUF.take().unwrap() };
        rts_producer.enqueue(completed_buffer).unwrap();
        let next_buf = rtf_consumer.dequeue().unwrap();
        stream.load_memory1(next_buf.ptr());
        unsafe { SLOT1_BUF = Some(next_buf) };
        *FIRST_BUFFER_ACTIVE = true;
    }
}