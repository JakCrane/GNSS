//! USB serial output in its own task, so the tracking loop never waits on the PC.
//!
//! The main loop drops text into OUT and carries on; `writer` sends it to the PC as
//! fast as the PC takes it.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering::Relaxed};
use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::Sender;

use super::driver::UsbDriver;

static OUT: Pipe<CriticalSectionRawMutex, 8192> = Pipe::new();
static DTR: AtomicBool = AtomicBool::new(false);
static DROPPED: AtomicU32 = AtomicU32::new(0);
static USB_ERRORS: AtomicU32 = AtomicU32::new(0);     // write_packet failures
static CLEARED: AtomicU32 = AtomicU32::new(0);        // queued bytes thrown away

/// A terminal has the port open (DTR set).
pub fn connected() -> bool {
    DTR.load(Relaxed)
}

/// Queue a line without waiting: the whole line, or (buffer full) nothing, counted.
/// For the 1 ms stream and status lines.
pub fn send(s: &str) {
    send_bytes(s.as_bytes());
}

/// `send` for raw bytes.
pub fn send_bytes(mut b: &[u8]) {
    if !connected() {
        return;
    }
    if OUT.free_capacity() < b.len() {
        DROPPED.fetch_add(1, Relaxed);
        return;
    }
    // try_write only fills the contiguous free space, so at the ring buffer's wrap point
    // it writes part of the line: keep going until it's all in. (Room for all of it was
    // checked above, and only this task writes, so this always finishes.)
    while !b.is_empty() {
        match OUT.try_write(b) {
            Ok(n) => b = &b[n..],
            Err(_) => break,
        }
    }
}

/// Queue a line, waiting for room. For replies and the capture dump, which mustn't be lost.
pub async fn send_wait(s: &str) {
    if connected() {
        OUT.write_all(s.as_bytes()).await;
    }
}

/// Lines dropped since the last call.
pub fn take_dropped() -> u32 {
    DROPPED.swap(0, Relaxed)
}

/// (write_packet errors, queued bytes discarded) since the last call.
pub fn take_errors() -> (u32, u32) {
    (USB_ERRORS.swap(0, Relaxed), CLEARED.swap(0, Relaxed))
}

fn clear_queue() {
    CLEARED.fetch_add(OUT.len() as u32, Relaxed);
    OUT.clear();
}

/// Owns the USB sender: moves queued bytes to the PC 64 at a time.
#[embassy_executor::task]
pub async fn writer(mut tx: Sender<'static, UsbDriver>) -> ! {
    let mut pkt = [0u8; 64];
    loop {
        tx.wait_connection().await;
        loop {
            let dtr = tx.dtr();
            DTR.store(dtr, Relaxed);
            if !dtr {
                clear_queue();              // nobody listening: don't send stale output later
            }
            // Wake for data, or every 20 ms to notice DTR changing.
            match select(OUT.read(&mut pkt), Timer::after_millis(20)).await {
                Either::First(n) => {
                    if tx.write_packet(&pkt[..n]).await.is_err() {
                        USB_ERRORS.fetch_add(1, Relaxed);
                        break;              // unplugged
                    }
                }
                Either::Second(()) => {}
            }
        }
        DTR.store(false, Relaxed);
        clear_queue();
    }
}