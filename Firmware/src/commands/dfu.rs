//! Reboot into the STM32's built-in USB DFU bootloader, no BOOT button needed.
//!
//! The request flag lives in DTCM, not normal RAM. Normal RAM (AXI SRAM) has
//! 64-bit ECC, so a 32-bit write waits in a buffer and is lost on reset. DTCM has
//! 32-bit ECC, isn't used by the program, and survives a reset.

const DFU_FLAG: *mut u32 = 0x2001_FFF0 as *mut u32; // last words of DTCM
const DFU_MAGIC: u32 = 0xB007_DF00;
const SYSTEM_BOOTLOADER: u32 = 0x1FF0_9800; // AN2606: STM32H74x/75x system memory

/// Call this first in main(): if a DFU reboot was requested, jump into the bootloader.
pub fn jump_if_requested() {
    unsafe {
        if core::ptr::read_volatile(DFU_FLAG) == DFU_MAGIC {
            core::ptr::write_volatile(DFU_FLAG, 0);
            cortex_m::asm::dsb();
            cortex_m::asm::bootload(SYSTEM_BOOTLOADER as *const u32);
        }
    }
}

/// Leave a note for the next boot, then reset.
pub fn reboot() -> ! {
    unsafe { core::ptr::write_volatile(DFU_FLAG, DFU_MAGIC) };
    cortex_m::asm::dsb();
    cortex_m::peripheral::SCB::sys_reset();
}