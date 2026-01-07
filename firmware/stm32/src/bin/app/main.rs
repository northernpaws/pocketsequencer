#![no_std]
#![no_main]

#[cfg(feature = "alloc")]
extern crate alloc;

mod engine;
mod interface;
mod project;

use defmt::*;

use cortex_m_rt::{ExceptionFrame, exception};

use embassy_executor::Spawner;

use embassy_stm32::rcc::{self};

use embassy_time::Timer;

use {
    // defmt_rtt as _,
    panic_probe as _,
};

use firmware::{
    hardware::{self, preamble::*},
    split_resources,
};

// NOTE: We try to rely on heap allocations as little
// as possible for core elements of the system.
//
// Where possible, core systems should use heapless static
// allocation or allocation pools so their size is known
// and we avoid running into OOM problems.
#[cfg(feature = "alloc")]
use embedded_alloc::LlffHeap as Heap;
#[cfg(feature = "alloc")]
#[global_allocator]
static HEAP: Heap = Heap::empty();

// TODO: add general exception handlers with keypad access to bink error LEDs.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("program start!");
    // If it returns, something went wrong.
    if let Err(err) = inner_main(spawner).await {
        defmt::panic!("{}", err);
    }
}

// Inner-main allows us us to returns errors via Result instead of relying on panics.
#[expect(
    clippy::future_not_send,
    reason = "Safe in single-threaded, bare-metal embedded context"
)]
#[expect(clippy::items_after_statements, reason = "Keeps related code together")]
async fn inner_main(spawner: Spawner) -> Result<(), ()> {
    // Initialize the ARM Cortex core and STM32 RCC.
    info!("initializing clocks and PLL to 480Mhz");
    let p = hardware::init_peripherals();

    // Segment the peripherals into organized
    // blocks based on their hardware components.
    info!("getting handles to hardware peripherals");
    let r = split_resources!(p);

    // Initialize the hardware components using the segmented resources.
    let mut hw = hardware::init_hardware(r, spawner).await?;

    // Debug logging of the current clock configuration.
    let clocks = rcc::clocks(&p.RCC);
    println!("System clock speed: {}", clocks.sys);
    println!("APB1  clock speed: {}", clocks.pclk1);
    println!("APB1 Timer clock speed: {}", clocks.pclk1_tim);
    println!("APB2  clock speed: {}", clocks.pclk2);
    println!("APB2 Timer clock speed: {}", clocks.pclk2_tim);

    // After initializing the external SDRAM, we can allocate the heap on it.
    let _sdram: &'static mut [u32] = {
        info!("initializing heap on SDRAM...");

        const HEAP_SIZE: usize = 1024 * 1024 * 16;

        // Allocate the heap pointer on the memory-mapped external SDRAM.
        unsafe { HEAP.init(&raw mut hw.sdram as usize, HEAP_SIZE) };

        // "Resize" the array pointer we have to the SDRAM to avoid the allocated heap block.
        unsafe {
            core::slice::from_raw_parts_mut(
                hw.sdram.as_mut_ptr(),
                (hw.sdram.len() - HEAP_SIZE) / core::mem::size_of::<u32>(),
            )
        }
    };

    info!("starting engine...");
    let (drive) = engine::start_engine(
        spawner,
        hw.sd_card,
        hw.internal_storage,
        hw.audio,
        hw.sai_resources,
        hw.usb_driver,
    )
    .await;

    info!("starting interface...");
    // todo

    info!("starting main loop...");
    loop {
        info!("main tick");
        Timer::after_millis(300).await;
    }

    Ok(())
}

/// Custom HardFault handler to improve debugging.
///
/// ref: https://doc.rust-lang.org/beta/embedded-book/start/exceptions.html#the-hard-fault-handler
#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    // Check `SCB -> CFSR_UFSR_BFSR_MMFSR` register
    // to see the fault and then reference:
    //  https://community.st.com/t5/stm32-mcus/how-to-debug-a-hardfault-on-an-arm-cortex-m-stm32/ta-p/672235#toc-hId--1471762501

    error!("HARD FAULT:");
    error!("r0  = 0x{:X}", ef.r0());
    error!("r1  = 0x{:X}", ef.r1());
    error!("r2  = 0x{:X}", ef.r2());
    error!("r3  = 0x{:X}", ef.r3());
    error!("r12 = 0x{:X}", ef.r12());
    error!("lr  = 0x{:X}", ef.lr());

    if (ef.lr() & 0x8) == 1 {
        error!("LR indicates Main Stack Pointer (MSP) TODO double check this is correct")
    } else {
        error!("LR indicates Process Stack Pointer (PSP) TODO double check this is correct")
    }

    // cargo objdump -- -d --no-show-raw-insn --print-imm-hex | grep <PC_HEX_WITHOUT_0x> -A 10 -B 10
    let pc = ef.pc();
    error!("pc: 0x{:X}", pc);
    error!("  Inspect assembled program with:");
    error!(
        "  $ cargo objdump -- -d --no-show-raw-insn --print-imm-hex | grep {:x} -A 10 -B 10",
        pc
    );
    error!("xpsr: {:b}", ef.xpsr());

    // Attempt to read the cortex registers to get more info.
    unsafe {
        let mut core_peri: cortex_m::Peripherals = cortex_m::Peripherals::steal();
        let scb = &mut core_peri.SCB;
        error!("hfsr: {:b}", scb.hfsr.read());

        // Chek HFSR bit 30 to see if we need to check other registers for details.
        if (scb.hfsr.read() & (1 << 30)) != 0 {
            error!("Forced Hard Fault!");
            error!(" HFSR FORCED bit set, see CFSR for details.");

            // NOTE: CFSR contains the UFSR, BFSR, and MMFSR registers.
            error!("  SCB->CFSR: {:b}", scb.cfsr.read());

            // Mask out the UFSR section of the CFSR register (first half).
            if (scb.cfsr.read() & 0xFFFF0000) != 0 {
                let ufsr: u16 = (scb.cfsr.read() >> 16) as u16;
                error!("  Usage Fault Indicator set!");
                error!("    SCB->CFSR->UFSR={:b}", ufsr);

                // TODO: error messages https://developer.arm.com/documentation/dui0552/a/cortex-m3-peripherals/system-control-block/configurable-fault-status-register
            }

            // BFSR
            if (scb.cfsr.read() & 0xFF00) != 0 {
                error!("  Bus Fault Indicator set!");

                let bsfr = (scb.cfsr.read() & 0xFF00) >> 8;
                error!("    SCB->CSFR->BSFR={:b}", bsfr);

                if (bsfr & 0b1) != 0 {
                    // 0
                    error!("    IBUSERR: Instruction bus error.")
                } else if (bsfr & 0b10) != 0 {
                    // 1
                    error!("    PRECISERR: Precise data bus error.");
                    error!("      Offending address (BFAR): 0x{:x}", scb.bfar.read());
                } else if (bsfr & 0b100) != 0 {
                    // 2
                    error!("    IMPRECISERR: Imprecise data bus error.");
                    error!("      Due to an Imprecise data bus error, the return address in the ");
                    error!("       stack does not point to the instruction that caused the fault!");
                } else if (bsfr & 0b1000) != 0 {
                    // 3
                    error!("    UNSTKERR: BusFault on unstacking for a return from exception.");
                } else if (bsfr & 0b10000) != 0 {
                    // 4
                    error!("    STKERR: BusFault on stacking for exception entry.");
                } else {
                    error!("    Unknown bus fault! See register for details.");
                }
            }

            // MMFSR
            if (scb.cfsr.read() & 0xFF) != 0 {
                error!("  Memory Management Indicator set!");
                error!("    Memory fault location: 0x{:X}", scb.mmfar.read());
                // TODO: error messages https://developer.arm.com/documentation/dui0552/a/cortex-m3-peripherals/system-control-block/configurable-fault-status-register
            }
        }
    }

    // TODO: attempt hard fault recovery https://github.com/kendiser5 000/Hardfault-Recovery-Cortex-M4/blob/master/hardfault_handler.c

    loop {}
}
