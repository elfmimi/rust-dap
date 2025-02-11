#![no_std]
#![no_main]

use rp235x_hal as hal;

use panic_halt as _;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const XTAL_FREQ_HZ: u32 = 12_000_000u32;


use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

/*

#[hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio25.into_push_pull_output();
    loop {
        led_pin.set_high().unwrap();
        timer.delay_ms(100);
        led_pin.set_low().unwrap();
        timer.delay_ms(300);
    }
}
*/

// Some things we need
use core::fmt::Write;
use heapless::String;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
// use usbd_serial::SerialPort;

/*
/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the rp235x peripherals, then writes to the UART in
/// an infinite loop.
#[hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USB,
        pac.USB_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let mut said_hello = false;
    loop {
        // A welcome message at the beginning
        if !said_hello && timer.get_counter().ticks() >= 2_000_000 {
            said_hello = true;
            let _ = serial.write(b"Hello, World!\r\n");

            let time = timer.get_counter().ticks();
            let mut text: String<64> = String::new();
            writeln!(&mut text, "Current timer ticks: {}", time).unwrap();

            // This only works reliably because the number of bytes written to
            // the serial port is smaller than the buffers available to the USB
            // peripheral. In general, the return value should be handled, so that
            // bytes not transferred yet don't get lost.
            let _ = serial.write(text.as_bytes());
        }

        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(count) => {
                    // Convert to upper case
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });
                    // Send back to the host
                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }
                }
            }
        }
    }
}
*/

use embedded_hal::digital::StatefulOutputPin;
use panic_halt as _;
use rust_dap::bitbang::{DelayFunc, SwdIoSet};
use rust_dap::{
    CmsisDap, DapCapabilities, USB_CLASS_MISCELLANEOUS, USB_PROTOCOL_IAD, USB_SUBCLASS_COMMON,
};

// use bsp::{entry, hal, pac};
// use hal::clock::GenericClockController;
// use hal::gpio::{Output, Pin, PushPull};
// use pac::{interrupt, CorePeripherals, Peripherals};
use hal::pac::interrupt;
// use xiao_m0 as bsp;

use usb_device::bus::UsbBusAllocator;
// use xiao_m0::hal::usb::UsbBus;
use hal::usb::UsbBus;

use usb_device::prelude::*;
use usbd_serial::SerialPort;

#[cfg(target_arch = "arm")]
use cortex_m::asm::delay as cycle_delay;
#[cfg(target_arch = "arm")]
use cortex_m::peripheral::NVIC;
#[cfg(target_arch = "arm")]
use cortex_m_rt::pre_init;


mod swdio_pin;
use swdio_pin::*;

// Import pin types.
// use hal::gpio::{PA02, PA05, PA07, PA18};

// type SwdIoPin = PA05; // D9
// type SwClkPin = PA07; // D8
// type ResetPin = PA02; // D0
type SwClkPin = hal::gpio::bank0::Gpio2;
type SwdIoPin = hal::gpio::bank0::Gpio3;
type ResetPin = hal::gpio::bank0::Gpio4;
// type SwdIoInputPin = XiaoSwdInputPin<SwdIoPin>;
// type SwdIoOutputPin = XiaoSwdOutputPin<SwdIoPin>;
// type SwClkInputPin = XiaoSwdInputPin<SwClkPin>;
// type SwClkOutputPin = XiaoSwdOutputPin<SwClkPin>;
// type ResetInputPin = XiaoSwdInputPin<ResetPin>;
// type ResetOutputPin = XiaoSwdOutputPin<ResetPin>;
type SwdIoInputPin = PicoSwdInputPin<SwdIoPin>;
type SwdIoOutputPin = PicoSwdOutputPin<SwdIoPin>;
type SwClkInputPin = PicoSwdInputPin<SwClkPin>;
type SwClkOutputPin = PicoSwdOutputPin<SwClkPin>;
type ResetInputPin = PicoSwdInputPin<ResetPin>;
type ResetOutputPin = PicoSwdOutputPin<ResetPin>;
type MySwdIoSet = SwdIoSet<
    SwClkInputPin,
    SwClkOutputPin,
    SwdIoInputPin,
    SwdIoOutputPin,
    ResetInputPin,
    ResetOutputPin,
    CycleDelay,
>;



struct CycleDelay {}
impl DelayFunc for CycleDelay {
    fn cycle_delay(&self, cycles: u32) {
        cortex_m::asm::delay(cycles);
    }
}



// #[entry]
#[hal::entry]
fn main() -> ! {
    // let mut peripherals = Peripherals::take().unwrap();
    // let mut core = CorePeripherals::take().unwrap();
    // let mut clocks = GenericClockController::with_internal_32kosc(
    //     peripherals.GCLK,
    //     &mut peripherals.PM,
    //     &mut peripherals.SYSCTRL,
    //     &mut peripherals.NVMCTRL,
    // );

    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();


    // let pins = bsp::Pins::new(peripherals.PORT);
    // let mut led0 = pins.led0.into_push_pull_output();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led0 = pins.gpio25.into_push_pull_output();

    // let bus_allocator = unsafe {
    //     USB_ALLOCATOR = Some(bsp::usb_allocator(
    //         peripherals.USB,
    //         &mut clocks,
    //         &mut peripherals.PM,
    //         pins.usb_dm,
    //         pins.usb_dp,
    //     ));
    //     USB_ALLOCATOR.as_ref().unwrap()
    // };

    // Set up the USB driver
    let bus_allocator = unsafe {
        USB_ALLOCATOR = Some(UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USB,
        pac.USB_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
        )));
        USB_ALLOCATOR.as_ref().unwrap()
    };

    // RESET pin of Cortex Debug 10-pin connector is negative logic
    // https://developer.arm.com/documentation/101453/0100/CoreSight-Technology/Connectors
    // let reset_pin = pins.a0.into_floating_input();
    let reset_pin = pins.gpio4.into_floating_input();

    let swdio = MySwdIoSet::new(
        // XiaoSwdInputPin::new(pins.a8.into_floating_input()),
        // XiaoSwdInputPin::new(pins.a9.into_floating_input()),
        // XiaoSwdInputPin::new(reset_pin),
        PicoSwdInputPin::new(pins.gpio2.into_floating_input()),
        PicoSwdInputPin::new(pins.gpio3.into_floating_input()),
        PicoSwdInputPin::new(reset_pin),
        CycleDelay {},
    );

    let usb_strings = StringDescriptors::default()
        .manufacturer("fugafuga.org")
        .product("CMSIS-DAP")
        .serial_number("test");

    unsafe {
        USB_SERIAL = Some(SerialPort::new(bus_allocator));
        USB_DAP = Some(CmsisDap::new(bus_allocator, swdio, DapCapabilities::SWD));
        USB_BUS = Some(
            UsbDeviceBuilder::new(bus_allocator, UsbVidPid(0x6666, 0x4444))
                .strings(&[usb_strings]).unwrap()
                .device_class(USB_CLASS_MISCELLANEOUS)
                .device_class(USB_SUBCLASS_COMMON)
                .device_protocol(USB_PROTOCOL_IAD)
                .composite_with_iads()
                .max_packet_size_0(64).unwrap()
                .build(),
        );
        // LED = Some(pins.led1.into_push_pull_output());
    }

    unsafe {
        // core.NVIC.set_priority(interrupt::USB, 1);
        // NVIC::unmask(interrupt::USB);
        #[cfg(target_arch = "arm")]
        NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    }

    loop {
        // unsafe {
        //     USB_DAP.as_mut().map(|dap| {
        //         let _ = dap.process();
        //     });
        // }
        cycle_delay(15 * 1024 * 1024);
        led0.toggle().ok();
    }
}



static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;
static mut USB_DAP: Option<CmsisDap<UsbBus, MySwdIoSet, 64>> = None;
// static mut LED: Option<Pin<PA18, Output<PushPull>>> = None;

fn poll_usb() {
    unsafe {
        if let Some(usb_dev) = USB_BUS.as_mut() {
            if let Some(serial) = USB_SERIAL.as_mut() {
                if let Some(dap) = USB_DAP.as_mut() {
                    usb_dev.poll(&mut [serial, dap]);

                    dap.process().ok();
                    let mut buf = [0u8; 64];
                    if let Ok(count) = serial.read(&mut buf) {
                        for (i, c) in buf.iter().enumerate() {
                            if i >= count {
                                break;
                            }
                            serial.write(&[*c]).unwrap();
                            // LED.as_mut().map(|led| led.toggle());
                        }
                    };
                }
            }
        }
    };
}

// #[interrupt]
// fn USB() {
//     poll_usb();
// }
#[interrupt]
fn USBCTRL_IRQ() {
    poll_usb();
}

#[pre_init]
unsafe fn pre_init() {
    // Switch to the high-frequency XOSC
    // rp235x_hal::clocks::switch_to_external_xtal();
    unsafe {
        const SIO_BASE: u32 = 0xd0000000;
        const SPINLOCK0_PTR: *mut u32 = (SIO_BASE + 0x100) as *mut u32;
        const SPINLOCK_COUNT: usize = 32;
        for i in 0..SPINLOCK_COUNT {
            SPINLOCK0_PTR.wrapping_add(i).write_volatile(1);
        }
        #[cfg(target_arch = "arm")]
        {
            // Enable the Double-Co-Pro and the GPIO Co-Pro in the CPACR register.
            // We have to do this early, before there's a chance we might call
            // any accelerated functions.
            const SCB_CPACR_PTR: *mut u32 = 0xE000_ED88 as *mut u32;
            const SCB_CPACR_FULL_ACCESS: u32 = 0b11;
            // Do a R-M-W, because the FPU enable is here and that's already been enabled
            let mut temp = SCB_CPACR_PTR.read_volatile();
            // DCP Co-Pro is 4, two-bits per entry
            temp |= SCB_CPACR_FULL_ACCESS << (4 * 2);
            // GPIO Co-Pro is 0, two-bits per entry
            temp |= SCB_CPACR_FULL_ACCESS << (0 * 2);
            SCB_CPACR_PTR.write_volatile(temp);
            // Don't allow any DCP code to be moved before this fence.
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        }
    }
}
