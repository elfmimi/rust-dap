// Copyright 2021 Kenta Ida
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#![no_std]
#![no_main]

mod line_coding;
// mod swdio_pin;
mod pio;

use defmt_rtt as _;
use panic_probe as _;

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    // use panic_halt as _;
    use defmt::println;

    use rust_dap::USB_CLASS_MISCELLANEOUS;
    use rust_dap::USB_PROTOCOL_IAD;
    use rust_dap::USB_SUBCLASS_COMMON;
    // use rust_dap::bitbang::*;
    use crate::pio::*;

    use rp_pico::hal;
    use hal::pac;
    use hal::gpio::{Pin, Output, PushPull};

    use hal::usb::UsbBus;
    use usb_device::bus::UsbBusAllocator;

    use rust_dap::CmsisDap;
    use usb_device::prelude::*;
    use usbd_serial::SerialPort;

    use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
    use embedded_hal::serial::{Read, Write};

/*
    type SwdIoPin = hal::gpio::bank0::Gpio4;
    type SwClkPin = hal::gpio::bank0::Gpio2;
    type SwdInputPin<P> = PicoSwdInputPin<P>;
    type SwdOutputPin<P> = PicoSwdOutputPin<P>;
    type SwdIoInputPin = SwdInputPin<SwdIoPin>;
    type SwdIoOutputPin = SwdOutputPin<SwdIoPin>;
    type SwClkInputPin = SwdInputPin<SwClkPin>;
    type SwClkOutputPin = SwdOutputPin<SwClkPin>;
    pub type MySwdIoSet = SwdIoSet<SwClkInputPin, SwClkOutputPin, SwdIoInputPin, SwdIoOutputPin, CycleDelay>;
*/    

    type SwClkPin = Pin<hal::gpio::bank0::Gpio2, hal::gpio::FunctionPio0>;
    type SwdIoPin = Pin<hal::gpio::bank0::Gpio4, hal::gpio::FunctionPio0>;
    pub type MySwdIoSet = SwdIoSet<SwClkPin, SwdIoPin, CycleDelay>;

    pub struct CycleDelay;
    impl DelayFunc for CycleDelay {
        fn cycle_delay(&self, cycles: u32) {
            cortex_m::asm::delay(cycles);
        }
    }

    // use crate::swdio_pin::*;
    use crate::line_coding::*;

    // UART Interrupt context
    const UART_RX_QUEUE_SIZE: usize = 256;
    const UART_TX_QUEUE_SIZE: usize = 128;
    // UART Shared context
    type UartPins = (
        hal::gpio::Pin<hal::gpio::bank0::Gpio0, hal::gpio::Function<hal::gpio::Uart>>,
        hal::gpio::Pin<hal::gpio::bank0::Gpio1, hal::gpio::Function<hal::gpio::Uart>>,
    );
    type Uart = hal::uart::UartPeripheral<hal::uart::Enabled, pac::UART0, UartPins>;
    pub struct UartReader(hal::uart::Reader<pac::UART0, UartPins>);
    pub struct UartWriter(hal::uart::Writer<pac::UART0, UartPins>);
    unsafe impl Sync for UartReader {}
    unsafe impl Sync for UartWriter {}
    unsafe impl Send for UartReader {}
    unsafe impl Send for UartWriter {}

    pub struct UartConfigAndClock {
        pub config: UartConfig,
        pub clock: embedded_time::rate::Hertz,
    }
    
    #[shared]
    struct Shared {
        uart_reader: Option<UartReader>,
        uart_writer: Option<UartWriter>,
        usb_serial: SerialPort<'static, UsbBus>,
        uart_rx_consumer: heapless::spsc::Consumer<'static, u8, UART_RX_QUEUE_SIZE>,
        uart_tx_producer: heapless::spsc::Producer<'static, u8, UART_TX_QUEUE_SIZE>,
        uart_tx_consumer: heapless::spsc::Consumer<'static, u8, UART_TX_QUEUE_SIZE>,
    }

    #[local]
    struct Local {
        uart_config: UartConfigAndClock,
        uart_rx_producer: heapless::spsc::Producer<'static, u8, UART_RX_QUEUE_SIZE>,
        usb_bus: UsbDevice<'static, UsbBus>,
        usb_dap: CmsisDap<'static, UsbBus, MySwdIoSet, 64>,
        usb_led: Pin<hal::gpio::bank0::Gpio25, Output<PushPull>>,
        idle_led: Pin<hal::gpio::bank0::Gpio17, Output<PushPull>>,
        debug_out: Pin<hal::gpio::bank0::Gpio6, Output<PushPull>>,
        debug_irq_out: Pin<hal::gpio::bank0::Gpio28, Output<PushPull>>,
        debug_usb_irq_out: Pin<hal::gpio::bank0::Gpio27, Output<PushPull>>,
    }

    #[init(local = [
        uart_rx_queue: heapless::spsc::Queue<u8, UART_RX_QUEUE_SIZE> = heapless::spsc::Queue::new(),
        uart_tx_queue: heapless::spsc::Queue<u8, UART_TX_QUEUE_SIZE> = heapless::spsc::Queue::new(),
        USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None,
        ])]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        println!("Hayon!");
        let mut resets = c.device.RESETS;
        let sio = hal::Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );
        
        let mut watchdog = hal::Watchdog::new(c.device.WATCHDOG);
        let clocks = hal::clocks::init_clocks_and_plls(
            rp_pico::XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let uart_pins = (
            pins.gpio0.into_mode::<hal::gpio::FunctionUart>(),  // TxD
            pins.gpio1.into_mode::<hal::gpio::FunctionUart>(),  // RxD
        );
        let uart_config = UartConfigAndClock {
            config: UartConfig::from(hal::uart::common_configs::_115200_8_N_1),
            clock: clocks.peripheral_clock.into(),
        };
        let mut uart = hal::uart::UartPeripheral::new(c.device.UART0, uart_pins, &mut resets)
            .enable(
                (&uart_config.config).into(),
                uart_config.clock,
            )
            .unwrap();
        // Enable RX interrupt. Note that TX interrupt is enabled when some TX data is available.
        uart.enable_rx_interrupt();
        let (uart_reader, uart_writer) = uart.split();
        let uart_reader = Some(UartReader(uart_reader));
        let uart_writer = Some(UartWriter(uart_writer));
        
        let usb_allocator = UsbBusAllocator::new(
            hal::usb::UsbBus::new(
                c.device.USBCTRL_REGS,
                c.device.USBCTRL_DPRAM,
                clocks.usb_clock,
                true,
                &mut resets,
        ));
        c.local.USB_ALLOCATOR.replace(usb_allocator);
        let usb_allocator = c.local.USB_ALLOCATOR.as_ref().unwrap();

        let swdio = MySwdIoSet::new(
            c.device.PIO0,
            pins.gpio2.into_mode::<hal::gpio::FunctionPio0>(),
            pins.gpio4.into_mode::<hal::gpio::FunctionPio0>(),
            CycleDelay,
            &mut resets,
        );

        let usb_serial = SerialPort::new(usb_allocator);
        let usb_dap = CmsisDap::new(usb_allocator, swdio);
        let usb_bus = UsbDeviceBuilder::new(usb_allocator, UsbVidPid(0x6666, 0x4444))
                .manufacturer("fugafuga.org")
                .product("CMSIS-DAP")
                .serial_number("test")
                .device_class(USB_CLASS_MISCELLANEOUS)
                .device_class(USB_SUBCLASS_COMMON)
                .device_protocol(USB_PROTOCOL_IAD)
                .composite_with_iads()
                .max_packet_size_0(64)
                .build();
        let usb_led = pins.led.into_push_pull_output();
        let (uart_rx_producer, uart_rx_consumer) = c.local.uart_rx_queue.split();
        let (uart_tx_producer, uart_tx_consumer) = c.local.uart_tx_queue.split();
        
        let mut debug_out = pins.gpio6.into_push_pull_output();
        debug_out.set_low().ok();
        let mut debug_irq_out = pins.gpio28.into_push_pull_output();
        debug_irq_out.set_low().ok();
        let mut debug_usb_irq_out = pins.gpio27.into_push_pull_output();
        debug_usb_irq_out.set_low().ok();

        pins.gpio7.into_floating_disabled();// Deassert nRESET pin.
        pins.gpio16.into_push_pull_output().set_high().ok();
        let mut idle_led = pins.gpio17.into_push_pull_output();
        idle_led.set_high().ok();
        (Shared {uart_reader, uart_writer, usb_serial, uart_rx_consumer, uart_tx_producer, uart_tx_consumer}, Local { uart_config, uart_rx_producer, usb_bus, usb_dap, usb_led, idle_led, debug_out, debug_irq_out, debug_usb_irq_out }, init::Monotonics())
    }

    #[idle(shared = [uart_writer, usb_serial, uart_rx_consumer, uart_tx_producer, uart_tx_consumer], local = [idle_led])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            (&mut c.shared.usb_serial, &mut c.shared.uart_tx_producer).lock(|usb_serial, uart_tx_producer| {
                while uart_tx_producer.ready() {
                    if let Ok(data) = read_usb_serial_byte_cs(usb_serial) {
                        uart_tx_producer.enqueue(data).unwrap();
                    } else {
                        break;
                    }
                }
            });
            (&mut c.shared.uart_writer, &mut c.shared.uart_tx_consumer).lock(|uart, uart_tx_consumer| {
                let uart = uart.as_mut().unwrap();
                while let Some(data) = uart_tx_consumer.peek() {
                    if uart.0.write(*data).is_ok() {
                        uart_tx_consumer.dequeue().unwrap();
                    } else {
                        break;
                    }
                }
            });
            
            // Process RX data.
            (&mut c.shared.usb_serial, &mut c.shared.uart_rx_consumer).lock(|usb_serial, uart_rx_consumer| {
                while let Some(data) = uart_rx_consumer.peek() {
                    match write_usb_serial_byte_cs(usb_serial, *data) {
                        Ok(_) => { let _ = uart_rx_consumer.dequeue().unwrap(); },
                        _ => break,
                    }
                }
                usb_serial.flush().ok();
            });

            c.local.idle_led.toggle().ok();
        }
    }

    #[task(
        binds = UART0_IRQ,
        priority = 2,
        shared = [uart_reader],
        local = [uart_rx_producer, debug_out, debug_irq_out],
    )]
    fn uart_irq(mut c: uart_irq::Context) {
        c.local.debug_irq_out.set_high().ok();
        while c.local.uart_rx_producer.ready() {
            if let Ok(data) = c.shared.uart_reader.lock(|uart| uart.as_mut().unwrap().0.read()) {
                c.local.debug_out.toggle().ok();
                let _ = c.local.uart_rx_producer.enqueue(data).ok();    // Enqueuing must not fail because we have already checked that the queue is ready to enqueue.
            } else {
                break
            }
        }
        c.local.debug_irq_out.set_low().ok();
    }

    #[task(
        binds = USBCTRL_IRQ,
        priority = 1,
        shared = [uart_reader, uart_writer, usb_serial, uart_rx_consumer, uart_tx_producer, uart_tx_consumer],
        local = [usb_bus, usb_dap, uart_config, usb_led, debug_usb_irq_out],
    )]
    fn usbctrl_irq(mut c: usbctrl_irq::Context) {
        c.local.debug_usb_irq_out.set_high().ok();

        let poll_result = c.shared.usb_serial.lock(|usb_serial| c.local.usb_bus.poll(&mut [usb_serial, c.local.usb_dap]));
        if !poll_result {
            c.local.debug_usb_irq_out.set_low().ok();
            return; // Nothing to do at this time...
        }
        // Process DAP commands.
        c.local.usb_dap.process().ok();

        // Process TX data.
        (&mut c.shared.usb_serial, &mut c.shared.uart_tx_producer).lock(|usb_serial, uart_tx_producer| {
            while uart_tx_producer.ready() {
                if let Ok(data) = read_usb_serial_byte_cs(usb_serial) {
                    uart_tx_producer.enqueue(data).unwrap();
                } else {
                    break;
                }
            }
        });
        (&mut c.shared.uart_writer, &mut c.shared.uart_tx_consumer).lock(|uart, uart_tx_consumer| {
            let uart = uart.as_mut().unwrap();
            while let Some(data) = uart_tx_consumer.peek() {
                if uart.0.write(*data).is_ok() {
                    uart_tx_consumer.dequeue().unwrap();
                } else {
                    break;
                }
            }
        });
    
        // Process RX data.
        (&mut c.shared.usb_serial, &mut c.shared.uart_rx_consumer).lock(|usb_serial, uart_rx_consumer| {
            while let Some(data) = uart_rx_consumer.peek() {
                match write_usb_serial_byte_cs(usb_serial, *data) {
                    Ok(_) => { 
                        let _ = uart_rx_consumer.dequeue().unwrap(); 
                    },
                    _ => break,
                }
            }
            usb_serial.flush().ok();
        });

        // Check if the UART transmitter must be re-configured.
        if let Ok(expected_config) = c.shared.usb_serial.lock(|usb_serial| UartConfig::try_from(usb_serial.line_coding().clone())) {
            let config = UartConfig::from(c.local.uart_config.config.clone());
            if expected_config != config {
                (&mut c.shared.uart_reader, &mut c.shared.uart_writer).lock(|reader, writer| {
                    reader.as_mut().unwrap().0.disable_rx_interrupt();
                    let disabled = Uart::join(reader.take().unwrap().0, writer.take().unwrap().0).disable();
                    let enabled = disabled.enable((&expected_config).into(), c.local.uart_config.clock).unwrap();
                    c.local.uart_config.config = expected_config.into();
                    let (new_reader, new_writer) = enabled.split();
                    reader.replace(UartReader(new_reader));
                    writer.replace(UartWriter(new_writer));
                    reader.as_mut().unwrap().0.enable_rx_interrupt();
                });
            }
        }
        
        c.local.usb_led.toggle().ok();
        c.local.debug_usb_irq_out.set_low().ok();
    }

    
    fn read_usb_serial_byte_cs(usb_serial: &mut SerialPort<UsbBus>) -> Result<u8, UsbError> {
        let mut buf = [0u8; 1];
        match usb_serial.read(&mut buf) {
            Ok(1) => Ok(buf[0]),
            Ok(0) => Err(UsbError::WouldBlock),
            Ok(_) => panic!("USB Serial read extra data."),
            Err(err) => Err(err),
        }
    }

    fn write_usb_serial_byte_cs(usb_serial: &mut SerialPort<UsbBus>, data: u8) -> Result<(), UsbError> {
        let buf = [data; 1];
        match usb_serial.write(&buf) {
            Ok(1) => Ok(()),
            Ok(0) => Err(UsbError::WouldBlock),
            Ok(_) => panic!("USB Serial wrote extra data."),
            Err(err) => Err(err),
        }
    }
}