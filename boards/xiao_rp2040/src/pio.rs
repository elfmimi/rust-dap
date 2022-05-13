// Copyright 2021-2022 Kenta Ida
//           2022 Ein Terakawa
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

// use crate::cmsis_dap::*;
use rust_dap::*;
// use rust_dap::{SwdIo, SwdIoConfig, SwdRequest, DapError};
// use rust_dap::{DAP_TRANSFER_OK, DAP_TRANSFER_WAIT, DAP_TRANSFER_FAULT, DAP_TRANSFER_ERROR, DAP_TRANSFER_MISMATCH};
use defmt::println;
use rp_pico::hal;
use hal::pac;
use hal::pio::PIOExt;
use hal::gpio::{PinId, bank0};
mod pio0 {
    use rp_pico::hal::{self, gpio::FunctionPio0};
    pub type Pin<P> = hal::gpio::Pin<P, FunctionPio0>;
}

pub trait DelayFunc {
    fn cycle_delay(&self, cycles: u32);
}

pub struct SwdIoSet<C, D, DelayFn>
    where DelayFn: DelayFunc,
{
    // We only need these infomation at initialization in fn new() .
    // clk_pin_id: u8,
    // dat_pin_id: u8,
    cycle_delay: DelayFn,
    running_sm: Option<hal::pio::StateMachine<hal::pio::PIO0SM0, hal::pio::Running>>,
    rx_fifo: hal::pio::Rx<hal::pio::PIO0SM0>,
    tx_fifo: hal::pio::Tx<hal::pio::PIO0SM0>,
    transfer_count: u32,
    _pins: core::marker::PhantomData<(C,D)>,
}

/*
impl <C, D, DelayFn> SwdIoSet<C, D, DelayFn>
    where
    C: AnyPin,
    D: AnyPin,
    DelayFn: DelayFunc
{
    pub fn new(_: C, _: D, cycle_delay: DelayFn) -> Self{
        Self {
            cycle_delay: cycle_delay,
            transfer_count: 0,
            clk_pin_id: <C as AnyPin>::Id::DYN.num,
            dat_pin_id: <D as AnyPin>::Id::DYN.num,
            _pins: core::marker::PhantomData,
        }
    }
}
*/

impl <C, D, DelayFn> SwdIoSet<pio0::Pin<C>, pio0::Pin<D>, DelayFn>
    where
    C: PinId + bank0::BankPinId,
    D: PinId + bank0::BankPinId,
    DelayFn: DelayFunc
{
    pub fn new(pio0: pac::PIO0, _: pio0::Pin<C>, _: pio0::Pin<D>, cycle_delay: DelayFn, resets: &mut pac::RESETS) -> Self{
        let clk_pin_id = C::DYN.num;
        let dat_pin_id = D::DYN.num;

        let mut a = pio::Assembler::<{pio::RP2040_MAX_PROGRAM_SIZE}>::new();
        a.side_set = pio::SideSet::new(true, 1, false);
        let mut write_loop = a.label();
        let mut write_loop_enter = a.label();
        let mut read_start = a.label();
        let mut read_loop = a.label();
        let mut read_loop_enter = a.label();
        let mut wrap_target = a.label();
        // let mut wrap_source = a.label();
        const HI: u8 = 1;
        const LO: u8 = 0;
        const Q: u8 = 3 - 1; // delay
        // Set SWCLK pin as output.
        // a.set(pio::SetDestination::PINDIRS, 1);
        a.bind(&mut wrap_target);
        // Get number of bits and direction bit from FIFO to OSR.
        a.pull(false, true);
        // Move number of bits to X register.
        a.out(pio::OutDestination::X, 31);
        // Copy direction bit to Y register.
        a.mov(pio::MovDestination::Y, pio::MovOperation::None, pio::MovSource::OSR);
        // Y == 0 means Read , Y != 0 means Write
        a.jmp(pio::JmpCondition::YIsZero, &mut read_start);

        // Write-Bits
        // Get data from FIFO to OSR.
        a.pull(false, true);
        // We want to prepare output value before setting pin direction.
        a.out(pio::OutDestination::PINS, 1);
        // Use ISP as a temporary store.
        a.mov(pio::MovDestination::ISR, pio::MovOperation::None, pio::MovSource::OSR);
        // Y register has value 1. Copy that 1 to OSR.
        a.mov(pio::MovDestination::OSR, pio::MovOperation::None, pio::MovSource::Y);
        // Set IO direction of SWDIO pin to output.
        a.out(pio::OutDestination::PINDIRS, 1);
        // Restore rest of data from ISR to OSR.
        a.mov(pio::MovDestination::OSR, pio::MovOperation::None, pio::MovSource::ISR);
        // Jump if X register is not 0. with post decrement.
        a.jmp(pio::JmpCondition::XDecNonZero, &mut write_loop_enter);

        // We have updated state of SWDIO pin while keeping SWCLK static.
        // Start over.
        a.jmp(pio::JmpCondition::Always, &mut wrap_target);

        a.bind(&mut write_loop);
        // Output 1-bit. and set SWCLK to High.
        a.set(pio::SetDestination::PINS, HI);
        a.out_with_delay(pio::OutDestination::PINS, 1, match Q { 0 => 0, _ => Q - 1 });
        a.bind(&mut write_loop_enter);
        // Keep looping unless X register is 0. and set SWCLK to Low.
        a.jmp_with_delay_and_side_set(pio::JmpCondition::XDecNonZero, &mut write_loop, Q, LO);
        // Set SWCLK to High.
        a.set(pio::SetDestination::PINS, HI);
        // a.set_with_delay(pio::SetDestination::PINS, HI, Q);
        // Start over.
        a.jmp(pio::JmpCondition::Always, &mut wrap_target);

        // Read-Bits
        a.bind(&mut read_start);
        // Set IO direction of SWDIO pin to input.
        a.out_with_delay(pio::OutDestination::PINDIRS, 1, match Q { 0 => 0, _ => Q - 1 });
        // Jump if X register is not 0. with post decrement.
        a.jmp(pio::JmpCondition::XDecNonZero, &mut read_loop_enter);
        // Wait before reading SWDIO pin.
        a.nop_with_delay(Q);
        // Read state of SWDIO pin without clocking SWCLK.
        a.r#in(pio::InSource::PINS, 1);
        // Shift in 31 bits of 0 to MSB of ISR.
        a.r#in(pio::InSource::NULL, 31);
        // Put result data from ISR to FIFO.
        a.push(false, true);
        // Start over.
        a.jmp(pio::JmpCondition::Always, &mut wrap_target);

        // Set SWCLK to Low.
        // a.set(pio::SetDestination::PINS, LO);
        a.bind(&mut read_loop);
        // Shift-in 1-bit to ISR. and set SWCLK to High.
        a.r#in_with_delay_and_side_set(pio::InSource::PINS, 1, Q, HI);
        a.bind(&mut read_loop_enter);
        // Keep looping unless X register is 0. and set SWCLK to Low.
        a.jmp_with_delay_and_side_set(pio::JmpCondition::XDecNonZero, &mut read_loop, Q, LO);
        a.r#in_with_side_set(pio::InSource::PINS, 1, HI);
        // a.r#in_with_delay_and_side_set(pio::InSource::PINS, 1, Q, HI);
        // Put result data from ISR to FIFO.
        a.push(false, true);
        // a.bind(&mut wrap_source);
        // Start over.
        a.jmp(pio::JmpCondition::Always, &mut wrap_target);

        // The labels wrap_target and wrap_source, as set above,
        // define a loop which is executed repeatedly by the PIO
        // state machine.
        // let program = a.assemble_with_wrap(wrap_source, wrap_target);
        let program = a.assemble_program();
        let program = program.set_origin(Some(0));

        // Initialize and start PIO
        let (mut pio, sm0, _, _, _) = pio0.split(resets);
        let installed = pio.install(&program).unwrap();
        // let div = 25f32; // 125MHz / 25 = 5Mhz
        // let div = 5f32; // 125MHz / 5 = 25Mhz
        let div = 1f32; // 125MHz / 1 = 125Mhz
        let (mut sm, rx, tx) = hal::pio::PIOBuilder::from_program(installed)
            .set_pins(clk_pin_id, 1)
            .side_set_pin_base(clk_pin_id)
            .out_pins(dat_pin_id, 1)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .in_pin_base(dat_pin_id)
            .in_shift_direction(hal::pio::ShiftDirection::Right)
            .clock_divisor(div)
            .build(sm0);
        sm.set_pindirs([(clk_pin_id, hal::pio::PinDir::Output), (dat_pin_id, hal::pio::PinDir::Output)]);
        let running_sm = sm.start();

        Self {
            // clk_pin_id,
            // dat_pin_id,
            cycle_delay: cycle_delay,
            running_sm: Some(running_sm),
            rx_fifo: rx,
            tx_fifo: tx,
            transfer_count: 0,
            _pins: core::marker::PhantomData,
        }
    }
}

impl<C, D, DelayFn> BasicSwdIo for SwdIoSet<C, D, DelayFn>
    where DelayFn: DelayFunc
{
    // if bits > 32 , it will behave as if value is extended with zeros.
    fn write_bits(&mut self, bits: u32, value: u32) {
        while !self.tx_fifo.write(bits | 1 << 31) { }
        while !self.tx_fifo.write(value) { }
    }
    // if bits > 32 , result contains only the last 32 bits.
    fn read_bits(&mut self, bits: u32) -> u32 {
        while !self.tx_fifo.write(bits | 0) { }
        while self.rx_fifo.is_empty() { }
        let value = self.rx_fifo.read().unwrap();
        value >> ((32 - bits) & 31)
    }
    fn to_swdio_in(&mut self) {
        self.read_bits(0);
    }
    fn to_swdio_out(&mut self, output: bool) {
        self.write_bits(0, match output { true => 1, false => 0 });
    }
}


/*
impl <SwClkInputPin, SwClkOutputPin, SwdIoInputPin, SwdIoOutputPin, DelayFn> BitBangSwdIo for SwdIoSet<SwClkInputPin, SwClkOutputPin, SwdIoInputPin, SwdIoOutputPin, DelayFn>
    where SwClkInputPin: InputPin + IoPin<SwClkInputPin, SwClkOutputPin>,
          SwClkOutputPin: OutputPin + IoPin<SwClkInputPin, SwClkOutputPin>,
          SwdIoInputPin: InputPin + IoPin<SwdIoInputPin, SwdIoOutputPin>,
          SwdIoOutputPin: OutputPin + IoPin<SwdIoInputPin, SwdIoOutputPin>,
          DelayFn: DelayFunc
{
    fn to_swclk_in(&mut self) {
        let mut pin = None;
        core::mem::swap(&mut pin, &mut self.swclk_out);
        if let Some(swclk_out) = pin {
            self.swclk_in = Some(swclk_out.into_input_pin().unwrap_or_else(|_| panic!("Failed to turn SWCLK pin to input.")));
        }
    }
    fn to_swclk_out(&mut self, output: bool) {
        let mut pin = None;
        core::mem::swap(&mut pin, &mut self.swclk_in);
        if let Some(swclk_in) = pin {
            let state = if output { PinState::High } else { PinState::Low };
            self.swclk_out = Some(swclk_in.into_output_pin(state).unwrap_or_else(|_| panic!("Failed to turn SWCLK pin to output.")));
        }
    }
    fn to_swdio_in(&mut self) {
        let mut pin = None;
        core::mem::swap(&mut pin, &mut self.swdio_out);
        if let Some(swdio_out) = pin {
            self.swdio_in = Some(swdio_out.into_input_pin().unwrap_or_else(|_| panic!("Failed to turn SWDIO pin to input.")));
        }
    }
    fn to_swdio_out(&mut self, output: bool) {
        let mut pin = None;
        core::mem::swap(&mut pin, &mut self.swdio_in);
        if let Some(swdio_in) = pin {
            let state = if output { PinState::High } else { PinState::Low };
            self.swdio_out = Some(swdio_in.into_output_pin(state).unwrap_or_else(|_| panic!("Failed to turn SWDIO pin to output.")));
        }
    }
    fn set_swclk_output(&mut self, output: bool) {
        match unsafe { self.swclk_out.as_mut().unwrap_unchecked() } { p => {
            use embedded_hal::digital::v2::PinState::{ Low, High };
            p.set_state(if output { High } else { Low }).ok()
        }};
    }
    fn set_swdio_output(&mut self, output: bool) {
        match unsafe { self.swdio_out.as_mut().unwrap_unchecked() } { p => {
            use embedded_hal::digital::v2::PinState::{ Low, High };
            p.set_state(if output { High } else { Low }).ok()
        }};
    }
    fn get_swdio_input(&mut self) -> bool {
        self.swdio_in.as_ref().and_then(|p| {
            Some(p.is_high().unwrap_or(false))
        }).unwrap()
    }
    fn clock_wait(&self, config: &SwdIoConfig) {
        self.cycle_delay.cycle_delay(config.clock_wait_cycles);
    }
}
*/

/*
pub trait BitBangSwdIo {
    fn to_swclk_in(&mut self);
    fn to_swclk_out(&mut self, output: bool);
    fn to_swdio_in(&mut self);
    fn to_swdio_out(&mut self, output: bool);
    fn set_swclk_output(&mut self, output: bool);
    fn set_swdio_output(&mut self, output: bool);
    fn get_swdio_input(&mut self) -> bool;
    fn clock_wait(&self, config: &SwdIoConfig);
}
*/

pub trait BasicSwdIo {
    fn write_bits(&mut self, bits: u32, value: u32);
    fn read_bits(&mut self, bits: u32) -> u32;
    fn to_swdio_in(&mut self);
    fn to_swdio_out(&mut self, output: bool);
    // fn set_swdio_output(&mut self, output: bool);
}

pub trait PrimitiveSwdIo {
    fn connect(&mut self);
    fn disconnect(&mut self);
    fn enable_output(&mut self);
    fn disable_output(&mut self);

    fn cycle_clock(&mut self, config: &SwdIoConfig);
    fn turn_around(&mut self, config: &SwdIoConfig);
    fn idle_cycle(&mut self, config: &SwdIoConfig);
    fn write_bit(&mut self, config: &SwdIoConfig, value: bool);
    fn read_bit(&mut self, config: &SwdIoConfig) -> bool;
    fn set_swdio(&mut self, value: bool);
    fn get_timestamp(&mut self) -> u32;
}

/*
impl<Io: BitBangSwdIo> PrimitiveSwdIo for Io {
    fn connect(&mut self) {
        println!("connect!");
        self.to_swclk_out(false);
        self.to_swdio_out(false);
    }
    fn disconnect(&mut self) {
        println!("disconnect!");
        self.to_swclk_in();
        self.to_swdio_in();
    }
    fn enable_output(&mut self) {
        self.to_swdio_out(false);
    }
    fn disable_output(&mut self) {
        self.to_swdio_in();
    }

    fn cycle_clock(&mut self, config: &SwdIoConfig) {
        self.set_swclk_output(false);
        self.clock_wait(config);
        self.set_swclk_output(true);
        self.clock_wait(config);
    }
    fn turn_around(&mut self, config: &SwdIoConfig) {
        for _ in 0..config.turn_around_cycles {
            self.cycle_clock(config);
        }
    }
    fn idle_cycle(&mut self, config: &SwdIoConfig) {
        for _ in 0..config.idle_cycles {
            self.write_bit(config, false);
        }
    }
    fn write_bit(&mut self, config: &SwdIoConfig, value: bool) {
        self.set_swdio_output(value);
        self.set_swclk_output(false);
        self.clock_wait(config);
        self.set_swclk_output(true);
        self.clock_wait(config);
    }
    fn read_bit(&mut self, config: &SwdIoConfig) -> bool {
        self.set_swclk_output(false);
        self.clock_wait(config);
        let value = self.get_swdio_input();
        self.set_swclk_output(true);
        self.clock_wait(config);
        value
    }
    fn set_swdio(&mut self, value: bool) {
        self.set_swdio_output(value);
    }
    #[allow(dead_code)]
    fn get_timestamp(&mut self) -> u32 {
        0
    }
}
*/

/*
impl<Io: BitBangSwdIo> PrimitiveSwdIo for Io {
    fn connect(&mut self) {
        println!("connect!");
        // self.to_swclk_out(false);
        // self.to_swdio_out(false);
    }
    fn disconnect(&mut self) {
        println!("disconnect!");
        // self.to_swclk_in();
        // self.to_swdio_in();
    }
    fn enable_output(&mut self) {
        // self.to_swdio_out(false);
    }
    fn disable_output(&mut self) {
        // self.to_swdio_in();
    }

    fn cycle_clock(&mut self, config: &SwdIoConfig) {
        // self.set_swclk_output(false);
        // self.clock_wait(config);
        // self.set_swclk_output(true);
        // self.clock_wait(config);
    }
    fn turn_around(&mut self, config: &SwdIoConfig) {
        for _ in 0..config.turn_around_cycles {
            // self.cycle_clock(config);
        }
    }
    fn idle_cycle(&mut self, config: &SwdIoConfig) {
        for _ in 0..config.idle_cycles {
            // self.write_bit(config, false);
        }
    }
    fn write_bit(&mut self, config: &SwdIoConfig, value: bool) {
        // self.set_swdio_output(value);
        // self.set_swclk_output(false);
        // self.clock_wait(config);
        // self.set_swclk_output(true);
        // self.clock_wait(config);
    }
    fn read_bit(&mut self, config: &SwdIoConfig) -> bool {
        // self.set_swclk_output(false);
        // self.clock_wait(config);
        // let value = self.get_swdio_input();
        // self.set_swclk_output(true);
        // self.clock_wait(config);
        // value
        false
    }
    fn set_swdio(&mut self, value: bool) {
        // self.set_swdio_output(value);
    }
    #[allow(dead_code)]
    fn get_timestamp(&mut self) -> u32 {
        0
    }
}
*/

impl<C, D, DelayFn> PrimitiveSwdIo for SwdIoSet<C, D, DelayFn>
    where DelayFn: DelayFunc
{
    fn connect(&mut self) {
        println!("connect!");
        // self.to_swclk_out(false);
        // self.to_swdio_out(false);
    }
    fn disconnect(&mut self) {
        println!("disconnect!");
        // self.to_swclk_in();
        // self.to_swdio_in();
    }
    fn enable_output(&mut self) {
        self.to_swdio_out(false);
    }
    fn disable_output(&mut self) {
        self.to_swdio_in();
    }

    fn cycle_clock(&mut self, _config: &SwdIoConfig) {
        self.read_bits(1);
    }
    fn turn_around(&mut self, config: &SwdIoConfig) {
        self.read_bits(config.turn_around_cycles);
    }
    fn idle_cycle(&mut self, config: &SwdIoConfig) {
        if config.idle_cycles != 0 {
            self.write_bits(config.idle_cycles, 0);
        }
    }
    fn write_bit(&mut self, _config: &SwdIoConfig, value: bool) {
        self.write_bits(1, match value { true => 1, false => 0 });
    }
    fn read_bit(&mut self, _config: &SwdIoConfig) -> bool {
        self.read_bits(1) != 0
    }
    fn set_swdio(&mut self, _value: bool) {
        // self.set_swdio_output(value);
    }
    #[allow(dead_code)]
    fn get_timestamp(&mut self) -> u32 {
        0
    }
}

/*
impl<Io: PrimitiveSwdIo> SwdIo for Io {
    fn connect(&mut self) {
        PrimitiveSwdIo::connect(self)
    }
    fn disconnect(&mut self) {
        PrimitiveSwdIo::disconnect(self)
    }
    fn swj_sequence(&mut self, config: &SwdIoConfig, count: usize, data: &[u8]) {
        let mut index = 0;
        let mut value = 0;
        let mut bits = 0;
        let mut count = count;

        while count > 0 {
            count -= 1;
            if bits == 0 {
                value = data[index];
                index += 1;
                bits = 8;
            }
            self.write_bit(config, value & 1 != 0);
            value >>= 1;
            bits -= 1;
        }
    }
    fn swd_read_sequence(&mut self, config: &SwdIoConfig, count: usize, data: &mut [u8]) {
        let mut count = count;
        let mut index = 0;
        while count > 0 {
            let mut value = 0;
            let mut bits = 8;
            while bits > 0 && count > 0 {
                bits -= 1;
                count -= 1;

                let bit_value = self.read_bit(config);
                value = if bit_value {
                    (value >> 1) | 0x80
                } else {
                    value >> 1
                };
            }
            value >>= bits;
            data[index] = value;
            index += 1;
        }
    }

    fn swd_write_sequence(&mut self, config: &SwdIoConfig, count: usize, data: &[u8]) {
        let mut count = count;
        let mut index = 0;
        while count > 0 {
            let mut value = data[index];
            index += 1;
            let mut bits = 8;
            while bits > 0 && count > 0 {
                bits -= 1;
                count -= 1;

                self.write_bit(config, value & 1 != 0);
                value >>= 1;
            }
        }
    }

    fn swd_transfer(
        &mut self,
        config: &SwdIoConfig,
        request: SwdRequest,
        data: u32,
    ) -> core::result::Result<u32, DapError> {
        // write request
        self.enable_output();
        {
            let mut parity = false;
            self.write_bit(config, true); // Start
            let bit = request.contains(SwdRequest::APnDP);
            self.write_bit(config, bit);
            parity ^= bit;
            let bit = request.contains(SwdRequest::RnW);
            self.write_bit(config, bit);
            parity ^= bit;
            let bit = request.contains(SwdRequest::A2);
            self.write_bit(config, bit);
            parity ^= bit;
            let bit = request.contains(SwdRequest::A3);
            self.write_bit(config, bit);
            parity ^= bit;
            self.write_bit(config, parity); // Parity
            self.write_bit(config, false); // Stop
            self.write_bit(config, true); // Park
        }

        // turnaround + read ack.
        self.disable_output();
        self.turn_around(config);
        let ack = {
            let mut ack = 0u8;
            ack |= if self.read_bit(config) { 0b001 } else { 0b000 };
            ack |= if self.read_bit(config) { 0b010 } else { 0b000 };
            ack |= if self.read_bit(config) { 0b100 } else { 0b000 };
            ack
        };
        if ack == DAP_TRANSFER_OK {
            let ack = if request.contains(SwdRequest::RnW) {
                // READ request
                let mut value = 0u32;
                let mut parity = false;
                for _ in 0..32 {
                    let bit = self.read_bit(config);
                    parity ^= bit;
                    value = (value >> 1) | if bit { 0x80000000 } else { 0x00000000 };
                }
                let parity_expected = self.read_bit(config);
                self.turn_around(config);
                self.enable_output();
                if parity == parity_expected {
                    Ok(value)
                } else {
                    Err(DapError::SwdError(DAP_TRANSFER_MISMATCH))
                }
            } else {
                // WRITE request
                self.turn_around(config);
                self.enable_output();
                let mut value = data;
                let mut parity = false;
                for _ in 0..32 {
                    let bit = value & 1 != 0;
                    self.write_bit(config, bit);
                    parity ^= bit;
                    value >>= 1;
                }
                self.write_bit(config, parity);
                Ok(0)
            };
            // TODO: capture timestamp
            self.idle_cycle(config);
            self.set_swdio(true);
            return ack;
        }

        // An error occured.
        if ack == DAP_TRANSFER_WAIT || ack == DAP_TRANSFER_FAULT {
            self.disable_output();
            if config.always_generate_data_phase && request.contains(SwdRequest::RnW) {
                for _ in 0..33 {
                    self.cycle_clock(config);
                }
            }
            self.turn_around(config);
            self.enable_output();
            if config.always_generate_data_phase && !request.contains(SwdRequest::RnW) {
                for _ in 0..33 {
                    self.write_bit(config, false);
                }
            }
            self.set_swdio(true);
            return Err(DapError::SwdError(ack));
        }

        // Protocol error
        self.turn_around(config);
        for _ in 0..33 {
            self.cycle_clock(config);
        }
        self.enable_output();
        self.set_swdio(true);
        return Err(DapError::SwdError(ack));
    }

    fn enable_output(&mut self) {
        PrimitiveSwdIo::enable_output(self);
    }

    fn disable_output(&mut self) {
        PrimitiveSwdIo::disable_output(self);
    }
}
*/


impl<C, D, DelayFn> SwdIo for SwdIoSet<C, D, DelayFn>
    where DelayFn: DelayFunc
{
    fn connect(&mut self) {
        self.transfer_count = 0;
        PrimitiveSwdIo::connect(self)
    }
    fn disconnect(&mut self) {
        PrimitiveSwdIo::disconnect(self)
    }
    fn swj_sequence(&mut self, config: &SwdIoConfig, count: usize, data: &[u8]) {
        let mut index = 0;
        let mut value = 0;
        let mut bits = 0;
        let mut count = count;

        while count > 0 {
            count -= 1;
            if bits == 0 {
                value = data[index];
                index += 1;
                bits = 8;
            }
            self.write_bit(config, value & 1 != 0);
            value >>= 1;
            bits -= 1;
        }
    }
    fn swd_read_sequence(&mut self, config: &SwdIoConfig, count: usize, data: &mut [u8]) {
        let mut count = count;
        let mut index = 0;
        while count > 0 {
            let mut value = 0;
            let mut bits = 8;
            while bits > 0 && count > 0 {
                bits -= 1;
                count -= 1;

                let bit_value = self.read_bit(config);
                value = if bit_value {
                    (value >> 1) | 0x80
                } else {
                    value >> 1
                };
            }
            value >>= bits;
            data[index] = value;
            index += 1;
        }
    }
    fn swd_write_sequence(&mut self, config: &SwdIoConfig, count: usize, data: &[u8]) {
        // count > 0 is assured
        let mut count: u32 = count as u32;
        let mut index = 0;
        let mut bits = 0;
        let mut value = 0;
        loop {
            value |= (data[index] as u32) << bits;
            index += 1;
            bits += 8;
            if count <= bits {
                bits = count;
                self.write_bits(bits, value);
                break;
            }
            if bits == 32 {
                self.write_bits(bits, value);
                count -= bits;
                bits = 0;
                value = 0;
            }
        }
    }
    fn swd_transfer(
        &mut self,
        config: &SwdIoConfig,
        request: SwdRequest,
        data: u32,
    ) -> core::result::Result<u32, DapError> {
        if self.transfer_count % 1000 == 0 {
            // println!("swd_transfer! {0}", self.transfer_count);
        }
        self.transfer_count += 1;
        // write request
        // self.enable_output();
        // SwdIo::enable_output(self);
        // PrimitiveSwdIo::enable_output(self);
        {
            let mut bits = request.bits() & 0b1111;
            bits |= (bits.count_ones() as u8 & 1) << 4;
            bits = (bits << 1) | 0x81;
            self.write_bits(8, bits as u32);
        }

        // turnaround + read ack. ( + turnaround, for write request )
        // self.disable_output();
        let ack =
            if request.contains(SwdRequest::RnW) {
                self.read_bits(3 + config.turn_around_cycles) as u8 >> config.turn_around_cycles & 0b111
            } else {
                self.read_bits(3 + config.turn_around_cycles * 2) as u8 >> config.turn_around_cycles & 0b111
            };
        if ack == DAP_TRANSFER_OK {
            let ack = if request.contains(SwdRequest::RnW) {
                // READ request
                let value = self.read_bits(32);
                let parity = value.count_ones() & 1;
                // let parity_expected = self.read_bit(config);
                // self.turn_around(config);
                let parity_expected = self.read_bits(1 + config.turn_around_cycles) & 1;
                // self.enable_output();
                if parity == parity_expected {
                    Ok(value)
                } else {
                    Err(DapError::SwdError(DAP_TRANSFER_MISMATCH))
                }
            } else {
                // WRITE request
                // self.turn_around(config);
                // self.enable_output();
                self.write_bits(32, data);
                self.write_bits(1, data.count_ones());
                Ok(0)
            };
            // TODO: capture timestamp
            self.idle_cycle(config);
            // self.set_swdio(true);
            self.to_swdio_out(true);
            return ack;
        }

        // An error occured.
        if ack == DAP_TRANSFER_WAIT || ack == DAP_TRANSFER_FAULT {
            // self.disable_output();
            // SwdIo::disable_output(self);
            PrimitiveSwdIo::disable_output(self);
            if config.always_generate_data_phase && request.contains(SwdRequest::RnW) {
/*
                for _ in 0..33 {
                    self.cycle_clock(config);
                }
*/
                self.read_bits(33);
            }
            self.turn_around(config);
            // self.enable_output();
            if config.always_generate_data_phase && !request.contains(SwdRequest::RnW) {
/*
                for _ in 0..33 {
                    self.write_bit(config, false);
                }
*/
                self.write_bits(33, 0);
            }
            // self.set_swdio(true);
            self.to_swdio_out(true);
            return Err(DapError::SwdError(ack));
        }

        // Protocol error
        // self.turn_around(config);
/*
        for _ in 0..33 {
            self.cycle_clock(config);
        }
*/
        self.read_bits(33);
        if request.contains(SwdRequest::RnW) {
            self.turn_around(config);
        }
        // self.enable_output();
        // self.set_swdio(true);
        self.to_swdio_out(true);
        return Err(DapError::SwdError(ack));
    }

    fn enable_output(&mut self) {
        PrimitiveSwdIo::enable_output(self);
    }

    fn disable_output(&mut self) {
        PrimitiveSwdIo::disable_output(self);
    }
}
