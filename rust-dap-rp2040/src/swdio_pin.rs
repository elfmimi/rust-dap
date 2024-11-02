// Copyright 2022 Kenta Ida
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

use embedded_hal::digital::{ErrorType, InputPin, OutputPin, PinState};
use rust_dap::bitbang::IoPin;
use hal::gpio::{Pin, PinId, ValidFunction, FunctionSioInput, FunctionSioOutput, PullUp, PullNone};
use rp2040_hal as hal;

/// InputPin implementation for SWD pin
pub struct PicoSwdInputPin<I>
where
    I: PinId,
{
    pin: Pin<I, FunctionSioInput, PullNone>,
}

impl<I> ErrorType for PicoSwdInputPin<I>
where
    I:  PinId + ValidFunction<FunctionSioInput>,
{
    type Error = core::convert::Infallible;
}

impl<I> PicoSwdInputPin<I>
where
    I: PinId + ValidFunction<FunctionSioInput>,
{
    pub fn new(pin: Pin<I, FunctionSioInput, PullNone>) -> Self {
        Self { pin }
    }
}

impl<I> InputPin for PicoSwdInputPin<I>
where
    I: PinId + ValidFunction<FunctionSioInput>,
{
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        self.pin.is_high()
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        self.pin.is_low()
    }
}

impl<I> IoPin<PicoSwdInputPin<I>, PicoSwdOutputPin<I>> for PicoSwdInputPin<I>
where
    I: PinId + ValidFunction<FunctionSioInput> + ValidFunction<FunctionSioOutput>,
{
    fn into_input_pin(self) -> Result<PicoSwdInputPin<I>, Self::Error> {
        Ok(self)
    }
    fn into_output_pin(self, state: PinState) -> Result<PicoSwdOutputPin<I>, Self::Error> {
        let output_pin = self.pin.into_push_pull_output_in_state(state);
        Ok(PicoSwdOutputPin::new(output_pin.into_pull_type()))
    }
}

/// OutputPin implementation for SWD pin
pub struct PicoSwdOutputPin<I>
where
    I: PinId,
{
    pin: Pin<I, FunctionSioOutput, PullNone>,
}

impl<I> ErrorType for PicoSwdOutputPin<I>
where
    I:  PinId + ValidFunction<FunctionSioOutput>,
{
    type Error = core::convert::Infallible;
}

impl<I> PicoSwdOutputPin<I>
where
    I: PinId + ValidFunction<FunctionSioOutput>,
{
    pub fn new(pin: Pin<I, FunctionSioOutput, PullNone>) -> Self {
        Self { pin }
    }
}

impl<I> OutputPin for PicoSwdOutputPin<I>
where
    I: PinId + ValidFunction<FunctionSioOutput>,
{
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high()
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low()
    }
    fn set_state(&mut self, state: PinState) -> Result<(), Self::Error> {
        self.pin.set_state(state)
    }
}

impl<I> IoPin<PicoSwdInputPin<I>, PicoSwdOutputPin<I>> for PicoSwdOutputPin<I>
where
    I: PinId + ValidFunction<FunctionSioInput> + ValidFunction<FunctionSioOutput>,
{
    fn into_input_pin(self) -> Result<PicoSwdInputPin<I>, Self::Error> {
        // let input_pin = self.pin.into_floating_input();
        // Ok(PicoSwdInputPin::new(input_pin))
        Ok(PicoSwdInputPin::new(self.pin.reconfigure()))
    }
    fn into_output_pin(mut self, state: PinState) -> Result<PicoSwdOutputPin<I>, Self::Error> {
        self.set_state(state)?;
        Ok(self)
    }
}

/*
/// Pico SWD pin
pub struct PicoSwdPin<I>
where
    I: PinId,
{
    pin: Pin<I, hal::gpio::FunctionNull, hal::gpio::PullBusKeep>,
}

impl<I> IoPin<PicoSwdInputPin<I>, PicoSwdOutputPin<I>> for PicoSwdPin<I>
where
    I: PinId + ValidFunction<FunctionSioInput> + ValidFunction<FunctionSioOutput>,
{
    type Error = core::convert::Infallible;
    fn into_input_pin(self) -> Result<PicoSwdInputPin<I>, Self::Error> {
        let input_pin = self.pin.into_floating_input();
        Ok(PicoSwdInputPin::new(input_pin))
    }
    fn into_output_pin(self, state: PinState) -> Result<PicoSwdOutputPin<I>, Self::Error> {
        let output_pin = self.pin.into_push_pull_output_in_state(state);
        Ok(PicoSwdOutputPin::new(output_pin))
    }
}
*/