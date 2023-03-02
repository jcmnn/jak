//! # Pico USB 'Twitchy' Mouse Example
//!
//! Creates a USB HID Class Pointing device (i.e. a virtual mouse) on a Pico
//! board, with the USB driver running in the main thread.
//!
//! It generates movement reports which will twitch the cursor up and down by a
//! few pixels, several times a second.
//!
//! See the `Cargo.toml` file for Copyright and license details.
//!
//! This is a port of
//! https://github.com/atsamd-rs/atsamd/blob/master/boards/itsybitsy_m0/examples/twitching_usb_mouse.rs

#![no_std]
#![no_main]

use core::sync::atomic::AtomicBool;

use cortex_m::prelude::*;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::PinState;

// The macro for our start-up function
use rp_pico::entry;

// The macro for marking our interrupt functions
use rp_pico::hal::gpio;
use rp_pico::hal::gpio::bank0::Gpio2;
use rp_pico::hal::gpio::bank0::Gpio3;
use rp_pico::hal::gpio::AnyPin;
use rp_pico::hal::gpio::Output;
use rp_pico::hal::gpio::Pin;
use rp_pico::hal::gpio::PullUp;
use rp_pico::hal::gpio::PushPull;
use rp_pico::hal::pac::interrupt;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

use usbd_hid::descriptor::KeyboardReport;
// USB Human Interface Device (HID) Class support
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::MouseReport;
use usbd_hid::hid_class::HIDClass;
use usbd_hid::hid_class::HidClassSettings;
use usbd_hid::hid_class::HidCountryCode;
use usbd_hid::hid_class::HidProtocol;
use usbd_hid::hid_class::HidSubClass;
use usbd_hid::hid_class::ProtocolModeConfig;

use embedded_hal::digital::v2::InputPin;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

static READY: AtomicBool = AtomicBool::new(false);

static mut BUTTON: Option<Pin<Gpio3, gpio::Input<PullUp>>> = None;
static mut LED: Option<Pin<Gpio2, Output<PushPull>>> = None;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then submits cursor movement
/// updates periodically.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led = pins.gpio2.into_push_pull_output();
    let mut button = pins.gpio3.into_pull_up_input();

    button.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);
    button.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);

    unsafe {
        LED = Some(led);
        BUTTON = Some(button);
    }

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::IO_IRQ_BANK0);
    };

    loop {
        delay.delay_ms(50);
        //led.set_state(PinState::from(button.is_low().unwrap()))
        //.unwrap();
    }
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn IO_IRQ_BANK0() {
    let led = LED.as_mut().unwrap();
    let button = BUTTON.as_mut().unwrap();
    led.set_state(PinState::from(button.is_low().unwrap())).unwrap();
}
