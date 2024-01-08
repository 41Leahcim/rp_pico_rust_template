#![no_std]
#![no_main]

use core::fmt::Write as _;

use embedded_hal::digital::v2::ToggleableOutputPin;
use heapless::String;
use panic_halt as _;
use rp_pico::{hal, pac};
use usb_device::{
    class_prelude::UsbBusAllocator,
    prelude::{UsbDeviceBuilder, UsbVidPid},
};
use usbd_serial::SerialPort;

#[rp_pico::entry]
fn main() -> ! {
    // Take control of the peripherals
    let mut pac = pac::Peripherals::take().unwrap();

    // Create a struct for controlling the watchdog
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Initialize the clocks
    let clock = hal::clocks::init_clocks_and_plls(
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

    // Create a struct for controlling the SIO
    let sio = hal::Sio::new(pac.SIO);

    // Create a struct for controlling the pins
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Create a struct for using delay
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clock);

    // Create a struct for controlling the usb
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clock.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Create a struct for controlling a serial connection on the usb
    let mut serial = SerialPort::new(&usb_bus);

    // Create a struct for controlling the usb device
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Michaeljoy")
        .product("template")
        .serial_number("123456")
        .device_class(2)
        .build();

    // Create a struct for the led pin
    let mut led = pins.led.into_push_pull_output();

    // Set the last second to 0
    let mut last_second = 0;

    loop {
        // Get the current time since initialization of the timer
        let now = timer.get_counter().duration_since_epoch().to_secs();

        // If that changed:
        if now != last_second {
            // Create a string
            let mut text = String::<32>::new();

            // Write a message and the current time to that string
            write!(&mut text, "Hello, world {now}!\n\r").ok();

            // Write the string to the host
            serial.write(text.as_bytes()).ok();

            // Toggle the led
            led.toggle().unwrap();

            // Store the current time as the last second
            last_second = now;
        }

        // Check for input, send the input back
        if usb_dev.poll(&mut [&mut serial]) {
            // Create a buffer
            let mut buffer = [0u8; 64];

            // Read the received data to the buffer
            match serial.read(&mut buffer) {
                Err(_) | Ok(0) => (),
                Ok(count) => {
                    // Take the received bytes
                    let mut wr_ptr = &buffer[..count];

                    // Send them, stop early if sending data fails
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(length) => wr_ptr = &wr_ptr[length..],
                            Err(_) => break,
                        }
                    }
                }
            }
        }
    }
}
