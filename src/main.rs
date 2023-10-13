#![no_std]
#![no_main]

extern crate alloc;
extern crate bme280;

use alloc::format;
use arduino_nano33iot as bsp;
use arduino_nano33iot::ehal::digital::v2::PinState;
use arduino_nano33iot::pac::interrupt;
use bme280::BME280;
use bsp::hal;
use embedded_alloc::Heap;

use usb_device;
use usbd_serial;

use panic_halt as _;

use bsp::entry;
use hal::clock::GenericClockController;
use hal::delay::Delay;
use hal::pac::{CorePeripherals, Peripherals};
use hal::prelude::*;
use hal::time::KiloHertz;
use hal::usb::UsbBus;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use cortex_m::asm::delay as cycle_delay;
use cortex_m::peripheral::NVIC;

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );
    let pins = bsp::Pins::new(peripherals.PORT);

    let mut led: bsp::Led = pins.led_sck.into();

    let mut delay = Delay::new(core.SYST, &mut clocks);

    let i2c = bsp::i2c_master(
        &mut clocks,
        KiloHertz(400),
        peripherals.SERCOM4,
        &mut peripherals.PM,
        pins.sda,
        pins.scl,
    );

    let mut bme280 = BME280::new_primary(i2c, delay);
    bme280.init().unwrap();

    let bus_allocator = unsafe {
        USB_ALLOCATOR = Some(bsp::usb_allocator(
            peripherals.USB,
            &mut clocks,
            &mut peripherals.PM,
            pins.usb_dm,
            pins.usb_dp,
        ));
        USB_ALLOCATOR.as_ref().unwrap()
    };

    unsafe {
        USB_SERIAL = Some(SerialPort::new(&bus_allocator));
        USB_BUS = Some(
            UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x2222, 0x3333))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")
                .device_class(USB_CLASS_CDC)
                .build(),
        );
    }

    unsafe {
        core.NVIC.set_priority(interrupt::USB, 1);
        NVIC::unmask(interrupt::USB);
    }

    let mut pin_state = PinState::Low;

    loop {
        cycle_delay(5 * 1024 * 1024);
        if pin_state == PinState::Low {
            pin_state = PinState::High;
        } else {
            pin_state = PinState::Low;
        }
        led.set_state(pin_state).unwrap();
        let measurements = bme280.measure().unwrap();

        // Turn off interrupts so we don't fight with the interrupt
        cortex_m::interrupt::free(|_| unsafe {
            USB_BUS.as_mut().map(|_| {
                USB_SERIAL.as_mut().map(|serial| {
                    // Skip errors so we can continue the program
                    let _ = serial.write(
                        format!("Relative Humidity = {}%\n\r", measurements.humidity).as_bytes(),
                    );
                    let _ = serial.write(
                        format!("Temperature = {} deg C\n\r", measurements.temperature).as_bytes(),
                    );
                    let _ = serial.write(
                        format!("Pressure = {} pascals\n\r", measurements.pressure).as_bytes(),
                    );
                });
            })
        });
    }
}

static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;

fn poll_usb() {
    unsafe {
        USB_BUS.as_mut().map(|usb_dev| {
            USB_SERIAL.as_mut().map(|serial| {
                usb_dev.poll(&mut [serial]);

                // Make the other side happy
                let mut buf = [0u8; 16];
                let _ = serial.read(&mut buf);
            });
        });
    };
}

#[interrupt]
fn USB() {
    poll_usb();
}
