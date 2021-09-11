#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::digital::v2::InputPin;
use hal::clocks::init_clocks_and_plls;
use hal::gpio::DynPin;
use hal::pac;
use hal::pac::interrupt;
use hal::sio::Sio;
use hal::usb::UsbBus;
use hal::watchdog::Watchdog;
use keyberon::debounce::Debouncer;
use keyberon::key_code;
use keyberon::layout;
use keyberon::layout::Layout;
use keyberon::matrix::{Matrix, PressedKeys};
use panic_halt as _;
use rp2040_hal as hal;
use usb_device::class_prelude::*;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

static mut USB_DEVICE: Option<usb_device::device::UsbDevice<'_, rp2040_hal::usb::UsbBus>> = None;
static mut USB_CLASS: Option<
    keyberon::hid::HidClass<'_, rp2040_hal::usb::UsbBus, keyberon::keyboard::Keyboard<()>>,
> = None;
static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;

const SCAN_TIME_US: u32 = 1000;

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum CustomActions {
    Dfu,
    Reset,
}
// TODO: implement uf2 and reset
// const DFU: Action<CustomActions> = Custom(CustomActions::Dfu);
// const RESET: Action<CustomActions> = Custom(CustomActions::Reset);

#[rustfmt::skip]
pub static LAYERS: keyberon::layout::Layers<CustomActions> = keyberon::layout::layout! {
    {[ // 0 TODO: make real layout
        Q W E R T   Y U I O P
        A S D F G   H J K L ;
        Z X C V B   N M , . /
        1 2 t t t   3 4 t t t
    ]}
    {[ // 1
        Q W E R T   Y U I O P
        A S D F G   H J K L ;
        Z X C V B   N M , . /
        1 2 t t t   3 4 t t t
    ]}
};
#[entry]
fn main() -> ! {
    let pac = pac::Peripherals::take().unwrap();
    let mut resets = pac.RESETS;
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(
        12_000_000u32,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut resets,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut resets,
    ));
    unsafe {
        USB_BUS = Some(usb_bus);
    }

    let usb_class = keyberon::new_class(unsafe { USB_BUS.as_ref().unwrap() }, ());
    unsafe {
        USB_CLASS = Some(usb_class);
    }
    let usb_dev = keyberon::new_device(unsafe { USB_BUS.as_ref().unwrap() });
    unsafe {
        USB_DEVICE = Some(usb_dev);
    }

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut resets);

    let gpio2 = pins.gpio2;
    let gpio28 = pins.gpio28;
    let gpio3 = pins.gpio3;
    let gpio27 = pins.gpio27;
    let gpio4 = pins.gpio4;
    let gpio5 = pins.gpio5;
    let gpio26 = pins.gpio26;
    let gpio6 = pins.gpio6;
    let gpio22 = pins.gpio22;
    let gpio7 = pins.gpio7;
    let gpio10 = pins.gpio10;
    let gpio11 = pins.gpio11;
    let gpio12 = pins.gpio12;
    let gpio21 = pins.gpio21;
    let gpio13 = pins.gpio13;
    let gpio15 = pins.gpio15;
    let gpio14 = pins.gpio14;

    let gpio20 = pins.gpio20;

    // GPIO1 is high for the right hand side
    let side = pins.gpio1.into_floating_input();
    let is_right = side.is_high().unwrap();
    let transform: fn(layout::Event) -> layout::Event = if is_right {
        |e| {
            e.transform(|i: u8, j: u8| -> (u8, u8) {
                // 0 -> 5,  5 -> 15, 10 -> 25
                let x = ((j / 5) * 10) + (j % 5) + 5;
                (i, x)
            })
        }
    } else {
        |e| {
            e.transform(|i: u8, j: u8| -> (u8, u8) {
                let x = ((j / 5) * 10) + (j % 5);
                (i, x)
            })
        }
    };

    let led = pins.gpio25.into_push_pull_output();

    let mut matrix: Matrix<DynPin, DynPin, 17, 1> = cortex_m::interrupt::free(move |_cs| {
        Matrix::new(
            [
                gpio2.into_pull_up_input().into(),
                gpio28.into_pull_up_input().into(),
                gpio3.into_pull_up_input().into(),
                gpio27.into_pull_up_input().into(),
                gpio4.into_pull_up_input().into(),
                gpio5.into_pull_up_input().into(),
                gpio26.into_pull_up_input().into(),
                gpio6.into_pull_up_input().into(),
                gpio22.into_pull_up_input().into(),
                gpio7.into_pull_up_input().into(),
                gpio10.into_pull_up_input().into(),
                gpio11.into_pull_up_input().into(),
                gpio12.into_pull_up_input().into(),
                gpio21.into_pull_up_input().into(),
                gpio13.into_pull_up_input().into(),
                gpio15.into_pull_up_input().into(),
                gpio14.into_pull_up_input().into(),
            ],
            [gpio20.into_push_pull_output().into()],
        )
    })
    .unwrap();

    let mut layout = Layout::new(LAYERS);
    let mut debouncer = Debouncer::new(PressedKeys::default(), PressedKeys::default(), 30);
    let mut last_time = pac.TIMER.timelr.read().bits();

    // Enable USB interrupt
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    loop {
        let current_time = pac.TIMER.timelr.read().bits();
        // TODO: fix overflow
        if (current_time - last_time) > SCAN_TIME_US {
            last_time = current_time;
            for event in debouncer.events(matrix.get().unwrap()).map(transform) {
                layout.event(event);
            }
            match layout.tick() {
                keyberon::layout::CustomEvent::Press(event) => match event {
                    CustomActions::Reset => {
                        cortex_m::peripheral::SCB::sys_reset();
                    }
                    _ => (),
                },
                _ => (),
            }
            let report: key_code::KbHidReport = layout.keycodes().collect();
            // TODO: use rtic
            // cortex_m::interrupt::free(|_| {
            let usb_c = unsafe { USB_CLASS.as_mut().unwrap() };
            if usb_c.device_mut().set_keyboard_report(report.clone()) {
                while let Ok(0) = usb_c.write(report.as_bytes()) {}
            }
            // });
        }
    }
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_class = USB_CLASS.as_mut().unwrap();
    if usb_dev.poll(&mut [usb_class]) {
        usb_class.poll();
    }
}
