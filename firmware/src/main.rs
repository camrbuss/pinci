#![no_std]
#![no_main]

use cortex_m::prelude::_embedded_hal_serial_Read;
use cortex_m::prelude::_embedded_hal_serial_Write;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use hal::clocks::init_clocks_and_plls;
use hal::gpio::DynPin;
use hal::pac;
use hal::pac::interrupt;
use hal::sio::Sio;
use hal::uart::UartPeripheral;
use hal::usb::UsbBus;
use hal::watchdog::Watchdog;
use keyberon::action::{k, l, Action, HoldTapConfig};
use keyberon::debounce::Debouncer;
use keyberon::key_code;
use keyberon::key_code::KeyCode::*;
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

const A_LSHIFT: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &k(LShift),
    tap: &k(A),
    config: HoldTapConfig::PermissiveHold,
    tap_hold_interval: 0,
};
const L5_S: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &l(5),
    tap: &k(S),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
};
const D_LALT: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &k(LAlt),
    tap: &k(D),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
};
const L2_F: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &l(2),
    tap: &k(F),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
};
const DOT_RALT: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &k(RAlt),
    tap: &k(Dot),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
};
const X_LALT: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &k(LAlt),
    tap: &k(X),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
};
const SLASH_RCTRL: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &k(RCtrl),
    tap: &k(Slash),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
};
const Z_LCTRL: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &k(LCtrl),
    tap: &k(Z),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
};
const L4_C: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &l(4),
    tap: &k(C),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
};
const SEMI_RSHIFT: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &k(RShift),
    tap: &k(SColon),
    config: HoldTapConfig::PermissiveHold,
    tap_hold_interval: 0,
};
const L7_SPACE: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &l(7),
    tap: &k(Space),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
};
const L4_COMMA: Action<CustomActions> = Action::HoldTap {
    timeout: 200,
    hold: &l(4),
    tap: &k(Comma),
    config: HoldTapConfig::Default,
    tap_hold_interval: 0,
};

#[rustfmt::skip]
pub static LAYERS: keyberon::layout::Layers<CustomActions> = keyberon::layout::layout! {
{[ // 0
        Q W E R T Y U I O P
        {A_LSHIFT} {L5_S} {D_LALT} {L2_F} G H J K L {SEMI_RSHIFT}
        {Z_LCTRL} {X_LALT} {L4_C} V B N M {L4_COMMA} {DOT_RALT} {SLASH_RCTRL}
        t t t (3) BSpace  {L7_SPACE} t Escape Enter Tab 
    ]}
    {[ // 1
        t t t t t t t t t t
        t t t t t t t t t t
        t t t t t t t t t t
        t t t t t t t t t t
    ]}
    {[ // 2
        t t t t t * 7 8 9 +
        t t t t t / 4 5 6 -
        t t t t t . 1 2 3 .
        t t t t t 0 0 t t t
    ]}
    {[ // 3
        t 7 8 9 t t t t t t
        t 4 5 6 t t t t t t
        0 1 2 3 t t t t t t
        t t t t t t t t t t
    ]}
    {[ // 4
        ! @ # $ % t '_' | = +
        '{' '}' '(' ')' t '`' ~ / '"' Quote
        '[' ']' ^ & * t - '\\' t t
        t t t t t t t t t t
    ]}
    {[ // 5
        t t t t t t t PgUp t t
        t t Delete t t Left Down Up Right Enter
        t t t t t t Home PgDown End t
        t t t t t t t t t t
    ]}
    {[ // 6
        t t t t t t t t t MediaSleep
        t t t t t t t t t t
        t t t t t t t t t t
        t t t t t t t t t t
    ]}
    {[ // 7
        t t t t t t t t t PScreen
        Tab Escape t t t MediaNextSong MediaPlayPause MediaVolDown MediaVolUp Enter
        t t t t t t t t t Delete
        t t t Delete t t t t t t
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

    let mut led = pins.gpio25.into_push_pull_output();
    // GPIO1 is high for the right hand side
    let side = pins.gpio1.into_floating_input();
    // delay for power on
    for _ in 0..1000 {
        cortex_m::asm::nop();
    }
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
                let x = ((j / 5) * 10) + 4 - (j % 5);
                (i, x)
            })
        }
    };
    if is_right {
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
    }

    let mut uart = UartPeripheral::<_, _>::enable(
        pac.UART0,
        &mut resets,
        hal::uart::common_configs::_115200_8_N_1,
        clocks.peripheral_clock.into(),
    )
    .unwrap();

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

    if is_right {
        let _rx_pin = pins.gpio17.into_mode::<hal::gpio::FunctionUart>();
        led.set_high().unwrap();
        // Enable USB interrupt
        unsafe {
            pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
        };
    } else {
        let _tx_pin = pins.gpio16.into_mode::<hal::gpio::FunctionUart>();
    }

    loop {
        if is_right {
            let current_time = pac.TIMER.timelr.read().bits();
            // TODO: make read interrupt driven
            if let Ok(r) = uart.read() {
                if (r & 0b01000000) > 0 {
                    layout.event(layout::Event::Press(0, r & 0b00111111));
                } else {
                    layout.event(layout::Event::Release(0, r & 0b00111111));
                }
            }
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
        } else {
            let current_time = pac.TIMER.timelr.read().bits();
            if (current_time - last_time) > SCAN_TIME_US {
                last_time = current_time;
                // coordinate cannot go past 63
                // end? press=1/release=0 key_number
                //   7         6            543210
                let mut events = debouncer
                    .events(matrix.get().unwrap())
                    .map(transform)
                    .peekable();
                while let Some(event) = events.next() {
                    let mut byte: u8;
                    if event.coord().1 <= 0b00111111 {
                        byte = event.coord().1;
                    } else {
                        byte = 0b00111111;
                    }
                    byte |= (event.is_press() as u8) << 6;
                    if events.peek().is_none() {
                        byte |= 0b10000000;
                    }
                    uart.write(byte).unwrap();
                }
            }
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
