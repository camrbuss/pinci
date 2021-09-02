#![no_main]
#![no_std]

use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use hal::clocks::init_clocks_and_plls;
use hal::gpio::DynPin;
use hal::pac;
use hal::sio::Sio;
use hal::watchdog::Watchdog;
use keyberon::chording::{ChordDef, Chording};
use keyberon::debounce::Debouncer;
use keyberon::layout::Layout;
use keyberon::matrix::{Matrix, PressedKeys};
use pac::interrupt;
use panic_halt as _;
use pio::SetDestination;
use rp2040_hal as hal;

const SCAN_TIME_US: u32 = 1000;

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub enum CustomActions {
    Dfu,
    Reset,
}
// TODO: implement uf2 and reset
// const DFU: Action<CustomActions> = Custom(CustomActions::Dfu);
// const RESET: Action<CustomActions> = Custom(CustomActions::Reset);
const HK_TAB: ChordDef = ChordDef::new((0, 0), &[(0, 0), (0, 1)]);

#[rustfmt::skip]
pub static LAYERS: keyberon::layout::Layers<CustomActions> = keyberon::layout::layout! {
    {[ // 0 TODO: make real layout
        Q W E R T Y U I O P
        A S D F G H J K L ;
        Z X C V B N M , . /
        1 2 3 4 
    ]}
    {[ // 1
        1 2 3 4 5 6 7 8 9 0
        A S D F G H J K L ;
        Z X C V B N M , . /
        1 2 3 4 
    ]}
};

static ALARM: AtomicBool = AtomicBool::new(false);

#[entry]
fn main() -> ! {
    let pac = pac::Peripherals::take().unwrap();

    let mut resets = pac.RESETS;

    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let _clocks = init_clocks_and_plls(
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

    resets.reset.modify(|_r, w| w.usbctrl().clear_bit());

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
    let mut ledo = pins.gpio18.into_push_pull_output();

    // TODO: make pin 8 and 9 a ps/2 interface
    let _: hal::gpio::Pin<_, hal::gpio::FunctionPio0> = pins.gpio8.into_mode();
    let _: hal::gpio::Pin<_, hal::gpio::FunctionPio0> = pins.gpio9.into_mode();

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
    let mut chording = Chording::new(&[HK_TAB]);

    // sideset is mandatory for each instruction, 1 pin is needed, pindirs is set to data
    let side_set = pio::SideSet::new(false, 2, false);
    let mut a = pio::Assembler::new_with_side_set(side_set);
    let mut bit_loop = a.label();
    a.set_with_side_set(SetDestination::PINS, 1, 0);
    a.pull_with_side_set(false, true, 1);
    a.set_with_side_set(SetDestination::PINS, 0, 0);
    a.set_with_side_set(SetDestination::X, 8, 1);
    a.bind(&mut bit_loop);
    a.out_with_side_set(pio::OutDestination::PINS, 1, 0);
    a.jmp_with_side_set(pio::JmpCondition::XDecNonZero, &mut bit_loop, 1);

    let program = a.assemble(None);

    let pio = hal::pio::PIO::new(pac.PIO0, &mut resets);
    let sm = &pio.state_machines()[0];

    hal::pio::PIOBuilder::default()
        .with_program(&program)
        .buffers(hal::pio::Buffers::OnlyTx)
        .out_pins(8, 1)
        .set_pins(8, 1)
        .side_set(side_set)
        .side_set_pin_base(9)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .in_shift_direction(hal::pio::ShiftDirection::Right)
        .autopull(false)
        .clock_divisor(5000.0) // TODO: document magic number
        .build(&pio, sm)
        .unwrap();

    sm.set_enabled(true);

    let mut last_time = pac.TIMER.timelr.read().bits();
    loop {
        let current_time = pac.TIMER.timelr.read().bits();
        if (current_time - last_time) > SCAN_TIME_US {
            last_time = current_time;
            let events = debouncer.events(matrix.get().unwrap());
            for event in chording.tick(events.collect()).into_iter() {
                layout.event(event);
            }
            match layout.tick() {
                // TODO: implement right and left transform
                keyberon::layout::CustomEvent::Press(event) => match event {
                    CustomActions::Reset => {
                        cortex_m::peripheral::SCB::sys_reset();
                    }
                    _ => (),
                },
                _ => (),
            }
            for key in layout.keycodes() {
                if key == keyberon::key_code::KeyCode::J {
                    // TODO: parse keycodes correctly
                    sm.push(0b110110001);
                    led.set_high().unwrap();
                    ledo.set_low().unwrap();
                } else {
                    // TODO: remove led debug
                    led.set_low().unwrap();
                    ledo.set_high().unwrap();
                }
            }
        }
    }
}

#[allow(non_snake_case)]
#[interrupt]
fn TIMER_IRQ_0() {
    ALARM.store(true, Ordering::Relaxed);
    // TODO: implement timer to prevent overflow issue
}
