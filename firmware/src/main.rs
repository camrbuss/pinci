#![no_std]
#![no_main]

use panic_halt as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {
    use cortex_m::prelude::_embedded_hal_watchdog_Watchdog;
    use cortex_m::prelude::_embedded_hal_watchdog_WatchdogEnable;
    use embedded_hal::digital::v2::{InputPin, OutputPin};
    use embedded_time::duration::units::*;
    use keyberon::action::{k, l, Action, HoldTapAction, HoldTapConfig};
    use keyberon::chording::{ChordDef, Chording};
    use keyberon::debounce::Debouncer;
    use keyberon::key_code::{self, KeyCode::*};
    use keyberon::layout::{self, Layout};
    use keyberon::matrix::Matrix;
    use rp_pico::{
        hal::{
            self, clocks::init_clocks_and_plls, gpio::DynPin, sio::Sio, timer::Alarm, usb::UsbBus,
            watchdog::Watchdog,
        },
        XOSC_CRYSTAL_FREQ,
    };
    use usb_device::class_prelude::*;
    use usb_device::device::UsbDeviceState;

    const SCAN_TIME_US: u32 = 1000;
    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<rp2040_hal::usb::UsbBus>> = None;

    #[derive(Debug, Clone, Copy, Eq, PartialEq)]
    pub enum CustomActions {
        Uf2,
        Reset,
    }
    const UF2: Action<CustomActions> = Action::Custom(CustomActions::Uf2);
    const RESET: Action<CustomActions> = Action::Custom(CustomActions::Reset);

    const QW_ESC: ChordDef = ((0, 37), &[(0, 0), (0, 1)]);
    const JU_ESC: ChordDef = ((0, 37), &[(0, 16), (0, 6)]);
    const KI_TAB: ChordDef = ((0, 39), &[(0, 17), (0, 7)]);
    const LO_ENTER: ChordDef = ((0, 38), &[(0, 18), (0, 8)]);
    const CHORDS: [ChordDef; 4] = [QW_ESC, JU_ESC, KI_TAB, LO_ENTER];

    const A_LSHIFT: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
        timeout: 200,
        hold: k(LShift),
        tap: k(A),
        config: HoldTapConfig::PermissiveHold,
        tap_hold_interval: 0,
    });
    const L5_S: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
        timeout: 200,
        hold: l(5),
        tap: k(S),
        config: HoldTapConfig::Default,
        tap_hold_interval: 0,
    });
    const D_LALT: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
        timeout: 200,
        hold: k(LAlt),
        tap: k(D),
        config: HoldTapConfig::Default,
        tap_hold_interval: 0,
    });
    const L2_F: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
        timeout: 200,
        hold: l(2),
        tap: k(F),
        config: HoldTapConfig::Default,
        tap_hold_interval: 0,
    });
    const DOT_RALT: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
        timeout: 200,
        hold: k(RAlt),
        tap: k(Dot),
        config: HoldTapConfig::Default,
        tap_hold_interval: 0,
    });
    const X_LALT: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
        timeout: 200,
        hold: k(LAlt),
        tap: k(X),
        config: HoldTapConfig::Default,
        tap_hold_interval: 0,
    });
    const SLASH_RCTRL: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
        timeout: 200,
        hold: k(RCtrl),
        tap: k(Slash),
        config: HoldTapConfig::Default,
        tap_hold_interval: 0,
    });
    const Z_LCTRL: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
        timeout: 200,
        hold: k(LCtrl),
        tap: k(Z),
        config: HoldTapConfig::Default,
        tap_hold_interval: 0,
    });
    const L4_C: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
        timeout: 200,
        hold: l(4),
        tap: k(C),
        config: HoldTapConfig::Default,
        tap_hold_interval: 0,
    });
    const SEMI_RSHIFT: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
        timeout: 200,
        hold: k(RShift),
        tap: k(SColon),
        config: HoldTapConfig::PermissiveHold,
        tap_hold_interval: 0,
    });
    const L7_SPACE: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
        timeout: 200,
        hold: l(7),
        tap: k(Space),
        config: HoldTapConfig::Default,
        tap_hold_interval: 0,
    });
    const L4_COMMA: Action<CustomActions> = Action::HoldTap(&HoldTapAction {
        timeout: 200,
        hold: l(4),
        tap: k(Comma),
        config: HoldTapConfig::Default,
        tap_hold_interval: 0,
    });

    #[rustfmt::skip]
    pub static LAYERS: keyberon::layout::Layers<40, 1, 8, CustomActions> = keyberon::layout::layout! {
    {[ // 0
        Q          W        E        R      T      Y          U   I          O          P
        {A_LSHIFT} {L5_S}   {D_LALT} {L2_F} G      H          J   K          L          {SEMI_RSHIFT}
        {Z_LCTRL}  {X_LALT} {L4_C}   V      B      N          M   {L4_COMMA} {DOT_RALT} {SLASH_RCTRL}
        t          t        t        (3)    BSpace {L7_SPACE} Tab Escape     Enter      Tab
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
        !   @   #   $   % t ~   |    '`' +
        '{' '}' '(' ')' t = '_' -    '"' Quote
        '[' ']' ^   &   * t /   '\\' t   t
        t   t   t   t   t t t   t    t   t
    ]}
    {[ // 5
        t t t      t t t    t    PgUp   t     t
        t t Delete t t Left Down Up     Right Enter
        t t t      t t t    Home PgDown End   t
        t t t      t t t    t    t      t     t
    ]}
    {[ // 6
        {RESET} {UF2} t t t t t t t MediaSleep
        t       t     t t t t t t t t
        t       t     t t t t t t t t
        t       t     t t t t t t t t
    ]}
    {[ // 7
        t t t t t      MediaNextSong MediaPlayPause MediaVolDown MediaVolUp PScreen
        t t t t t      t             Escape         Tab          Enter      Enter
        t t t t t      t             t              t            t          Delete
        t t t t Delete t             t              t            t          t
    ]}

};

    #[shared]
    struct Shared {
        usb_dev: usb_device::device::UsbDevice<'static, rp2040_hal::usb::UsbBus>,
        usb_class: keyberon::hid::HidClass<
            'static,
            rp2040_hal::usb::UsbBus,
            keyberon::keyboard::Keyboard<()>,
        >,
        uart: rp2040_hal::pac::UART0,
        layout: Layout<40, 1, 8, CustomActions>,
    }

    #[local]
    struct Local {
        watchdog: hal::watchdog::Watchdog,
        chording: Chording<4>,
        matrix: Matrix<DynPin, DynPin, 17, 1>,
        debouncer: Debouncer<[[bool; 17]; 1]>,
        alarm: hal::timer::Alarm0,
        transform: fn(layout::Event) -> layout::Event,
        is_right: bool,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(c.device.SIO);
        let pins = hal::gpio::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        // 17 input pins and 1 empty pin that is not really used, but
        // is needed by keyberon as a "row"
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

        // Use a transform to get correct layout from right and left side
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

        // Enable UART0
        resets.reset.modify(|_, w| w.uart0().clear_bit());
        while resets.reset_done.read().uart0().bit_is_clear() {}
        let uart = c.device.UART0;
        uart.uartibrd.write(|w| unsafe { w.bits(0b0100_0011) });
        uart.uartfbrd.write(|w| unsafe { w.bits(0b0011_0100) });
        uart.uartlcr_h.write(|w| unsafe { w.bits(0b0110_0000) });
        uart.uartcr.write(|w| unsafe { w.bits(0b11_0000_0001) });
        uart.uartimsc.write(|w| w.rxim().set_bit());

        let matrix: Matrix<DynPin, DynPin, 17, 1> = Matrix::new(
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
        .unwrap();

        let layout = Layout::new(&LAYERS);
        let debouncer = Debouncer::new([[false; 17]; 1], [[false; 17]; 1], 20);

        let chording = Chording::new(&CHORDS);

        let mut timer = hal::Timer::new(c.device.TIMER, &mut resets);
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(SCAN_TIME_US.microseconds());
        alarm.enable_interrupt();

        // TRS cable only supports one direction of communication
        if is_right {
            let _rx_pin = pins.gpio17.into_mode::<hal::gpio::FunctionUart>();
            led.set_high().unwrap();
        } else {
            let _tx_pin = pins.gpio16.into_mode::<hal::gpio::FunctionUart>();
        }

        let usb_bus = UsbBusAllocator::new(UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        ));
        unsafe {
            USB_BUS = Some(usb_bus);
        }
        let usb_class = keyberon::new_class(unsafe { USB_BUS.as_ref().unwrap() }, ());
        let usb_dev = keyberon::new_device(unsafe { USB_BUS.as_ref().unwrap() });

        // Start watchdog and feed it with the lowest priority task at 1000hz
        watchdog.start(10_000.microseconds());

        (
            Shared {
                usb_dev,
                usb_class,
                uart,
                layout,
            },
            Local {
                alarm,
                chording,
                watchdog,
                matrix,
                debouncer,
                transform,
                is_right,
            },
            init::Monotonics(),
        )
    }

    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [usb_dev, usb_class])]
    fn usb_rx(c: usb_rx::Context) {
        let mut usb_d = c.shared.usb_dev;
        let mut usb_c = c.shared.usb_class;
        usb_d.lock(|d| {
            usb_c.lock(|c| {
                if d.poll(&mut [c]) {
                    c.poll();
                }
            })
        });
    }

    #[task(priority = 2, capacity = 8, shared = [usb_dev, usb_class, layout])]
    fn handle_event(mut c: handle_event::Context, event: Option<layout::Event>) {
        match event {
            // TODO: Support Uf2 for the side not performing USB HID
            // The right side only passes None here and buffers the keys
            // for USB to send out when polled by the host
            None => match c.shared.layout.lock(|l| l.tick()) {
                layout::CustomEvent::Press(event) => match event {
                    CustomActions::Uf2 => {
                        hal::rom_data::reset_to_usb_boot(0, 0);
                    }
                    CustomActions::Reset => {
                        cortex_m::peripheral::SCB::sys_reset();
                    }
                },
                _ => (),
            },
            Some(e) => {
                c.shared.layout.lock(|l| l.event(e));
                return;
            }
        };
        let report: key_code::KbHidReport = c.shared.layout.lock(|l| l.keycodes().collect());
        if !c
            .shared
            .usb_class
            .lock(|k| k.device_mut().set_keyboard_report(report.clone()))
        {
            return;
        }
        if c.shared.usb_dev.lock(|d| d.state()) != UsbDeviceState::Configured {
            return;
        }
        while let Ok(0) = c.shared.usb_class.lock(|k| k.write(report.as_bytes())) {}
    }

    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        shared = [uart],
        local = [matrix, debouncer, chording, watchdog, alarm, transform, is_right],
    )]
    fn scan_timer_irq(mut c: scan_timer_irq::Context) {
        let alarm = c.local.alarm;
        alarm.clear_interrupt();
        let _ = alarm.schedule(SCAN_TIME_US.microseconds());

        c.local.watchdog.feed();
        let keys_pressed = c.local.matrix.get().unwrap();
        let deb_events = c
            .local
            .debouncer
            .events(keys_pressed)
            .map(c.local.transform);
        // TODO: right now chords cannot only be exclusively on one side
        let events = c.local.chording.tick(deb_events.collect()).into_iter();

        // TODO: With a TRS cable, we can only have one device support USB
        if *c.local.is_right {
            for event in events {
                handle_event::spawn(Some(event)).unwrap();
            }
            handle_event::spawn(None).unwrap();
        } else {
            // coordinate and press/release is encoded in a single byte
            // the first 6 bits are the coordinate and therefore cannot go past 63
            // The last bit is to signify if it is the last byte to be sent, but
            // this is not currently used as serial rx is the highest priority
            // end? press=1/release=0 key_number
            //   7         6            543210
            let mut es: [Option<layout::Event>; 16] = [None; 16];
            for (i, e) in events.enumerate() {
                es[i] = Some(e);
            }
            let stop_index = es.iter().position(|&v| v == None).unwrap();
            for i in 0..(stop_index + 1) {
                let mut byte: u8;
                if let Some(ev) = es[i] {
                    if ev.coord().1 <= 0b0011_1111 {
                        byte = ev.coord().1;
                    } else {
                        byte = 0b0011_1111;
                    }
                    byte |= (ev.is_press() as u8) << 6;
                    if i == stop_index + 1 {
                        byte |= 0b1000_0000;
                    }
                    // Watchdog will catch any possibility for an infinite loop
                    while c.shared.uart.lock(|u| u.uartfr.read().txff().bit_is_set()) {}
                    c.shared
                        .uart
                        .lock(|u| u.uartdr.write(|w| unsafe { w.data().bits(byte) }));
                }
            }
        }
    }

    #[task(binds = UART0_IRQ, priority = 4, shared = [uart])]
    fn rx(mut c: rx::Context) {
        // RX FIFO is disabled so we just check that the byte received is valid
        // and then we read it. If a bad byte is received, it is possible that the
        // receiving side will never read. TODO: fix this
        if c.shared.uart.lock(|u| {
            u.uartmis.read().rxmis().bit_is_set()
                && u.uartfr.read().rxfe().bit_is_clear()
                && u.uartdr.read().oe().bit_is_clear()
                && u.uartdr.read().be().bit_is_clear()
                && u.uartdr.read().pe().bit_is_clear()
                && u.uartdr.read().fe().bit_is_clear()
        }) {
            let d: u8 = c.shared.uart.lock(|u| u.uartdr.read().data().bits());
            if (d & 0b01000000) > 0 {
                handle_event::spawn(Some(layout::Event::Press(0, d & 0b0011_1111))).unwrap();
            } else {
                handle_event::spawn(Some(layout::Event::Release(0, d & 0b0011_1111))).unwrap();
            }
        }
    }
}
