#![no_std]
#![no_main]

use panic_halt as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {
    use core::sync::atomic::{AtomicUsize, Ordering};
    use cortex_m::prelude::_embedded_hal_watchdog_Watchdog;
    use cortex_m::prelude::_embedded_hal_watchdog_WatchdogEnable;
    use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
    use fugit::{MicrosDurationU32, RateExtU32};
    use keyberon::action::{k, l, Action, HoldTapAction, HoldTapConfig};
    use keyberon::chording::{ChordDef, Chording};
    use keyberon::debounce::Debouncer;
    use keyberon::key_code::{self, KeyCode::*};
    use keyberon::layout::{self, Layout};
    use keyberon::matrix::Matrix;
    use rp_pico::{
        hal::{
            self,
            clocks::init_clocks_and_plls,
            gpio::{pin::bank0, DynPin, Function, Pin, Uart},
            pac::UART0,
            sio::Sio,
            timer::Alarm,
            uart::{DataBits, Reader, StopBits, UartConfig, UartPeripheral, Writer},
            usb::UsbBus,
            watchdog::Watchdog,
            Clock,
        },
        XOSC_CRYSTAL_FREQ,
    };
    use rtt_target::{rprintln, rtt_init_print};
    use usb_device::class_prelude::*;
    use usb_device::device::UsbDeviceState;

    const SCAN_INTERVAL: MicrosDurationU32 = MicrosDurationU32::millis(1000);
    const WATCHDOG_INTERVAL: MicrosDurationU32 = MicrosDurationU32::millis(100000);

    const COMS_NUM_BYTES: usize = 3;

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
        #[lock_free]
        matrix: Matrix<DynPin, DynPin, 17, 1>,
        layout: Layout<40, 1, 8, CustomActions>,
        is_right: bool,
        watchdog: hal::watchdog::Watchdog,
    }

    #[local]
    struct Local {
        alarm: hal::timer::Alarm0,
        chording: Chording<4>,
        debouncer: Debouncer<[[bool; 17]; 1]>,
        led: Pin<bank0::Gpio25, hal::gpio::PushPullOutput>,
        transform: fn(layout::Event) -> layout::Event,
        uart_r: Reader<
            UART0,
            (
                Pin<bank0::Gpio16, Function<Uart>>,
                Pin<bank0::Gpio17, Function<Uart>>,
            ),
        >,
        uart_w: Writer<
            UART0,
            (
                Pin<bank0::Gpio16, Function<Uart>>,
                Pin<bank0::Gpio17, Function<Uart>>,
            ),
        >,
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

        // TRS cable only supports one direction of communication
        if is_right {
            // led.set_high().unwrap();
        } else {
            led.set_low().unwrap();
        }
        let _ = alarm.schedule(SCAN_INTERVAL);
        alarm.enable_interrupt();
        let tx_pin = pins.gpio16.into_mode::<hal::gpio::FunctionUart>();
        let rx_pin = pins.gpio17.into_mode::<hal::gpio::FunctionUart>();
        let uart = UartPeripheral::new(c.device.UART0, (tx_pin, rx_pin), &mut resets)
            .enable(
                UartConfig::new(460800.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();
        let (uart_r, uart_w) = uart.split();

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

        // Start watchdog and feed it in timer for leftside and idle loop for right
        // watchdog.start(WATCHDOG_INTERVAL);

        rtt_init_print!();
        rprintln!("Finishing Setup");

        (
            Shared {
                usb_dev,
                usb_class,
                matrix,
                layout,
                is_right,
                watchdog,
            },
            Local {
                alarm,
                chording,
                debouncer,
                led,
                transform,
                uart_r,
                uart_w,
            },
            init::Monotonics(),
        )
    }

    // #[idle(
    //     shared = [&is_right, watchdog],
    //     local = [uart_r, led, debouncer, chording])]
    // fn idle(mut c: idle::Context) -> ! {
    //     let uart = c.local.uart_r;
    //     '_outter: loop {
    //         if *c.shared.is_right {
    //             c.shared.watchdog.lock(|w| w.feed());
    //             let mut buffer: [u8; 5] = [77; 5];
    //             // let mut byte: [u8; 1] = [0];
    //             let mut offset = 0;
    //             let mut found_last_byte = false;

    //             while offset != buffer.len() || found_last_byte {
    //                 offset += match uart.read_raw(&mut buffer[offset..]) {
    //                     Ok(bytes_read) => bytes_read,
    //                     Err(e) => continue,
    //                 };
    //                 if let Some(last_byte_pos) =
    //                     buffer.into_iter().position(|x| (x & 0b1000_0000) > 0)
    //                 {
    //                     found_last_byte = true;
    //                     // rprintln!("{:?} offset: {:?}", buffer, offset);
    //                     c.local.led.toggle().unwrap();
    //                     if last_byte_pos >= 2 {
    //                         let rec_buf: &[u8] = &buffer[last_byte_pos - 2..last_byte_pos + 1];
    //                         // The buffer has a full packet
    //                         rprintln!("rec_buf: {:?}", rec_buf);
    //                         break;
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }

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

    #[task(
        capacity = 1,
        priority = 1,
        shared = [&is_right, watchdog, matrix],
        local = [led, uart_r, debouncer],
    )]
    fn poll_uart(mut c: poll_uart::Context) {
        let uart = c.local.uart_r;
        '_outter: loop {
            if *c.shared.is_right {
                c.shared.watchdog.lock(|w| w.feed());
                let mut buffer: [u8; 5] = [77; 5];
                // let mut byte: [u8; 1] = [0];
                let mut offset = 0;
                let mut found_last_byte = false;

                while offset != buffer.len() || found_last_byte {
                    offset += match uart.read_raw(&mut buffer[offset..]) {
                        Ok(bytes_read) => bytes_read,
                        Err(_e) => continue,
                    };
                    if let Some(last_byte_pos) =
                        buffer.into_iter().position(|x| (x & 0b1000_0000) > 0)
                    {
                        found_last_byte = true;
                        // rprintln!("{:?} offset: {:?}", buffer, offset);
                        c.local.led.toggle().unwrap();
                        if last_byte_pos >= 2 {
                            let rec_buf: &[u8] = &buffer[last_byte_pos - 2..last_byte_pos + 1];
                            // The buffer has a full packet
                            rprintln!("rec_buf: {:?}", rec_buf);
            // let keys_pressed = c.shared.matrix.get().unwrap()[0];
                            break;
                        }
                    }
                }
            }
        }
    }

    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        shared = [&is_right, watchdog, matrix],
        local = [alarm, uart_w],
    )]
    fn scan_timer_irq(mut c: scan_timer_irq::Context) {
        let alarm = c.local.alarm;
        alarm.clear_interrupt();
        let _ = alarm.schedule(SCAN_INTERVAL);

        c.shared.watchdog.lock(|w| w.feed());

        if *c.shared.is_right {
            alarm.disable_interrupt();
            // this should spawn and then loop forever
            if poll_uart::spawn().is_err() {
                alarm.disable_interrupt();
            }
        } else {
            let keys_pressed = c.shared.matrix.get().unwrap()[0];
            // Each key on the left side is represented by one bit
            // the MSB of each byte is always 0, except for the last byte
            //                          x            x            1
            let mut key_bytes: [u8; COMS_NUM_BYTES] = [0b0000_0000, 0b0000_0000, 0b1000_0000];
            for x in 0..keys_pressed.len() {
                key_bytes[x / 7] |= ((keys_pressed[x] == true) as u8) << (x % 7);
            }

            c.local.uart_w.write_full_blocking(&key_bytes);
        }
    }
}
