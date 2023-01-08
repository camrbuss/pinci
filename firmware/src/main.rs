#![no_std]
#![no_main]

use panic_halt as _;
mod layout;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0, PIO1_IRQ_0])]
mod app {
    use crate::layout::CustomActions;
    use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
    use fugit::{MicrosDurationU32, RateExtU32};
    use keyberon::chording::Chording;
    use keyberon::debounce::Debouncer;
    use keyberon::key_code::KbHidReport;
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

    // const SCAN_INTERVAL: MicrosDurationU32 = MicrosDurationU32::millis(1000);
    const SCAN_INTERVAL: MicrosDurationU32 = MicrosDurationU32::millis(1);

    const COMS_NUM_BYTES: usize = 3;

    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<rp2040_hal::usb::UsbBus>> = None;

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
        is_right: bool,
    }

    #[local]
    struct Local {
        alarm: hal::timer::Alarm0,
        chording: Chording<6>,
        debouncer: Debouncer<[[bool; 40]; 1]>,
        layout: Layout<40, 1, 5, CustomActions>,
        led: Pin<bank0::Gpio25, hal::gpio::PushPullOutput>,
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

        let is_right = side.is_high().unwrap();

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

        let layout = Layout::new(&crate::layout::LAYERS);
        let debouncer = Debouncer::new([[false; 40]; 1], [[false; 40]; 1], 20);

        let chording = Chording::new(&crate::layout::CHORDS);

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

        rtt_init_print!();
        rprintln!("Finishing Setup");

        (
            Shared {
                usb_dev,
                usb_class,
                matrix,
                is_right,
            },
            Local {
                alarm,
                chording,
                debouncer,
                layout,
                led,
                uart_r,
                uart_w,
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

    #[task(priority = 2, capacity = 1, shared = [usb_class], local = [chording, debouncer, layout])]
    fn handle_event(c: handle_event::Context, keys_pressed: [[bool; 40]; 1]) {
        // for event in c
        //     .local
        //     .chording
        //     .tick(c.local.debouncer.events(keys_pressed).collect())
        for event in c.local.debouncer.events(keys_pressed) {
            // rprintln!("event: {:?}", event);
            rprintln!("e{}", event.coord().1);
            c.local.layout.event(event);
        }
        match c.local.layout.tick() {
            layout::CustomEvent::Press(event) => match event {
                CustomActions::Uf2 => hal::rom_data::reset_to_usb_boot(0, 0),
                CustomActions::Reset => cortex_m::peripheral::SCB::sys_reset(),
            },
            _ => (),
        }
        let report: KbHidReport = c.local.layout.keycodes().collect();
        let mut usb_class = c.shared.usb_class;
        if usb_class.lock(|k| k.device_mut().set_keyboard_report(report.clone())) {
            rprintln!("{:?}", report.as_bytes());
            while let Ok(0) = usb_class.lock(|k| k.write(report.as_bytes())) {}
            rprintln!("b");
        }
    }

    #[task(
        capacity = 1,
        priority = 1,
        shared = [&is_right, matrix, usb_dev, usb_class],
        local = [led, uart_r],
    )]
    fn poll_uart(c: poll_uart::Context) {
        let uart = c.local.uart_r;
        // let mut usb_class = c.shared.usb_class;
        let is_right: bool = *c.shared.is_right;
        '_outter: loop {
            if is_right {
                let mut buffer: [u8; 3] = [77; 3];
                // let mut byte: [u8; 1] = [0];
                let mut offset = 0;

                while offset != buffer.len() {
                    offset += match uart.read_raw(&mut buffer[offset..]) {
                        Ok(bytes_read) => bytes_read,
                        Err(_e) => continue,
                    };
                    if let Some(last_byte_pos) =
                        buffer.into_iter().position(|x| (x & 0b1000_0000) > 0)
                    {
                        rprintln!("{:?}", buffer);
                        c.local.led.toggle().unwrap();
                        if last_byte_pos >= 2 {
                            let rec_buf: &[u8] = &buffer[last_byte_pos - 2..last_byte_pos + 1];
                            // rprintln!("{:?}", rec_buf);
                            // The buffer has a full packet
                            let kpou: [bool; 17] = [
                                (rec_buf[0] & 0b0000_0001) != 0,
                                (rec_buf[0] & 0b0000_0010) != 0,
                                (rec_buf[0] & 0b0000_0100) != 0,
                                (rec_buf[0] & 0b0000_1000) != 0,
                                (rec_buf[0] & 0b0001_0000) != 0,
                                (rec_buf[0] & 0b0010_0000) != 0,
                                (rec_buf[0] & 0b0100_0000) != 0,
                                (rec_buf[1] & 0b0000_0001) != 0,
                                (rec_buf[1] & 0b0000_0010) != 0,
                                (rec_buf[1] & 0b0000_0100) != 0,
                                (rec_buf[1] & 0b0000_1000) != 0,
                                (rec_buf[1] & 0b0001_0000) != 0,
                                (rec_buf[1] & 0b0010_0000) != 0,
                                (rec_buf[1] & 0b0100_0000) != 0,
                                (rec_buf[2] & 0b0000_0001) != 0,
                                (rec_buf[2] & 0b0000_0010) != 0,
                                (rec_buf[2] & 0b0000_0100) != 0,
                            ];
                            let kptu = c.shared.matrix.get().unwrap()[0];

                            #[rustfmt::skip]
                            let keys_pressed: [[bool; 40]; 1] = [[
                                kpou[4], kpou[3], kpou[2], kpou[1], kpou[0], kptu[0], kptu[1], kptu[2], kptu[3], kptu[4],
                                kpou[9], kpou[8], kpou[7], kpou[6], kpou[5], kptu[5], kptu[6], kptu[7], kptu[8], kptu[9],
                                kpou[14], kpou[13], kpou[12], kpou[11], kpou[10], kptu[10], kptu[11], kptu[12], kptu[13], kptu[14],
                                false, false, false, kpou[16], kpou[15], kptu[15], kptu[16], false, false, false
                            ]];

                            if handle_event::spawn(keys_pressed).is_err() {
                                rprintln!("f");
                            }
                        }
                        break;
                    }
                }
            }
        }
    }

    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        shared = [&is_right, matrix],
        local = [alarm, uart_w],
    )]
    fn scan_timer_irq(c: scan_timer_irq::Context) {
        let alarm = c.local.alarm;
        alarm.clear_interrupt();
        let _ = alarm.schedule(SCAN_INTERVAL);

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
