#![no_std]
#![no_main]

#[rtic::app(device = bsp::hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {

    use bsp::hal::{
        clocks::init_clocks_and_plls,
        gpio::{DynPin, PushPull},
        sio::Sio,
        timer::Alarm,
        usb::UsbBus,
        watchdog::Watchdog,
    };
    use bsp::XOSC_CRYSTAL_FREQ;
    use cortex_m::prelude::_embedded_hal_watchdog_Watchdog;
    use cortex_m::prelude::_embedded_hal_watchdog_WatchdogEnable;
    use defmt_rtt as _;
    use embedded_hal::digital::v2::OutputPin;
    use fugit::MicrosDurationU32;
    use keyberon::action::{
        d, k, l, m, Action,
        Action::{HoldTap, Trans},
        HoldTapAction, HoldTapConfig,
    };
    use keyberon::debounce::Debouncer;
    use keyberon::key_code::KbHidReport;
    use keyberon::key_code::KeyCode::*;
    use keyberon::layout::{Event, Layout};
    use keyberon::matrix::Matrix;
    use panic_probe as _;
    use usb_device::prelude::{UsbDeviceBuilder, UsbVidPid};
    use vcc_gnd_yd_rp2040 as bsp;

    const LCTL_ESC: Action<()> = HoldTap(&HoldTapAction {
        timeout: 200,
        tap_hold_interval: 0,
        config: HoldTapConfig::HoldOnOtherKeyPress,
        hold: k(LCtrl),
        tap: k(Escape),
    });

    const RALT_EDIT: Action<()> = HoldTap(&HoldTapAction {
        timeout: 140,
        tap_hold_interval: 0,
        config: HoldTapConfig::HoldOnOtherKeyPress,
        hold: k(RAlt),
        tap: d(4),
    });

    const TILD: Action<()> = m(&[LShift, Grave].as_slice());
    const EXLM: Action<()> = m(&[LShift, Kb1].as_slice());
    const AT: Action<()> = m(&[LShift, Kb2].as_slice());
    const HASH: Action<()> = m(&[LShift, Kb3].as_slice());
    const DLR: Action<()> = m(&[LShift, Kb4].as_slice());
    const PERC: Action<()> = m(&[LShift, Kb5].as_slice());
    const CIRC: Action<()> = m(&[LShift, Kb6].as_slice());
    const AMPR: Action<()> = m(&[LShift, Kb7].as_slice());
    const ASTR: Action<()> = m(&[LShift, Kb8].as_slice());
    const LPRN: Action<()> = m(&[LShift, Kb9].as_slice());
    const RPRN: Action<()> = m(&[LShift, Kb0].as_slice());
    const UNDS: Action<()> = m(&[LShift, Minus].as_slice());
    const PLUS: Action<()> = m(&[LShift, Equal].as_slice());
    const LCBR: Action<()> = m(&[LShift, LBracket].as_slice());
    const RCBR: Action<()> = m(&[LShift, RBracket].as_slice());
    const PIPE: Action<()> = m(&[LShift, Bslash].as_slice());
    const COPY: Action<()> = m(&[LCtrl, C].as_slice());
    const PASTE: Action<()> = m(&[LCtrl, V].as_slice());
    const VSFMT: Action<()> = m(&[LCtrl, K, F].as_slice());

    #[rustfmt::skip]
    pub const LAYERS: keyberon::layout::Layers<14, 4, 5, ()> = [
        [
            [k(Tab),    k(Q),     k(W),    k(E),    k(R), k(T),     Trans,     Trans,     k(Y),      k(U), k(I),     k(O),    k(P),      k(Minus)],
            [LCTL_ESC,  k(A),     k(S),    k(D),    k(F), k(G),     Trans,     Trans,     k(H),      k(J), k(K),     k(L),    k(SColon), k(Quote)],
            [k(LShift), k(Z),     k(X),    k(C),    k(V), k(B),     l(3),      k(RShift), k(N),      k(M), k(Comma), k(Dot),  k(Slash),  k(Enter)],
            [k(Grave),  k(LCtrl), k(LAlt), k(LGui), l(1), k(Space), RALT_EDIT, k(RAlt),   k(BSpace), l(2), k(Left),  k(Down), k(Up),     k(Right)],
        ],
        [
            [TILD,      EXLM,  AT,    HASH,  DLR,    PERC,   Trans, Trans, CIRC,   AMPR,   ASTR,             LPRN,            RPRN,          k(Delete)],
            [k(Delete), k(F1), k(F2), k(F3), k(F4),  k(F5),  Trans, Trans, k(F6),  UNDS,   PLUS,             LCBR,            RCBR,          PIPE],
            [Trans,     k(F7), k(F8), k(F9), k(F10), k(F11), Trans, Trans, k(F12), k(End), Trans,            Trans,           Trans,         Trans],
            [Trans,     Trans, Trans, Trans, Trans,  Trans,  Trans, Trans, Trans,  Trans,  k(MediaNextSong), k(MediaVolDown), k(MediaVolUp), k(MediaPlayPause)],
        ],
        [
            [k(Grave),  k(Kb1), k(Kb2), k(Kb3), k(Kb4), k(Kb5), Trans, Trans, k(Kb6), k(Kb7),   k(Kb8),           k(Kb9),          k(Kb0),        k(Delete)],
            [k(Delete), k(F1),  k(F2),  k(F3),  k(F4),  k(F5),  Trans, Trans, k(F6),  k(Minus), k(Equal),         k(LBracket),     k(RBracket),   k(Bslash)],
            [Trans,     k(F7),  k(F8),  k(F9),  k(F10), k(F11), Trans, Trans, k(F12), k(End),   Trans,            Trans,           Trans,         Trans],
            [Trans,     Trans,  Trans,  Trans,  Trans,  Trans,  Trans, Trans, Trans,  Trans,    k(MediaNextSong), k(MediaVolDown), k(MediaVolUp), k(MediaPlayPause)],
        ],
        [
            [TILD,      EXLM,  AT,    HASH,  DLR,    PERC,   Trans, Trans, CIRC,       AMPR,    k(Up),            LPRN,           RPRN,           k(Delete)],
            [k(Delete), k(F1), k(F2), k(F3), k(F4),  k(F5),  Trans, Trans, k(F6),      k(Left), k(Down),          k(Right),        RCBR,          PIPE],
            [Trans,     k(F7), k(F8), k(F9), k(F10), k(F11), Trans, Trans, k(F12),     k(End),  Trans,            Trans,           Trans,         Trans],
            [Trans,     Trans, Trans, Trans, Trans,  Trans,  Trans, Trans, k(PgDown),  k(PgUp), k(MediaNextSong), k(MediaVolDown), k(MediaVolUp), k(MediaPlayPause)],
        ],
        [
            [k(Tab),    k(Q),     k(W),    k(E),    k(R),  k(T),     Trans, Trans,     k(Y),      k(U), k(I),     k(O),    k(P),      k(Minus)],
            [LCTL_ESC,  k(A),     k(S),    PASTE,   COPY,  k(G),     Trans, Trans,     k(H),      k(J), k(K),     k(L),    k(SColon), k(Quote)],
            [k(LShift), k(Z),     k(X),    k(C),    VSFMT, k(B),     l(3),  k(RShift), k(N),      k(M), k(Comma), k(Dot),  k(Slash),  k(Enter)],
            [k(Grave),  k(LCtrl), k(LAlt), k(LGui), l(1),  k(Space), d(0),  k(RAlt),   k(BSpace), l(2), k(Left),  k(Down), k(Up),     k(Right)],
        ],
    ];

    use usb_device::class_prelude::*;
    use usb_device::device::UsbDeviceState;

    const SCAN_TIME_US: MicrosDurationU32 = MicrosDurationU32::millis(1);

    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<bsp::hal::usb::UsbBus>> = None;
    const VID: u16 = 0x16c0;
    const PID: u16 = 0x27db;

    #[shared]
    struct Shared {
        usb_dev: usb_device::device::UsbDevice<'static, bsp::hal::usb::UsbBus>,
        usb_class: keyberon::hid::HidClass<
            'static,
            bsp::hal::usb::UsbBus,
            keyberon::keyboard::Keyboard<()>,
        >,
        #[lock_free]
        layout: Layout<14, 4, 5, ()>,
    }

    #[local]
    struct Local {
        watchdog: bsp::hal::watchdog::Watchdog,
        alarm: bsp::hal::timer::Alarm0,
        matrix: Matrix<DynPin, DynPin, 14, 4>,
        debouncer: Debouncer<[[bool; 14]; 4]>,
        led: bsp::hal::gpio::Pin<
            bsp::hal::gpio::pin::bank0::Gpio25,
            bsp::hal::gpio::Output<PushPull>,
        >,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("Starting Keyberon");

        unsafe {
            bsp::hal::sio::spinlock_reset();
        }

        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let sio = Sio::new(c.device.SIO);

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

        let pins = bsp::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let led = pins.led.into_push_pull_output();

        let matrix = Matrix::new(
            [
                pins.gpio2.into_pull_up_input().into(),
                pins.gpio28.into_pull_up_input().into(),
                pins.gpio3.into_pull_up_input().into(),
                pins.gpio27.into_pull_up_input().into(),
                pins.gpio4.into_pull_up_input().into(),
                pins.gpio5.into_pull_up_input().into(),
                pins.gpio26.into_pull_up_input().into(),
                pins.gpio6.into_pull_up_input().into(),
                pins.gpio22.into_pull_up_input().into(),
                pins.gpio7.into_pull_up_input().into(),
                pins.gpio10.into_pull_up_input().into(),
                pins.gpio11.into_pull_up_input().into(),
                pins.gpio12.into_pull_up_input().into(),
                pins.gpio21.into_pull_up_input().into(),
            ],
            [
                pins.gpio13.into_push_pull_output().into(),
                pins.gpio15.into_push_pull_output().into(),
                pins.gpio14.into_push_pull_output().into(),
                pins.gpio20.into_push_pull_output().into(),
            ],
        )
        .unwrap();

        let layout = Layout::new(&LAYERS);
        let debouncer = Debouncer::new([[false; 14]; 4], [[false; 14]; 4], 20);

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

        let usb_dev =
            UsbDeviceBuilder::new(unsafe { USB_BUS.as_ref().unwrap() }, UsbVidPid(VID, PID))
                .manufacturer("Ciota")
                .product("Atreus_52")
                .serial_number(env!("CARGO_PKG_VERSION"))
                .build();

        watchdog.start(MicrosDurationU32::millis(10));

        let mut timer = bsp::hal::Timer::new(c.device.TIMER, &mut resets);
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(SCAN_TIME_US);
        alarm.enable_interrupt();

        defmt::info!("Enabled");

        (
            Shared {
                usb_dev,
                usb_class,
                layout,
            },
            Local {
                watchdog,
                alarm,
                matrix,
                debouncer,
                led,
            },
            init::Monotonics(),
        )
    }

    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [usb_dev, usb_class])]
    fn usb_rx(c: usb_rx::Context) {
        let usb = c.shared.usb_dev;
        let kb = c.shared.usb_class;
        (usb, kb).lock(|usb, kb| {
            if usb.poll(&mut [kb]) {
                kb.poll();
            }
        });
    }

    #[task(priority = 2, capacity = 8, shared = [layout])]
    fn handle_event(c: handle_event::Context, event: Event) {
        c.shared.layout.event(event)
    }

    #[task(priority = 2, local = [led], shared = [usb_dev, usb_class, layout])]
    fn tick_keyberon(mut c: tick_keyberon::Context) {
        if c.shared.usb_dev.lock(|d| d.state()) != UsbDeviceState::Configured {
            return;
        }

        let report: KbHidReport = c.shared.layout.keycodes().collect();
        if !c
            .shared
            .usb_class
            .lock(|k| k.device_mut().set_keyboard_report(report.clone()))
        {
            return;
        }
        c.local.led.set_high().unwrap();
        while let Ok(0) = c.shared.usb_class.lock(|k| k.write(report.as_bytes())) {}
        c.local.led.set_low().unwrap();
    }

    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        shared = [],
        local = [watchdog, alarm, matrix, debouncer],
    )]
    fn scan_timer_irq(cx: scan_timer_irq::Context) {
        let alarm = cx.local.alarm;
        alarm.clear_interrupt();
        let _ = alarm.schedule(SCAN_TIME_US);

        cx.local.watchdog.feed();

        for event in cx.local.debouncer.events(cx.local.matrix.get().unwrap()) {
            handle_event::spawn(event).unwrap();
        }
        tick_keyberon::spawn().unwrap();
    }

    #[idle(local = [])]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }
}
