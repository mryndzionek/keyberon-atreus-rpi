#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use panic_probe as _;

use bsp::hal::{
    clocks::init_clocks_and_plls,
    gpio::{DynPinId, FunctionSioInput, FunctionSioOutput, Pin, PullDown, PullUp},
    sio::Sio,
    timer::Alarm,
    usb::UsbBus,
    watchdog::Watchdog,
};
use bsp::XOSC_CRYSTAL_FREQ;

use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use fugit::MicrosDurationU32;

use keyberon::debounce::Debouncer;
use keyberon::key_code::KbHidReport;
use keyberon::layout::{CustomEvent, Event, Layout};
use keyberon::matrix::Matrix;

use usb_device::class_prelude::*;
use usb_device::device::UsbDeviceState;
use usb_device::prelude::{StringDescriptors, UsbDeviceBuilder, UsbVidPid};

use embedded_hal::pwm::SetDutyCycle;
use vcc_gnd_yd_rp2040 as bsp;

use rtic_sync::{
    channel::{Receiver, Sender},
    make_channel,
};

mod layout;

#[rtic::app(device = bsp::hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {

    use super::*;

    const SCAN_TIME_US: MicrosDurationU32 = MicrosDurationU32::millis(1);
    const EV_CHAN_CAPACITY: usize = 9;

    const VID: u16 = 0x16c0;
    const PID: u16 = 0x27db;

    const MAX_LED_BRIGHTNESS: u16 = 10000;

    type KbdMatrix = Matrix<
        Pin<DynPinId, FunctionSioInput, PullUp>,
        Pin<DynPinId, FunctionSioOutput, PullDown>,
        14,
        4,
    >;

    type LedPwm = bsp::hal::pwm::Channel<
        bsp::hal::pwm::Slice<bsp::hal::pwm::Pwm1, bsp::hal::pwm::FreeRunning>,
        bsp::hal::pwm::A,
    >;

    #[derive(Debug)]
    enum KbdEvent {
        Ev { event: Event },
        Tick,
    }

    #[derive(Debug)]
    enum LedEvent {
        KeyPress,
        Tick,
    }

    #[shared]
    struct Shared {
        usb_dev: usb_device::device::UsbDevice<'static, bsp::hal::usb::UsbBus>,
        usb_class: keyberon::hid::HidClass<
            'static,
            bsp::hal::usb::UsbBus,
            keyberon::keyboard::Keyboard<()>,
        >,
    }

    #[local]
    struct Local {
        watchdog: bsp::hal::watchdog::Watchdog,
        alarm: bsp::hal::timer::Alarm0,
        matrix: KbdMatrix,
        debouncer: Debouncer<[[bool; 14]; 4]>,
        led: Pin<bsp::hal::gpio::bank0::Gpio25, FunctionSioOutput, PullDown>,
        ev_sender: Sender<'static, KbdEvent, EV_CHAN_CAPACITY>,
        layout: Layout<14, 4, 4, ()>,
        led_pwm: LedPwm,
        led_ch_s: Sender<'static, LedEvent, 4>,
    }

    #[init(local = [bus: Option<UsbBusAllocator<UsbBus>> = None])]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("Starting Keyberon");

        unsafe {
            bsp::hal::sio::spinlock_reset();
        }

        let mut resets = cx.device.RESETS;
        let mut watchdog = Watchdog::new(cx.device.WATCHDOG);
        let sio = Sio::new(cx.device.SIO);

        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            cx.device.XOSC,
            cx.device.CLOCKS,
            cx.device.PLL_SYS,
            cx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let pins = bsp::Pins::new(
            cx.device.IO_BANK0,
            cx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let led = pins.led.into_push_pull_output();

        let matrix: KbdMatrix = Matrix::new(
            [
                pins.gpio29.into_pull_up_input().into_dyn_pin(),
                pins.gpio28.into_pull_up_input().into_dyn_pin(),
                pins.gpio27.into_pull_up_input().into_dyn_pin(),
                pins.gpio26.into_pull_up_input().into_dyn_pin(),
                pins.gpio22.into_pull_up_input().into_dyn_pin(),
                pins.gpio19.into_pull_up_input().into_dyn_pin(),
                pins.gpio18.into_pull_up_input().into_dyn_pin(),
                pins.gpio14.into_pull_up_input().into_dyn_pin(),
                pins.gpio11.into_pull_up_input().into_dyn_pin(),
                pins.gpio9.into_pull_up_input().into_dyn_pin(),
                pins.gpio8.into_pull_up_input().into_dyn_pin(),
                pins.gpio7.into_pull_up_input().into_dyn_pin(),
                pins.gpio6.into_pull_up_input().into_dyn_pin(),
                pins.gpio3.into_pull_up_input().into_dyn_pin(),
            ],
            [
                pins.gpio4.into_push_pull_output().into_dyn_pin(),
                pins.gpio20.into_push_pull_output().into_dyn_pin(),
                pins.gpio12.into_push_pull_output().into_dyn_pin(),
                pins.gpio16.into_push_pull_output().into_dyn_pin(),
            ],
        )
        .unwrap();

        let layout = Layout::new(&crate::layout::LAYERS);
        let debouncer = Debouncer::new([[false; 14]; 4], [[false; 14]; 4], 5);

        let pwm_slices = bsp::hal::pwm::Slices::new(cx.device.PWM, &mut resets);
        let mut pwm = pwm_slices.pwm1;
        pwm.set_ph_correct();
        pwm.enable();

        let mut led_pwm = pwm.channel_a;
        led_pwm.output_to(pins.gpio2.into_push_pull_output());

        let mut timer = bsp::hal::Timer::new(cx.device.TIMER, &mut resets, &clocks);
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(SCAN_TIME_US);

        let (ev_sender, r) = make_channel!(KbdEvent, EV_CHAN_CAPACITY);
        handle_event::spawn(r).unwrap();

        let (led_ch_s, led_ch_r) = make_channel!(LedEvent, 4);
        led_task::spawn(led_ch_r).unwrap();

        *cx.local.bus = Some(UsbBusAllocator::new(UsbBus::new(
            cx.device.USBCTRL_REGS,
            cx.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        )));

        let usb_bus = cx.local.bus.as_ref().unwrap();

        let usb_class = keyberon::new_class(usb_bus, ());

        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(VID, PID))
            .strings(&[StringDescriptors::default()
                .manufacturer("Molcos")
                .product("Atreus_52")
                .serial_number(concat!(env!("GIT_HASH"), "-", env!("BUILD_TIMESTAMP")))])
            .unwrap()
            .device_class(3)
            .build();

        watchdog.start(MicrosDurationU32::millis(10));
        alarm.enable_interrupt();

        defmt::info!("Enabled");

        (
            Shared { usb_dev, usb_class },
            Local {
                watchdog,
                alarm,
                matrix,
                debouncer,
                led,
                ev_sender,
                layout,
                led_pwm,
                led_ch_s,
            },
        )
    }

    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [usb_dev, usb_class])]
    fn usb_rx(cx: usb_rx::Context) {
        let usb = cx.shared.usb_dev;
        let kb = cx.shared.usb_class;
        (usb, kb).lock(|usb, kb| {
            if usb.poll(&mut [kb]) {
                kb.poll();
            }
        });
    }

    #[task(priority = 2, local = [layout, led], shared = [usb_dev, usb_class])]
    async fn handle_event(
        mut cx: handle_event::Context,
        mut receiver: Receiver<'static, KbdEvent, EV_CHAN_CAPACITY>,
    ) {
        let layout = cx.local.layout;

        while let Ok(event) = receiver.recv().await {
            match event {
                KbdEvent::Ev { event } => layout.event(event),
                KbdEvent::Tick => {
                    let tick = layout.tick();

                    if cx.shared.usb_dev.lock(|d| d.state()) != UsbDeviceState::Configured {
                        continue;
                    }
                    if let CustomEvent::Release(()) = tick {
                        reset_to_bootloader()
                    }

                    let report: KbHidReport = layout.keycodes().collect();

                    if !cx
                        .shared
                        .usb_class
                        .lock(|k| k.device_mut().set_keyboard_report(report.clone()))
                    {
                        continue;
                    }

                    cx.local.led.set_high().unwrap();
                    while let Ok(0) = cx.shared.usb_class.lock(|k| k.write(report.as_bytes())) {}
                    cx.local.led.set_low().unwrap();
                }
            }
        }
    }

    fn brightness_to_duty(bri: u16) -> u16 {
        let mut duty = bri as u32;
        duty *= duty;
        duty /= u16::MAX as u32;
        u16::MAX - (duty as u16)
    }

    #[task(priority = 2, local = [led_pwm])]
    async fn led_task(cx: led_task::Context, mut receiver: Receiver<'static, LedEvent, 4>) {
        let led_pwm = cx.local.led_pwm;
        let mut state = false;
        let mut brightness: u16 = 0;
        let mut dir = true;
        let mut counter: u16 = 0;

        while let Ok(event) = receiver.recv().await {
            match state {
                false => match event {
                    LedEvent::Tick => {
                        if dir {
                            brightness += 5;
                            if brightness == MAX_LED_BRIGHTNESS {
                                dir = false;
                            }
                        } else {
                            brightness -= 5;
                            if brightness == 0 {
                                dir = true;
                            }
                        }
                        led_pwm
                            .set_duty_cycle(brightness_to_duty(brightness))
                            .unwrap();
                    }
                    LedEvent::KeyPress => {
                        led_pwm
                            .set_duty_cycle(brightness_to_duty(4 * MAX_LED_BRIGHTNESS))
                            .unwrap();
                        counter = 10;
                        state = true;
                    }
                },
                true => match event {
                    LedEvent::Tick => {
                        counter -= 1;
                        if counter == 0 {
                            led_pwm
                                .set_duty_cycle(brightness_to_duty(brightness))
                                .unwrap();
                            state = false;
                        }
                    }
                    LedEvent::KeyPress => {
                        counter = 10;
                    }
                },
            }
        }
    }

    fn reset_to_bootloader() {
        cortex_m::interrupt::disable();

        // jump to usb
        bsp::hal::rom_data::reset_to_usb_boot(0, 0);
        loop {
            cortex_m::asm::delay(1000)
        }
    }

    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        shared = [],
        local = [watchdog, alarm, matrix, debouncer, ev_sender, led_ch_s],
    )]
    fn scan_timer_irq(cx: scan_timer_irq::Context) {
        let alarm = cx.local.alarm;
        alarm.clear_interrupt();
        let _ = alarm.schedule(SCAN_TIME_US);

        cx.local.watchdog.feed();

        for event in cx.local.debouncer.events(
            cx.local
                .matrix
                .get_with_delay(|| cortex_m::asm::delay(1000))
                .unwrap(),
        ) {
            cx.local.ev_sender.try_send(KbdEvent::Ev { event }).unwrap();
            if let Event::Press(_, _) = event {
                cx.local.led_ch_s.try_send(LedEvent::KeyPress).unwrap();
            }
        }
        cx.local.ev_sender.try_send(KbdEvent::Tick).unwrap();
        cx.local.led_ch_s.try_send(LedEvent::Tick).unwrap();
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }
}
