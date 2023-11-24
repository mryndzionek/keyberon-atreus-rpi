#![no_std]
#![no_main]

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
use cortex_m::prelude::_embedded_hal_watchdog_Watchdog;
use cortex_m::prelude::_embedded_hal_watchdog_WatchdogEnable;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use fugit::MicrosDurationU32;

use keyberon::debounce::Debouncer;
use keyberon::key_code::KbHidReport;
use keyberon::layout::{CustomEvent, Event, Layout};
use keyberon::matrix::Matrix;

use usb_device::class_prelude::*;
use usb_device::device::UsbDeviceState;
use usb_device::prelude::{UsbDeviceBuilder, UsbVidPid};

use vcc_gnd_yd_rp2040 as bsp;

mod layout;

#[rtic::app(device = bsp::hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {
    use super::*;

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
        layout: Layout<14, 4, 4, ()>,
    }

    #[local]
    struct Local {
        watchdog: bsp::hal::watchdog::Watchdog,
        alarm: bsp::hal::timer::Alarm0,
        matrix: Matrix<
            Pin<DynPinId, FunctionSioInput, PullUp>,
            Pin<DynPinId, FunctionSioOutput, PullDown>,
            14,
            4,
        >,
        debouncer: Debouncer<[[bool; 14]; 4]>,
        led: bsp::hal::gpio::Pin<
            bsp::hal::gpio::bank0::Gpio25,
            bsp::hal::gpio::FunctionSio<bsp::hal::gpio::SioOutput>,
            bsp::hal::gpio::PullDown,
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

        let led: bsp::hal::gpio::Pin<
            bsp::hal::gpio::bank0::Gpio25,
            bsp::hal::gpio::FunctionSio<bsp::hal::gpio::SioOutput>,
            bsp::hal::gpio::PullDown,
        > = pins.led.into_push_pull_output();

        let matrix: Matrix<
            bsp::hal::gpio::Pin<
                DynPinId,
                bsp::hal::gpio::FunctionSio<bsp::hal::gpio::SioInput>,
                PullUp,
            >,
            bsp::hal::gpio::Pin<
                DynPinId,
                bsp::hal::gpio::FunctionSio<bsp::hal::gpio::SioOutput>,
                bsp::hal::gpio::PullDown,
            >,
            14,
            4,
        > = Matrix::new(
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

        let mut timer = bsp::hal::Timer::new(c.device.TIMER, &mut resets, &clocks);
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(SCAN_TIME_US);

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
                .manufacturer("Molcos")
                .product("Atreus_52")
                .serial_number(env!("CARGO_PKG_VERSION"))
                .build();

        watchdog.start(MicrosDurationU32::millis(10));
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

    fn reset_to_bootloader() {
        cortex_m::interrupt::disable();

        // jump to usb
        bsp::hal::rom_data::reset_to_usb_boot(0, 0);
        loop {}
    }

    #[task(priority = 2, local = [led], shared = [usb_dev, usb_class, layout])]
    fn tick_keyberon(mut c: tick_keyberon::Context) {
        let tick = c.shared.layout.tick();
        if c.shared.usb_dev.lock(|d| d.state()) != UsbDeviceState::Configured {
            return;
        }
        if let CustomEvent::Release(()) = tick {
            reset_to_bootloader()
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

        for event in cx.local.debouncer.events(
            cx.local
                .matrix
                .get_with_delay(|| cortex_m::asm::delay(1000))
                .unwrap(),
        ) {
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
