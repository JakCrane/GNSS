use embassy_stm32::{bind_interrupts, peripherals, usb};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, UsbDevice};
use static_cell::StaticCell;
use crate::config::UsbPins;

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

pub type UsbDriver = usb::Driver<'static, peripherals::USB_OTG_FS>;

pub struct Usb {
    pub device: UsbDevice<'static, UsbDriver>,
    pub tx: Sender<'static, UsbDriver>,
    pub rx: Receiver<'static, UsbDriver>,
}

pub fn init(pins: UsbPins) -> Usb {
    static EP_OUT_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static STATE: StaticCell<State<'static>> = StaticCell::new();

    let mut usb_config = usb::Config::default();
    usb_config.vbus_detection = false; // VBUS is not routed to PA9 on this board
    let driver = usb::Driver::new_fs(pins.otg, Irqs, pins.dp, pins.dm, EP_OUT_BUFFER.init([0; 256]), usb_config);

    let mut config = embassy_usb::Config::new(0x1209, 0x0001); // pid.codes test VID/PID
    config.manufacturer = Some("GNSS");
    config.product = Some("GNSS receiver bring-up");
    config.serial_number = Some("0002");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        &mut [], // no Microsoft OS descriptors
        CONTROL_BUF.init([0; 64]),
    );
    let class = CdcAcmClass::new(&mut builder, STATE.init(State::new()), 64);
    let device = builder.build();
    let (tx, rx) = class.split();
    Usb { device, tx, rx }
}

/// Background task that keeps the USB device alive (enumeration, control requests).
#[embassy_executor::task]
pub async fn run(mut device: UsbDevice<'static, UsbDriver>) -> ! {
    device.run().await
}

pub async fn write_str(tx: &mut Sender<'static, UsbDriver>, s: &str) -> Result<(), EndpointError> {
    for chunk in s.as_bytes().chunks(64) {
        tx.write_packet(chunk).await?;
    }
    Ok(())
}