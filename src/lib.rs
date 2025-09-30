#![no_std]
#![doc = include_str!("../README.md")]
#![warn(missing_docs)]

//! USB CDC logging & command channel for RP2040 (embassy).
//!
//! This crate provides a zero-heap solution for bidirectional USB communication
//! on RP2040 microcontrollers using the Embassy async framework. It enables both
//! logging output and command input over a standard USB CDC ACM interface.
//!
//! # Quick Start
//!
//! ```rust,no_run
//! use embassy_executor::Spawner;
//! use embassy_sync::channel::Channel;
//! use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
//! use log::info;
//!
//! // Create a channel for receiving commands from USB
//! static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, [u8; 64], 4> = Channel::new();
//!
//! #[embassy_executor::main]
//! async fn main(spawner: Spawner) {
//!     let p = embassy_rp::init(Default::default());
//!     
//!     // Initialize USB logging with Info level
//!     rp_usb_console::start(
//!         spawner,
//!         log::LevelFilter::Info,
//!         p.USB,
//!         COMMAND_CHANNEL.sender()
//!     );
//!     
//!     // Now you can use standard log macros
//!     info!("Hello over USB!");
//!     
//!     // Handle incoming commands in a separate task
//!     spawner.spawn(command_handler()).unwrap();
//! }
//!
//! #[embassy_executor::task]
//! async fn command_handler() {
//!     let receiver = COMMAND_CHANNEL.receiver();
//!     loop {
//!         let command = receiver.receive().await;
//!         // Process received command data
//!         info!("Received command: {:?}", command);
//!     }
//! }
//! ```
//!
//! # Features
//!
//! - **Zero-heap operation**: All buffers are fixed-size and statically allocated
//! - **Non-blocking logging**: Messages are dropped rather than blocking when channels are full
//! - **USB CDC ACM**: Standard USB serial interface compatible with most terminal programs
//! - **Packet fragmentation**: Large messages are automatically split for reliable transmission
//! - **Bidirectional**: Both log output and command input over the same USB connection
//! - **Embassy integration**: Designed for Embassy's async executor on RP2040
//!
//! # Design Principles
//!
//! This crate prioritizes:
//! - **Real-time behavior**: Never blocks the caller, even when USB is disconnected
//! - **Memory efficiency**: Fixed buffers with no dynamic allocation
//! - **Reliability**: Fragmented transmission reduces host-side latency issues
//! - **Safety**: Single-core assumptions with proper synchronization primitives

use core::cell::UnsafeCell;
use core::fmt::{Result as FmtResult, Write};
use embassy_executor::Spawner;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_rp::{bind_interrupts, Peri};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender as UsbSender, State};
use embassy_usb::{Builder, UsbDevice};
use log::{LevelFilter, Log, Metadata, Record};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

/// Size of each USB packet fragment.
const PACKET_SIZE: usize = 64;

/// Fixed-size (255 byte) log/command message buffer with USB packet fragmentation support.
///
/// This struct provides a fixed-size buffer for log messages that can be efficiently
/// transmitted over USB by fragmenting into smaller packets. Messages longer than the
/// buffer capacity are automatically truncated with an ellipsis indicator.
///
/// ```
pub struct LogMessage {
    len: usize,
    buf: [u8; 255],
}

impl LogMessage {
    /// Create an empty message buffer.
    pub fn new() -> Self {
        Self { len: 0, buf: [0; 255] }
    }

    /// Append a string (UTF-8 bytes) truncating if capacity exceeded.
    ///
    /// If the message would exceed the 255-byte capacity, it is truncated
    /// and the last three bytes are replaced with dots to indicate truncation.
    fn push_str(&mut self, s: &str) {
        for &b in s.as_bytes() {
            if self.len >= self.buf.len() {
                self.buf[self.len - 1] = b'.'; // Indicate truncation with ellipsis
                self.buf[self.len - 2] = b'.';
                self.buf[self.len - 3] = b'.';
                break;
            }
            self.buf[self.len] = b;
            self.len += 1;
        }
    }

    /// Number of 64-byte USB packets required to send this message.
    ///
    /// Returns the minimum number of USB packets needed to transmit the
    /// entire message content.
    pub fn packet_count(&self) -> usize {
        self.len / PACKET_SIZE + if self.len % PACKET_SIZE == 0 { 0 } else { 1 }
    }

    /// Slice for a specific packet index (0-based) containing that chunk.
    ///
    /// Returns the bytes for the specified packet index. The last packet
    /// may be shorter than `PACKET_SIZE` if the message doesn't divide evenly.
    ///
    pub fn as_packet_bytes(&self, packet_index: usize) -> &[u8] {
        let start = core::cmp::min(packet_index * PACKET_SIZE, self.len);
        let end = core::cmp::min(start + PACKET_SIZE, self.len);
        &self.buf[start..end]
    }
}

impl Write for LogMessage {
    fn write_str(&mut self, s: &str) -> FmtResult {
        self.push_str(s);
        Ok(())
    }
}

/// Channel type for sending log messages from the application to the USB sender task.
type LogChannel = Channel<CriticalSectionRawMutex, LogMessage, 4>;
static LOG_CHANNEL: LogChannel = Channel::new();

/// Internal logger implementation that forwards formatted lines to the USB channel.
///
/// This logger formats log records and sends them through the internal channel
/// to be transmitted over USB. If the channel is full, messages are silently
/// dropped to maintain non-blocking behavior.
struct USBLogger;

impl Log for USBLogger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        true
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let mut message = LogMessage::new();
            let path = if let Some(p) = record.module_path() { p } else { "" };
            if write!(&mut message, "[{}] {}: {}\r\n", record.level(), path, record.args()).is_ok() {
                // Non-blocking send. If the channel is full, the message is dropped.
                let _ = LOG_CHANNEL.try_send(message);
            }
        }
    }
    fn flush(&self) {}
}

static LOGGER: USBLogger = USBLogger;

/// USB receive task that handles incoming data from the host.
///
/// This task waits for USB connection, then continuously reads incoming packets
/// and forwards them to the provided command channel. Each received packet is
/// exactly 64 bytes and represents raw USB data from the host.
///
/// The task automatically handles USB disconnection/reconnection cycles.
#[embassy_executor::task]
async fn usb_rx_task(mut receiver: Receiver<'static, Driver<'static, USB>>, command_sender: Sender<'static, CriticalSectionRawMutex, [u8; 64], 4>) {
    loop {
        receiver.wait_connection().await;
        let mut buf = [0u8; 64];
        loop {
            match receiver.read_packet(&mut buf).await {
                Ok(_) => {
                    command_sender.send(buf).await;
                }
                Err(_) => break,
            }
        }
    }
}

/// USB transmit task that sends log messages over USB.
///
/// This task drains the internal log channel and transmits each message by
/// fragmenting it into 64-byte USB packets. Large messages are automatically
/// split across multiple packets for reliable transmission.
///
/// The task handles USB disconnection gracefully and will resume transmission
/// when the connection is restored.
#[embassy_executor::task]
async fn usb_tx_task(mut sender: UsbSender<'static, Driver<'static, USB>>) {
    loop {
        sender.wait_connection().await;
        loop {
            // Wait for a log message from the channel.
            let message = LOG_CHANNEL.receive().await;
            let mut problem = false;
            for i in 0..message.packet_count() {
                let packet = message.as_packet_bytes(i);

                // Send the log message over USB. If sending fails, break to wait for reconnection.
                if sender.write_packet(packet).await.is_err() {
                    problem = true;
                }
            }
            if problem {
                break;
            }
        }
    }
}

/// USB device task that runs the USB device state machine.
///
/// This task handles the low-level USB device protocol and must be spawned
/// for USB communication to function. It manages device enumeration,
/// configuration, and the USB control endpoint.
#[embassy_executor::task]
async fn usb_device_task(mut usb: UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await;
}

/// Initialize USB CDC logging and command channel, spawning all necessary tasks.
///
/// This function sets up the USB CDC ACM interface for bidirectional communication
/// and spawns three tasks to handle the USB device, transmission, and reception.
/// It also initializes the global logger to forward log messages over USB.
///
/// # Arguments
///
/// * `spawner` - Embassy task spawner for creating the USB tasks
/// * `level` - Maximum log level to transmit (e.g., `LevelFilter::Info`)
/// * `usb_peripheral` - RP2040 USB peripheral instance
/// * `command_sender` - Channel sender for receiving commands from the host
///
/// # Panics
///
/// Panics if called more than once, as it sets the global logger.
///
/// # Examples
///
/// ```rust,no_run
/// use embassy_executor::Spawner;
/// use embassy_sync::channel::Channel;
/// use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
/// use log::LevelFilter;
/// # use rp_usb_console;
///
/// static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, [u8; 64], 4> = Channel::new();
///
/// # async fn example(spawner: Spawner, usb_peripheral: embassy_rp::peripherals::USB) {
/// rp_usb_console::start(
///     spawner,
///     LevelFilter::Info,
///     usb_peripheral,
///     COMMAND_CHANNEL.sender()
/// );
/// # }
/// ```
pub fn start(spawner: Spawner, level: LevelFilter, usb_peripheral: Peri<'static, USB>, command_sender: Sender<'static, CriticalSectionRawMutex, [u8; 64], 4>) {
    // Initialize the logger (use racy variants on targets without atomic ptr support, e.g. thumbv6m/RP2040)
    unsafe {
        log::set_logger_racy(&LOGGER).unwrap();
        log::set_max_level_racy(level);
    }

    // Simple wrapper to mark our single-threaded statics as Sync. RP2040 + embassy executor: we ensure single-core access.
    struct StaticCell<T>(UnsafeCell<T>);
    unsafe impl<T> Sync for StaticCell<T> {}

    static DEVICE_DESC: StaticCell<[u8; 256]> = StaticCell(UnsafeCell::new([0; 256]));
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell(UnsafeCell::new([0; 256]));
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell(UnsafeCell::new([0; 256]));
    static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell(UnsafeCell::new([0; 128]));
    static STATE: StaticCell<State> = StaticCell(UnsafeCell::new(State::new()));

    let driver = Driver::new(usb_peripheral, Irqs);

    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("Modular USB-Serial");
    config.serial_number = Some("12345678");
    config.max_power = 100;

    let mut builder = Builder::new(
        driver,
        config,
        unsafe { &mut *DEVICE_DESC.0.get() },
        unsafe { &mut *CONFIG_DESC.0.get() },
        unsafe { &mut *BOS_DESC.0.get() },
        unsafe { &mut *CONTROL_BUF.0.get() },
    );

    let class = CdcAcmClass::new(&mut builder, unsafe { &mut *STATE.0.get() }, 64);
    let (sender, receiver) = class.split();
    let usb = builder.build();

    // Spawn all the necessary tasks.
    spawner.spawn(usb_device_task(usb)).unwrap();
    spawner.spawn(usb_tx_task(sender)).unwrap();
    spawner.spawn(usb_rx_task(receiver, command_sender)).unwrap();
}
