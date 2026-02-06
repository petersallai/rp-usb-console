#![no_std]
#![doc = include_str!("../README.md")]
#![warn(missing_docs)]

//! USB CDC logging & command channel for RP2040 and RP2350 (embassy).
//!
//! This crate provides a zero-heap solution for bidirectional USB communication
//! on RP2040 and RP2350 microcontrollers using the Embassy async framework.
//! It enables both logging output and line-buffered command input
//!  (terminated by `\r`, `\n`, or `\r\n`) over a standard USB CDC ACM interface.
//!
//! # Quick Start
//!
//! ```rust,no_run
//! use embassy_executor::Spawner;
//! use embassy_sync::channel::Channel;
//! use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
//! use log::info;
//! use rp_usb_console::USB_READ_BUFFER_SIZE;
//!
//! // Create a channel for receiving line-buffered commands from USB
//! static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, [u8; USB_READ_BUFFER_SIZE], 4> = Channel::new();
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
//!         Some(COMMAND_CHANNEL.sender()),
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
//!         // Process received command data (trailing zeros may be present)
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
//! - **Bidirectional**: Both log output and line-buffered command input over the same USB connection
//! - **Configurable command sink**: Forward parsed commands to your own channel or disable command forwarding entirely
//! - **Embassy integration**: Designed for Embassy's async executor on RP2040
//!
//! # Design Principles
//!
//! This crate prioritizes:
//! - **Real-time behavior**: Never blocks the caller, even when USB is disconnected
//! - **Memory efficiency**: Fixed buffers with no dynamic allocation
//! - **Reliability**: Fragmented transmission reduces host-side latency issues
//! - **Safety**: Single-core assumptions with proper synchronization primitives

#[cfg(all(feature = "rp2040", feature = "rp235xa"))]
compile_error!("Feature 'rp2040' and 'rp235xa' cannot be enabled at the same time. You'll have to decide on the hardware you are using!");
#[cfg(not(any(feature = "rp2040", feature = "rp235xa")))]
compile_error!("Select feature 'rp2040' or 'rp235xa' depending on your hardware.");

use core::cell::{RefCell, UnsafeCell};
use core::cmp::min;
use core::fmt::{Result as FmtResult, Write};
use critical_section::Mutex as CsMutex;
use embassy_executor::Spawner;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_rp::{Peri, bind_interrupts};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver as UsbReceiver, Sender as UsbSender, State};
use embassy_usb::{Builder, UsbDevice};
use log::{Level, LevelFilter, Log, Metadata, Record};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

/// Size of each USB packet fragment.
const PACKET_SIZE: usize = 64;
const MODULE_FILTER_CAPACITY: usize = 255;
/// Size (in bytes) of the receive buffer used to accumulate incoming USB command data before processing.
///
/// Applications that forward commands should allocate their channels using this
/// buffer size to avoid truncation.
pub const USB_READ_BUFFER_SIZE: usize = 255;

#[derive(Copy, Clone)]
struct LogModuleSettings {
    module_filter: [u8; MODULE_FILTER_CAPACITY],
    module_filter_len: usize,
    module_level: LevelFilter,
    other_level: LevelFilter,
}

impl LogModuleSettings {
    fn new(module_name: &str, module_level: LevelFilter, other_level: LevelFilter) -> Self {
        let mut buf = [0u8; MODULE_FILTER_CAPACITY];
        let bytes = module_name.as_bytes();
        let len = min(bytes.len(), MODULE_FILTER_CAPACITY);
        buf[..len].copy_from_slice(&bytes[..len]);

        Self {
            module_filter: buf,
            module_filter_len: len,
            module_level,
            other_level,
        }
    }

    fn module_name(&self) -> &str {
        core::str::from_utf8(&self.module_filter[..self.module_filter_len]).unwrap_or("")
    }
}

/// Fixed-size (255 byte) log/command message buffer with USB packet fragmentation support.
///
/// This struct provides a fixed-size buffer for log messages that can be efficiently
/// transmitted over USB by fragmenting into smaller packets. Messages longer than the
/// buffer capacity are automatically truncated with an ellipsis indicator.
///
/// ```
pub struct LogMessage {
    len: usize,
    /// Byte buffer for messages
    pub buf: [u8; 255],
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
    pub fn push_str(&mut self, s: &str) {
        self.push_bytes(s.as_bytes())
    }

    /// Append bytes (UTF-8 bytes) truncating if capacity exceeded.
    ///
    /// If the message would exceed the 255-byte capacity, it is truncated
    /// and the last three bytes are replaced with dots to indicate truncation.
    fn push_bytes(&mut self, bs: &[u8]) {
        for &b in bs {
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
///
/// Each `LogMessage` contains a 255-byte buffer, so the total memory usage of this channel is
/// approximately `CAPACITY × 255` bytes plus channel bookkeeping. A capacity of 32 was chosen
/// as a balance between buffering bursty logs and conserving RAM on RP2040-class MCUs with
/// limited memory.
///
/// Earlier revisions of this module experimented with higher capacities (for example, 100
/// messages), but this significantly increased static RAM usage without a proportional benefit
/// on typical RP2040 workloads. The current capacity of 32 is therefore an intentional
/// compromise. Increase this value only if profiling shows frequent log drops and your
/// application can afford the additional static RAM usage; conversely, you may reduce it further
/// on extremely memory-constrained systems at the cost of more aggressive log dropping.
type LogChannel = Channel<CriticalSectionRawMutex, LogMessage, 10>;
static LOG_CHANNEL: LogChannel = Channel::new();

// Log settings protected by critical section for dual-core safety.
// RP2040's critical sections use hardware spinlocks to synchronize between cores.
static LOG_SETTINGS: CsMutex<RefCell<Option<LogModuleSettings>>> = CsMutex::new(RefCell::new(None));

/// Read current log settings (inside critical section).
#[inline]
fn get_log_settings() -> Option<LogModuleSettings> {
    critical_section::with(|cs| *LOG_SETTINGS.borrow(cs).borrow())
}

/// Update log settings (inside critical section).
#[inline]
fn set_log_settings(settings: Option<LogModuleSettings>) {
    critical_section::with(|cs| {
        *LOG_SETTINGS.borrow(cs).borrow_mut() = settings;
    });
}

fn parse_level_filter(token: &str) -> Option<LevelFilter> {
    let mut chars = token.chars();
    let ch = chars.next()?.to_ascii_uppercase();
    if chars.next().is_some() {
        return None;
    }
    match ch {
        'T' => Some(LevelFilter::Trace),
        'D' => Some(LevelFilter::Debug),
        'I' => Some(LevelFilter::Info),
        'W' => Some(LevelFilter::Warn),
        'E' => Some(LevelFilter::Error),
        'O' => Some(LevelFilter::Off),
        _ => None,
    }
}

fn level_allowed(filter: LevelFilter, level: Level) -> bool {
    match filter.to_level() {
        Some(max) => level <= max,
        None => false,
    }
}

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
            let should_emit = match get_log_settings() {
                Some(settings) => {
                    let module_name = settings.module_name();
                    let target_filter = match record.module_path() {
                        Some(path) if !module_name.is_empty() && path.contains(module_name) => settings.module_level,
                        _ => settings.other_level,
                    };
                    level_allowed(target_filter, record.level())
                }
                None => true,
            };

            if !should_emit {
                return;
            }

            let mut message = LogMessage::new();
            let path = if let Some(p) = record.module_path() { p } else { "" };
            if message.len + path.len() + 10 >= 255 {
                return; // Avoid exceeding buffer capacity
            }
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
/// and accumulates them into a single buffer until a line terminator (`\r`,
/// `\n`, or `\r\n`) is received. Completed lines are dispatched to the optional
/// command channel for application handling, while built-in control commands
/// (such as `/BS` and `/LM`) are processed internally. If `command_sender` is
/// `None`, application commands are ignored after internal processing.
///
/// The task automatically handles USB disconnection/reconnection cycles.
#[embassy_executor::task]
async fn usb_rx_task(
    mut receiver: UsbReceiver<'static, Driver<'static, USB>>,
    command_sender: Option<Sender<'static, CriticalSectionRawMutex, [u8; USB_READ_BUFFER_SIZE], 4>>,
) {
    let mut buf = [0u8; USB_READ_BUFFER_SIZE];
    let mut buf_position: usize = 0;
    let mut echo = cfg!(feature = "echo");
    loop {
        receiver.wait_connection().await;
        let mut read_buf = [0u8; USB_READ_BUFFER_SIZE];
        loop {
            match receiver.read_packet(&mut read_buf).await {
                Ok(len) => {
                    if len + buf_position > USB_READ_BUFFER_SIZE {
                        buf_position = 0;
                    }
                    buf[buf_position..buf_position + len].copy_from_slice(&read_buf[..len]);
                    buf_position += len;

                    if echo {
                        let mut message = LogMessage::new();
                        message.push_bytes(&read_buf[..len]);
                        let _ = LOG_CHANNEL.try_send(message);
                    }

                    if !(buf.contains(&b'\n') || buf.contains(&b'\r')) {
                        continue; // Wait for more data
                    }

                    let mut processed = false;
                    if let Ok(command_str) = core::str::from_utf8(&buf[0..3]) {
                        match command_str {
                            "/BS" => {
                                embassy_rp::rom_data::reset_to_usb_boot(0, 0);
                            }
                            "/RS" => {
                                processed = true;
                                const REBOOT2_FLAG_NO_RETURN_ON_SUCCESS: u32 = 0x100;
                                embassy_rp::rom_data::reboot(REBOOT2_FLAG_NO_RETURN_ON_SUCCESS, 10, 0, 0);
                            }
                            "/E0" => {
                                processed = true;
                                echo = false;
                                log::debug!("Echo disabled");
                            }
                            "/E1" => {
                                processed = true;
                                echo = true;
                                log::debug!("Echo enabled");
                            }
                            "/LT" => {
                                processed = true;
                                unsafe {
                                    log::set_max_level_racy(LevelFilter::Trace);
                                }
                                log::info!("Log level set to Trace");
                            }
                            "/LD" => {
                                processed = true;
                                unsafe {
                                    log::set_max_level_racy(LevelFilter::Debug);
                                }
                                log::info!("Log level set to Debug");
                            }
                            "/LI" => {
                                processed = true;
                                unsafe {
                                    log::set_max_level_racy(LevelFilter::Info);
                                }
                                log::info!("Log level set to Info");
                            }
                            "/LW" => {
                                processed = true;
                                unsafe {
                                    log::set_max_level_racy(LevelFilter::Warn);
                                }
                                log::warn!("Log level set to Warn");
                            }
                            "/LE" => {
                                processed = true;
                                unsafe {
                                    log::set_max_level_racy(LevelFilter::Error);
                                }
                                log::error!("Log level set to Error");
                            }
                            "/LO" => {
                                processed = true;
                                unsafe {
                                    log::set_max_level_racy(LevelFilter::Off);
                                }
                                // Cannot log here since logging is now off
                            }
                            "/LM" => {
                                processed = true;
                                if let Ok(param_string) = core::str::from_utf8(&buf[4..]) {
                                    let param_string = param_string.trim_matches(char::from(0)).trim();

                                    let mut parts = param_string.splitn(2, ',');
                                    match (parts.next(), parts.next()) {
                                        (Some(module_filter), Some(module_log_level)) => {
                                            let module_filter = module_filter.trim();
                                            let module_level_str = module_log_level.trim();

                                            if module_filter.is_empty() {
                                                log::error!("Invalid /LM parameters '{}'. Module name cannot be empty", param_string);
                                                buf_position = 0; // Reset buffer for next command
                                                buf = [0u8; USB_READ_BUFFER_SIZE];
                                                continue;
                                            }

                                            if module_level_str == "-" {
                                                log::info!("Module logging override cleared for '{}'", module_filter);
                                                if let Some(settings) = get_log_settings() {
                                                    unsafe {
                                                        log::set_max_level_racy(settings.other_level);
                                                    }
                                                }
                                                set_log_settings(None);
                                                buf_position = 0; // Reset buffer for next command
                                                buf = [0u8; USB_READ_BUFFER_SIZE];
                                                continue;
                                            }

                                            let Some(module_level) = parse_level_filter(module_level_str) else {
                                                log::error!("Invalid /LM module level '{}'. Use one of T,D,I,W,E,O", module_level_str);
                                                buf_position = 0; // Reset buffer for next command
                                                buf = [0u8; USB_READ_BUFFER_SIZE];
                                                continue;
                                            };
                                            let other_level = log::max_level();

                                            unsafe {
                                                log::set_max_level_racy(module_level);
                                            }

                                            let settings = LogModuleSettings::new(module_filter, module_level, other_level);
                                            set_log_settings(Some(settings));

                                            log::info!(
                                                "Module logging override: module='{}' module_level={:?}",
                                                module_filter,
                                                module_level
                                            );
                                        }
                                        _ => {
                                            log::error!(
                                                "Invalid /LM parameters '{}'. Expected format: /LM <module_filter>,<module_log_level[T|D|I|W|E]>",
                                                param_string
                                            );
                                        }
                                    }
                                }
                            }

                            _ => {}
                        }
                    }
                    if echo {
                        let mut message = LogMessage::new();
                        if processed || buf_position < 2 {
                            message.push_str("\r\n> ");
                        } else if !processed && buf_position > 1 {
                            message.push_str("\r\n");
                        }
                        let _ = LOG_CHANNEL.try_send(message);
                    }
                    if !processed && buf_position > 1 {
                        if let Some(sender) = &command_sender {
                            // Null-terminate the command string
                            if buf_position < USB_READ_BUFFER_SIZE {
                                buf[buf_position] = 0;
                            } else {
                                buf[USB_READ_BUFFER_SIZE - 1] = 0;
                            }
                            // Use try_send to avoid blocking if channel is full
                            let _ = sender.try_send(buf);
                        }
                    }
                    buf_position = 0; // Reset buffer for next command
                    buf = [0u8; USB_READ_BUFFER_SIZE];
                }
                Err(_) => break,
            }
        }
    }
}

#[embassy_executor::task]
async fn data_receiver_task(command_sender: Option<Receiver<'static, CriticalSectionRawMutex, [u8; USB_READ_BUFFER_SIZE], 4>>) {
    if let Some(receiver) = &command_sender {
        loop {
            let message = receiver.receive().await;
            if let Ok(data) = core::str::from_utf8(&message) {
                let mut message = LogMessage::new();
                message.push_str(data);
                let _ = LOG_CHANNEL.try_send(message);
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
/// * `command_sender` - Optional channel sender for receiving line-buffered commands from the host (`None` disables forwarding)
/// * `data_receiver`  - Optional channel receiver for sending commands to the host (`None` disables forwarding)
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
/// static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, [u8; rp_usb_console::USB_READ_BUFFER_SIZE], 4> = Channel::new();
///
/// # async fn example(spawner: Spawner, usb_peripheral: embassy_rp::peripherals::USB) {
/// rp_usb_console::start(
///     spawner,
///     LevelFilter::Info,
///     usb_peripheral,
///     Some(COMMAND_CHANNEL.sender()),
///     None,
/// );
/// # }
/// ```

#[cfg(feature = "log")]
pub fn start(
    spawner: Spawner,
    level: LevelFilter,
    usb_peripheral: Peri<'static, USB>,
    command_sender: Option<Sender<'static, CriticalSectionRawMutex, [u8; USB_READ_BUFFER_SIZE], 4>>,
    data_receiver: Option<Receiver<'static, CriticalSectionRawMutex, [u8; USB_READ_BUFFER_SIZE], 4>>,
) {
    start_impl(spawner, level, usb_peripheral, command_sender, data_receiver)
}

/// Initialize USB command channel, spawning all necessary tasks, but without logging enabled.
///
/// This function sets up the USB CDC ACM interface for bidirectional communication
/// and spawns three tasks to handle the USB device, transmission, and reception.
/// It also initializes the global logger to forward log messages over USB.
///
/// # Arguments
///
/// * `spawner` - Embassy task spawner for creating the USB tasks
/// * `usb_peripheral` - RP2040 USB peripheral instance
/// * `command_sender` - Optional channel sender for receiving line-buffered commands from the host (`None` disables forwarding)
/// * `data_receiver`  - Optional channel receiver for sending commands to the host (`None` disables forwarding)
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
///
/// static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, [u8; rp_usb_console::USB_READ_BUFFER_SIZE], 4> = Channel::new();
///
/// # async fn example(spawner: Spawner, usb_peripheral: embassy_rp::peripherals::USB) {
/// rp_usb_console::start(
///     spawner,
///     usb_peripheral,
///     Some(COMMAND_CHANNEL.sender()),
///     None,
/// );
/// # }
/// ```

#[cfg(not(feature = "log"))]
pub fn start(
    spawner: Spawner,
    usb_peripheral: Peri<'static, USB>,
    command_sender: Option<Sender<'static, CriticalSectionRawMutex, [u8; USB_READ_BUFFER_SIZE], 4>>,
    data_receiver: Option<Receiver<'static, CriticalSectionRawMutex, [u8; USB_READ_BUFFER_SIZE], 4>>,
) {
    start_impl(spawner, log::LevelFilter::Off, usb_peripheral, command_sender, data_receiver)
}

fn start_impl(
    spawner: Spawner,
    level: LevelFilter,
    usb_peripheral: Peri<'static, USB>,
    command_sender: Option<Sender<'static, CriticalSectionRawMutex, [u8; USB_READ_BUFFER_SIZE], 4>>,
    data_receiver: Option<Receiver<'static, CriticalSectionRawMutex, [u8; USB_READ_BUFFER_SIZE], 4>>,
) {
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
    spawner.spawn(data_receiver_task(data_receiver)).unwrap();
}
