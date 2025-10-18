pub use hackrf_sys as ffi;
use core::ptr::NonNull;
use core::ffi::{CStr, c_int, c_void};
use core::ops::ControlFlow;

pub mod options;

/// Get library version.
pub fn version() -> &'static str {
    let bytes;
    unsafe {
        let ptr = ffi::hackrf_library_version();
        assert!(!ptr.is_null(), "hackrf_library_version returned null");
        let cstr = core::ffi::CStr::from_ptr(ptr);
        bytes = cstr.to_bytes();
    }
    core::str::from_utf8(bytes)
        .expect("hackrf_library_version return ASCII strings, this should never fail")
}

/// Get library release.
pub fn release() -> &'static str {
    let bytes;
    unsafe {
        let ptr = ffi::hackrf_library_release();
        assert!(!ptr.is_null(), "hackrf_library_release returned null");
        let cstr = core::ffi::CStr::from_ptr(ptr);
        bytes = cstr.to_bytes();
    }
    core::str::from_utf8(bytes)
        .expect("hackrf_library_release return ASCII strings, this should never fail")
}

/// An entry from `DeviceList`
#[derive(Debug)]
pub struct DeviceEntry<'a> {
    /// Index of the device in the device list
    pub index: usize,
    /// Human-readable serial number.
    pub serial_number: Option<&'a CStr>,
    /// ID of the board, based on USB product ID. Can be used for general HW identification without opening the device.
    pub board_id: HackrfUSBBoardId,
    /// USB device (`*mut libusb_device`)
    pub usb_device: Option<NonNull<c_void>>,
}

/// List of connected HackRF devices
pub struct DeviceList {
    device_list: NonNull<ffi::hackrf_device_list>,
}

impl DeviceList {
    /// Get the list of available HackRF devices
    pub fn new() -> Result<Self, HackrfError> {
        let rv = unsafe { ffi::hackrf_init() };
        HackrfError::from_i32(rv)?;
        let device_list = unsafe { ffi::hackrf_device_list() };
        let device_list = NonNull::new(device_list)
            .expect("hackrf_device_list returns null");
        Ok(DeviceList { device_list })
    }

    /// The number of available devices
    pub fn device_count(&self) -> usize {
        let ptr = self.device_list.as_ptr();
        unsafe { (*ptr).devicecount as usize }
    }

    /// Open the device under specific index
    pub fn open(&self, index: usize) -> Result<Hackrf, HackrfError> {
        let index: c_int = index.try_into().expect("index too large (doesn't fit c_int)");
        let mut device = core::ptr::null_mut();
        let rv = unsafe { ffi::hackrf_device_list_open(self.device_list.as_ptr(), index, &mut device) };
        HackrfError::from_i32(rv)?;
        let device = NonNull::new(device)
            .expect("hackrf_device_list_open didn't fill in the device");
        Ok(Hackrf::new(device))
    }

    /// Iterate over devices in the list
    pub fn iter<'a>(&'a self) -> impl Iterator<Item=DeviceEntry<'a>> {
        (0..self.device_count()).map(|index| unsafe {
            let device_list = &*self.device_list.as_ptr();
            let serial_number = device_list.serial_numbers.add(index).read();
            let serial_number = NonNull::new(serial_number).map(|sn| CStr::from_ptr(sn.as_ptr()));
            let board_id = device_list.usb_board_ids.add(index).read();
            let board_id = HackrfUSBBoardId::from_u32(board_id);
            let usb_device_index = device_list.usb_device_index.add(index).read();
            let usb_device = device_list.usb_devices.add(usb_device_index as usize).read();
            let usb_device = NonNull::new(usb_device);
            DeviceEntry { index, serial_number, board_id, usb_device }
        })
    }
}

impl Drop for DeviceList {
    fn drop(&mut self) {
        unsafe {
            ffi::hackrf_device_list_free(self.device_list.as_ptr());
        }
    }
}

// struct SweepRecord {}
type RxClosure = Box<dyn FnMut(&[u8]) -> ControlFlow<(), ()>>;
type TxClosure = Box<dyn FnMut(&mut[u8]) -> ControlFlow<usize, usize>>;
type FlushClosure = Box<dyn FnMut(c_int)>;
// type SweepClosure = Box<dyn FnMut(&[SweepRecord]) -> ControlFlow<(), ()>>;
enum HackrfState {
    Idle,
    Receiving { closure: *mut RxClosure },
    Transmitting { closure: *mut TxClosure, flush: *mut FlushClosure },
    // Sweeping { closure: *mut  },
}
impl HackrfState {
    /// Create a receiving closure
    fn receiving<F>(cb: F) -> (Self, *mut RxClosure)
        where F: FnMut(&[u8]) -> ControlFlow<(), ()> + Send + 'static
    {
        let closure;
        unsafe {
            let layout = std::alloc::Layout::new::<RxClosure>();
            closure = std::alloc::alloc(layout) as *mut RxClosure;
            closure.write(Box::new(cb));
        }
        (Self::Receiving { closure }, closure)
    }

    /// Create a transmitting+flush closure
    fn transmitting<F, E>(fill_cb: F, flush_cb: E) -> (Self, *mut TxClosure, *mut FlushClosure)
        where
            F: FnMut(&mut[u8]) -> ControlFlow<usize, usize> + Send + 'static,
            E: FnMut(c_int) + Send + 'static,
    {
        let closure;
        unsafe {
            let layout = std::alloc::Layout::new::<TxClosure>();
            closure = std::alloc::alloc(layout) as *mut TxClosure;
            closure.write(Box::new(fill_cb));
        }
        let flush;
        unsafe {
            let layout = std::alloc::Layout::new::<FlushClosure>();
            flush = std::alloc::alloc(layout) as *mut FlushClosure;
            flush.write(Box::new(flush_cb));
        }
        (Self::Transmitting { closure, flush }, closure, flush)
    }
}

impl Drop for HackrfState {
    fn drop(&mut self) {
        use HackrfState::*;
        match self {
            Idle => {},
            Receiving { closure } => unsafe {
                let layout = std::alloc::Layout::new::<RxClosure>();
                drop(closure.read());
                std::alloc::dealloc(*closure as *mut u8, layout);
            }
            Transmitting { closure, flush } => unsafe {
                let layout = std::alloc::Layout::new::<TxClosure>();
                drop(closure.read());
                std::alloc::dealloc(*closure as *mut u8, layout);

                let layout = std::alloc::Layout::new::<FlushClosure>();
                drop(flush.read());
                std::alloc::dealloc(*flush as *mut u8, layout);
            }
            // Sweeping { closure } => unsafe { drop(Box::from_raw(closure)); }
        }
    }
}

/// Owned handle for `hackrf_device`, including associated resources
pub struct Hackrf {
    device: NonNull<ffi::hackrf_device>,
    state: HackrfState,
}

extern "C" fn rx_callback(transfer: *mut ffi::hackrf_transfer) -> c_int {
    let transfer = unsafe { &*transfer };
    assert!(transfer.valid_length >= 0, "negative valid_length in hackrf_transfer");
    let slice = unsafe { core::slice::from_raw_parts(transfer.buffer, transfer.valid_length as usize) };
    let cb_ptr = transfer.rx_ctx as *mut RxClosure;
    let rv = unsafe { (*cb_ptr)(slice) };
    rv.is_break() as i32
}

extern "C" fn tx_callback(transfer: *mut ffi::hackrf_transfer) -> c_int {
    let transfer = unsafe { &mut *transfer };
    assert!(transfer.valid_length >= 0, "negative buffer_length in hackrf_transfer");
    let slice = unsafe { core::slice::from_raw_parts_mut(transfer.buffer, transfer.buffer_length as usize) };
    let cb_ptr = transfer.tx_ctx as *mut TxClosure;
    let rv = unsafe { (*cb_ptr)(slice) };
    transfer.valid_length = match rv {
        ControlFlow::Continue(filled)
        | ControlFlow::Break(filled) => filled as i32,
    };
    rv.is_break() as i32
}

extern "C" fn flush_callback(context: *mut c_void, success: c_int) {
    let cb_ptr = context as *mut FlushClosure;
    unsafe { (*cb_ptr)(success) };
}

impl Hackrf {
    fn new(device: NonNull<ffi::hackrf_device>) -> Self {
        Self { device, state: HackrfState::Idle }
    }

    /// Open first available HackRF device
    pub fn open_first() -> Result<Self, HackrfError> {
        let mut device = core::ptr::null_mut();
        let rv = unsafe { ffi::hackrf_open(&mut device) };
        HackrfError::from_i32(rv)?;
        let device = NonNull::new(device)
            .expect("hackrf_open didn't fill in the device");
        Ok(Hackrf::new(device))
    }

    /// Open HackRF device by serial number
    pub fn open_by_serial(serial: &str) -> Result<Self, HackrfError> {
        let mut device = core::ptr::null_mut();
        let rv = unsafe { ffi::hackrf_open_by_serial(serial.as_ptr() as *const i8, &mut device) };
        HackrfError::from_i32(rv)?;
        let device = NonNull::new(device)
            .expect("open_by_serial didn't fill in the device");
        Ok(Hackrf::new(device))
    }

    /// Read board part ID and serial number
    /// 
    /// Read MCU part id and serial number. See the documentation of the MCU for details!
    pub fn partid_serialno(&self) -> Result<ffi::read_partid_serialno_t, HackrfError> {
        let mut partid_serialno = unsafe { core::mem::zeroed::<ffi::read_partid_serialno_t>() };
        let rv = unsafe { ffi::hackrf_board_partid_serialno_read(self.device.as_ptr(), &mut partid_serialno) };
        HackrfError::from_i32(rv)?;
        Ok(partid_serialno)
    }

    /// Set hardware sync mode (hardware triggering)
    /// 
    /// Excerpt from "Hardware Triggering" section from HackrfOne documentation:
    ///
    /// HackRF One transmit and receive operations can be synchronized with another HackRF One
    /// or with other external equipment by using the trigger input and output on pin header P28.
    /// Triggering provides time synchronization with error of less than one sample period.
    pub fn set_hw_sync_mode(&mut self, enabled: bool) -> Result<(), HackrfError> {
        let rv = unsafe { ffi::hackrf_set_hw_sync_mode(self.device.as_ptr(), enabled as u8) };
        HackrfError::from_i32(rv)?;
        Ok(())
    }

    /// Read supported platforms bitmask of device
    pub fn supported_platform(&self) -> Result<SupportedPlatform, HackrfError> {
        let mut platform: u32 = 0;
        let rv = unsafe { ffi::hackrf_supported_platform_read(self.device.as_ptr(), &mut platform) };
        let rv = HackrfError::from_i32(rv)?;
        Ok(SupportedPlatform(rv as u32))
    }

    /// Reset HackRF device
    /// 
    /// Requires USB API version 0x0102 or above!
    pub fn reset(&mut self) -> Result<(), HackrfError> {
        let rv = unsafe { ffi::hackrf_reset(self.device.as_ptr()) };
        HackrfError::from_i32(rv)?;
        self.state = HackrfState::Idle;
        Ok(())
    }

    /// Enable / disable CLKOUT
    /// 
    /// Requires USB API version 0x0103 or above!
    pub fn set_clkout_enable(&mut self, enabled: bool) -> Result<(), HackrfError> {
        let rv = unsafe { ffi::hackrf_set_clkout_enable(self.device.as_ptr(), enabled as u8) };
        HackrfError::from_i32(rv)?;
        Ok(())
    }

    /// Query device streaming status
    ///
    /// This will indicate if there's a tx/rx/sweep in progress
    ///
    /// returns Ok(true) if the device is streaming, else one of
    /// `HackrfError::{StreamingThreadErr, StreamingStopped, StreamingExitCalled}`.
    pub fn is_streaming(&self) -> Result<bool, HackrfError> {
        let rv = unsafe { ffi::hackrf_is_streaming(self.device.as_ptr()) };
        HackrfError::from_i32(rv)?;
        Ok(rv != 0)
    }

    /// Turn on or off (override) the LEDs of the HackRF device
    /// 
    /// This function can turn on or off the LEDs of the device. There are 3 controllable LEDs
    /// on the HackRF one: USB, RX and TX. On the Rad1o, there are 4 LEDs.
    /// Each LEDcan be set individually, but the setting might get overridden by other functions.
    /// 
    /// The LEDs can be set via specifying them as bits of a 8 bit number @p state,
    /// bit 0 representing the first (USB on the HackRF One) and bit 3 or 4 representing the last LED.
    /// The upper 4 or 5 bits are unused. For example, binary value `0bxxxxx101` turns on the USB and TX LEDs on the HackRF One. 
    /// 
    /// Requires USB API version 0x0107 or above!
    pub fn set_leds(&mut self, leds: u8) -> Result<(), HackrfError> {
        let rv = unsafe { ffi::hackrf_set_leds(self.device.as_ptr(), leds) };
        HackrfError::from_i32(rv)?;
        Ok(())
    }

    fn apply_rx_options(&mut self, rx_options: options::RxOptions) -> Result<(), HackrfError> { unsafe {
        let device_ptr = self.device.as_ptr();
        let rv = match rx_options.center_freq {
            options::CenterFreq::Auto { freq_hz } => 
                ffi::hackrf_set_freq(device_ptr, freq_hz),
            options::CenterFreq::Explicit { if_freq_hz, lo_freq_hz, filter_path } =>
                ffi::hackrf_set_freq_explicit(device_ptr, if_freq_hz, lo_freq_hz, filter_path as u32),
        };
        HackrfError::from_i32(rv)?;
        let rv = match rx_options.sample_rate {
            options::SampleRate::Auto { freq_hz } =>
                ffi::hackrf_set_sample_rate(device_ptr, freq_hz),
            options::SampleRate::Explicit { freq_hz, divider } =>
                ffi::hackrf_set_sample_rate_manual(device_ptr, freq_hz, divider),
        };
        HackrfError::from_i32(rv)?;
        let bandwidth = ffi::hackrf_compute_baseband_filter_bw(rx_options.bandwidth);
        let rv = ffi::hackrf_set_baseband_filter_bandwidth(device_ptr, bandwidth);
        HackrfError::from_i32(rv)?;
        let rv = ffi::hackrf_set_amp_enable(device_ptr, rx_options.enable_amp as u8);
        HackrfError::from_i32(rv)?;
        let rv = ffi::hackrf_set_lna_gain(device_ptr, rx_options.lna_gain);
        HackrfError::from_i32(rv)?;
        let rv = ffi::hackrf_set_vga_gain(device_ptr, rx_options.vga_gain);
        HackrfError::from_i32(rv)?;
        let rv = ffi::hackrf_set_antenna_enable(device_ptr, rx_options.enable_bias_tee as u8);
        HackrfError::from_i32(rv)?;
        Ok(())
    } }

    pub fn start_rx<F>(&mut self, rx_options: options::RxOptions, cb: F) -> Result<(), HackrfError>
        where F: FnMut(&[u8]) -> ControlFlow<(), ()> + Send + 'static
    {
        self.apply_rx_options(rx_options)?;
        let device_ptr = self.device.as_ptr();
        let closure;
        (self.state, closure) = HackrfState::receiving(cb);
        let rv = unsafe { ffi::hackrf_start_rx(device_ptr, Some(rx_callback), closure as *mut c_void) };
        HackrfError::from_i32(rv)?;
        Ok(())
    }

    pub fn stop_rx(&mut self) -> Result<(), HackrfError> {
        let rv = unsafe { ffi::hackrf_stop_rx(self.device.as_ptr()) };
        HackrfError::from_i32(rv)?;
        self.state = HackrfState::Idle;
        Ok(())
    }

    fn apply_tx_options(&mut self, tx_options: options::TxOptions) -> Result<(), HackrfError> { unsafe {
        let device_ptr = self.device.as_ptr();
        let rv = match tx_options.center_freq {
            options::CenterFreq::Auto { freq_hz } => 
                ffi::hackrf_set_freq(device_ptr, freq_hz),
            options::CenterFreq::Explicit { if_freq_hz, lo_freq_hz, filter_path } =>
                ffi::hackrf_set_freq_explicit(device_ptr, if_freq_hz, lo_freq_hz, filter_path as u32),
        };
        HackrfError::from_i32(rv)?;
        let rv = match tx_options.sample_rate {
            options::SampleRate::Auto { freq_hz } =>
                ffi::hackrf_set_sample_rate(device_ptr, freq_hz),
            options::SampleRate::Explicit { freq_hz, divider } =>
                ffi::hackrf_set_sample_rate_manual(device_ptr, freq_hz, divider),
        };
        HackrfError::from_i32(rv)?;
        let bandwidth = ffi::hackrf_compute_baseband_filter_bw(tx_options.bandwidth);
        let rv = ffi::hackrf_set_baseband_filter_bandwidth(device_ptr, bandwidth);
        HackrfError::from_i32(rv)?;
        let rv = ffi::hackrf_set_amp_enable(device_ptr, tx_options.enable_amp as u8);
        HackrfError::from_i32(rv)?;
        let rv = ffi::hackrf_set_txvga_gain(device_ptr, tx_options.txvga_gain);
        HackrfError::from_i32(rv)?;
        let rv = ffi::hackrf_set_antenna_enable(device_ptr, tx_options.enable_bias_tee as u8);
        HackrfError::from_i32(rv)?;
        Ok(())
    } }

    pub fn start_tx<F, E>(&mut self, tx_options: options::TxOptions, fill_cb: F, flush_cb: E) -> Result<(), HackrfError>
        where
            F: FnMut(&mut[u8]) -> ControlFlow<usize, usize> + Send + 'static,
            E: FnMut(c_int) + Send + 'static,
    {
        self.apply_tx_options(tx_options)?;
        let device_ptr = self.device.as_ptr();
        let closure;
        let flush;
        (self.state, closure, flush) = HackrfState::transmitting(fill_cb, flush_cb);
        let rv = unsafe { ffi::hackrf_enable_tx_flush(device_ptr, Some(flush_callback), flush as *mut c_void) };
        HackrfError::from_i32(rv)?;
        let rv = unsafe { ffi::hackrf_start_tx(device_ptr, Some(tx_callback), closure as *mut c_void) };
        HackrfError::from_i32(rv)?;
        Ok(())
    }

    pub fn stop_tx(&mut self) -> Result<(), HackrfError> {
        let rv = unsafe { ffi::hackrf_stop_tx(self.device.as_ptr()) };
        HackrfError::from_i32(rv)?;
        self.state = HackrfState::Idle;
        Ok(())
    }
}

impl Drop for Hackrf {
    fn drop(&mut self) {
        unsafe {
            ffi::hackrf_close(self.device.as_ptr());
        }
    }
}

/// macro to make repetitive name implementations
macro_rules! impl_enum_name {
    ($enum:ident, $fn:ident) => {
        impl $enum {
            /// Return library-defined name for enum variant
            pub fn name(&self) -> &'static str {
                let bytes;
                unsafe {
                    let ptr = ffi::$fn(*self as _);
                    assert!(!ptr.is_null(), "{} returned null", stringify!($fn));
                    let cstr = core::ffi::CStr::from_ptr(ptr);
                    bytes = cstr.to_bytes();
                }
                core::str::from_utf8(bytes)
                    .expect("hackrf_*_name functions all return ASCII strings, this should never fail")
            }
        }

        impl core::fmt::Display for $enum {
            fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
                f.write_str(self.name())
            }
        }
    }
}

/// error enum, returned by many libhackrf functions
///
/// This enum avoids using non-error values (`success` & `true`), compared to the `-sys` crate
/// These values will be represented using `Result<_, HackrfError>`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(i32)]
pub enum HackrfError {
    /// The function was called with invalid parameters.
    InvalidParam = ffi::hackrf_error_HACKRF_ERROR_INVALID_PARAM,
    /// USB device not found, returned at opening.
    NotFound = ffi::hackrf_error_HACKRF_ERROR_NOT_FOUND,
    /// Resource is busy, possibly the device is already opened.
    Busy = ffi::hackrf_error_HACKRF_ERROR_BUSY,
    /// Memory allocation (on host side) failed
    NoMem = ffi::hackrf_error_HACKRF_ERROR_NO_MEM,
    /// LibUSB error, use @ref hackrf_error_name to get a human-readable error string (using `libusb_strerror`)
    LibUSB = ffi::hackrf_error_HACKRF_ERROR_LIBUSB,
    /// Error setting up transfer thread (pthread-related error)
    Thread = ffi::hackrf_error_HACKRF_ERROR_THREAD,
    /// Streaming thread could not start due to an error
    StreamingThreadErr = ffi::hackrf_error_HACKRF_ERROR_STREAMING_THREAD_ERR,
    /// Streaming thread stopped due to an error
    StreamingStopped = ffi::hackrf_error_HACKRF_ERROR_STREAMING_STOPPED,
    /// Streaming thread exited (normally)
    StreamingExitCalled = ffi::hackrf_error_HACKRF_ERROR_STREAMING_EXIT_CALLED,
    /// The installed firmware does not support this function
    UsbApiVersion = ffi::hackrf_error_HACKRF_ERROR_USB_API_VERSION,
    /// Can not exit library as one or more HackRFs still in use
    NotLastDevice = ffi::hackrf_error_HACKRF_ERROR_NOT_LAST_DEVICE,
    /// Unspecified error
    Other = ffi::hackrf_error_HACKRF_ERROR_OTHER,
}

impl core::error::Error for HackrfError {}

impl_enum_name!(HackrfError, hackrf_error_name);

impl HackrfError {
    pub fn from_i32(value: i32) -> Result<i32, Self> {
        use HackrfError::*;
        let rv = match value {
            // Non-negative values indicate success
            0.. => return Ok(value),
            ffi::hackrf_error_HACKRF_ERROR_INVALID_PARAM => InvalidParam,
            ffi::hackrf_error_HACKRF_ERROR_NOT_FOUND => NotFound,
            ffi::hackrf_error_HACKRF_ERROR_BUSY => Busy,
            ffi::hackrf_error_HACKRF_ERROR_NO_MEM => NoMem,
            ffi::hackrf_error_HACKRF_ERROR_LIBUSB => LibUSB,
            ffi::hackrf_error_HACKRF_ERROR_THREAD => Thread,
            ffi::hackrf_error_HACKRF_ERROR_STREAMING_THREAD_ERR => StreamingThreadErr,
            ffi::hackrf_error_HACKRF_ERROR_STREAMING_STOPPED => StreamingStopped,
            ffi::hackrf_error_HACKRF_ERROR_STREAMING_EXIT_CALLED => StreamingExitCalled,
            ffi::hackrf_error_HACKRF_ERROR_USB_API_VERSION => UsbApiVersion,
            ffi::hackrf_error_HACKRF_ERROR_NOT_LAST_DEVICE => NotLastDevice,
            ffi::hackrf_error_HACKRF_ERROR_OTHER | _ => Other,
        };
        Err(rv)
    }
}

/// HackRF board id
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum HackrfBoardId {
    /// Jellybean (pre-production revision, not supported)
    Jellybean = ffi::hackrf_board_id_BOARD_ID_JELLYBEAN as u8,
    /// Jawbreaker (beta platform, 10-6000MHz, no bias-tee)
    Jawbreaker = ffi::hackrf_board_id_BOARD_ID_JAWBREAKER as u8,
    /// HackRF One (prior to rev 9, same limits: 1-6000MHz, 20MSPS, bias-tee)
    Hackrf1OG = ffi::hackrf_board_id_BOARD_ID_HACKRF1_OG as u8,
    /// RAD1O (Chaos Computer Club special edition with LCD & other features. 50M-4000MHz, 20MSPS, no bias-tee)
    RAD1O = ffi::hackrf_board_id_BOARD_ID_RAD1O as u8,
    /// HackRF One (rev. 9 & later. 1-6000MHz, 20MSPS, bias-tee)
    Hackrf1R9 = ffi::hackrf_board_id_BOARD_ID_HACKRF1_R9 as u8,
    /// Unknown board (failed detection)
    Unrecognized = ffi::hackrf_board_id_BOARD_ID_UNRECOGNIZED as u8,
    /// Unknown board (detection not yet attempted, should be default value)
    Undetected = ffi::hackrf_board_id_BOARD_ID_UNDETECTED as u8,
}

impl_enum_name!(HackrfBoardId, hackrf_board_id_name);

/// HackRF board revision
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum HackrfBoardRev {
    /// Older than rev6
    Hackrf1Old = ffi::hackrf_board_rev_BOARD_REV_HACKRF1_OLD as u8,
    /// board revision 6, generic
    Hackrf1R6 = ffi::hackrf_board_rev_BOARD_REV_HACKRF1_R6 as u8,
    /// board revision 7, generic
    Hackrf1R7 = ffi::hackrf_board_rev_BOARD_REV_HACKRF1_R7 as u8,
    /// board revision 8, generic
    Hackrf1R8 = ffi::hackrf_board_rev_BOARD_REV_HACKRF1_R8 as u8,
    /// board revision 9, generic
    Hackrf1R9 = ffi::hackrf_board_rev_BOARD_REV_HACKRF1_R9 as u8,
    /// board revision 10, generic
    Hackrf1R10 = ffi::hackrf_board_rev_BOARD_REV_HACKRF1_R10 as u8,

    /// board revision 6, made by GSG
    GSGHackrf1R6 = ffi::hackrf_board_rev_BOARD_REV_GSG_HACKRF1_R6 as u8,
    /// board revision 7, made by GSG
    GSGHackrf1R7 = ffi::hackrf_board_rev_BOARD_REV_GSG_HACKRF1_R7 as u8,
    /// board revision 8, made by GSG
    GSGHackrf1R8 = ffi::hackrf_board_rev_BOARD_REV_GSG_HACKRF1_R8 as u8,
    /// board revision 9, made by GSG
    GSGHackrf1R9 = ffi::hackrf_board_rev_BOARD_REV_GSG_HACKRF1_R9 as u8,
    /// board revision 10, made by GSG
    GSGHackrf1R10 = ffi::hackrf_board_rev_BOARD_REV_GSG_HACKRF1_R10 as u8,

    /// unknown board revision (detection failed)
    Unrecognized = ffi::hackrf_board_rev_BOARD_REV_UNRECOGNIZED as u8,
    /// unknown board revision (detection not yet attempted)
    Undetected = ffi::hackrf_board_rev_BOARD_REV_UNDETECTED as u8,
}

impl_enum_name!(HackrfBoardRev, hackrf_board_rev_name);

/// USB board ID (product ID) enum
/// 
/// Contains USB-IF product id (field `idProduct` in `libusb_device_descriptor`). Can be used to identify general type of hardware.
/// Only used in @ref hackrf_device_list.usb_board_ids field of @ref hackrf_device_list, and can be converted into human-readable string via @ref hackrf_usb_board_id_name.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum HackrfUSBBoardId {
    /// Jawbreaker (beta platform) USB product id
    Jawbreaker = ffi::hackrf_usb_board_id_USB_BOARD_ID_JAWBREAKER as u16,
    /// HackRF One USB product id
    HackrfOne = ffi::hackrf_usb_board_id_USB_BOARD_ID_HACKRF_ONE as u16,
    /// RAD1O (custom version) USB product id
    RAD1O = ffi::hackrf_usb_board_id_USB_BOARD_ID_RAD1O as u16,
    /// Invalid / unknown USB product id
    Invalid = ffi::hackrf_usb_board_id_USB_BOARD_ID_INVALID as u16,
}

impl_enum_name!(HackrfUSBBoardId, hackrf_usb_board_id_name);
impl HackrfUSBBoardId {
    pub fn from_u32(value: u32) -> Self {
        match value {
            ffi::hackrf_usb_board_id_USB_BOARD_ID_JAWBREAKER => Self::Jawbreaker,
            ffi::hackrf_usb_board_id_USB_BOARD_ID_HACKRF_ONE => Self::HackrfOne,
            ffi::hackrf_usb_board_id_USB_BOARD_ID_RAD1O      => Self::RAD1O,
            _ => Self::Invalid,
        }
    }
}

/// RF filter path setting enum
/// 
/// Used only when performing explicit tuning using @ref hackrf_set_freq_explicit, or can be converted into a human readable string using @ref hackrf_filter_path_name.
/// This can select the image rejection filter (U3, U8 or none) to use - using switches U5, U6, U9 and U11. When no filter is selected, the mixer itself is bypassed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum RfPathFilter {
    /// No filter is selected, **the mixer is bypassed**, \f$f_{center} = f_{IF}\f$
    Bypass = ffi::rf_path_filter_RF_PATH_FILTER_BYPASS,
    /// LPF is selected, \f$f_{center} = f_{IF} - f_{LO}\f$
    LowPass = ffi::rf_path_filter_RF_PATH_FILTER_LOW_PASS,
    /// HPF is selected, \f$f_{center} = f_{IF} + f_{LO}\f$
    HighPass = ffi::rf_path_filter_RF_PATH_FILTER_HIGH_PASS,
}

impl_enum_name!(RfPathFilter, hackrf_filter_path_name);

/// Opera Cake secondary ports (A1-A4, B1-B4)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum OperacakePorts {
    PA1 = ffi::operacake_ports_OPERACAKE_PA1 as u8,
    PA2 = ffi::operacake_ports_OPERACAKE_PA2 as u8,
    PA3 = ffi::operacake_ports_OPERACAKE_PA3 as u8,
    PA4 = ffi::operacake_ports_OPERACAKE_PA4 as u8,
    PB1 = ffi::operacake_ports_OPERACAKE_PB1 as u8,
    PB2 = ffi::operacake_ports_OPERACAKE_PB2 as u8,
    PB3 = ffi::operacake_ports_OPERACAKE_PB3 as u8,
    PB4 = ffi::operacake_ports_OPERACAKE_PB4 as u8,
}

/// Opera Cake port switching mode. Set via @ref hackrf_set_operacake_mode and quaried via @ref hackrf_get_operacake_mode
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum OperacakeSwichingMode {
    /// Port connections are set manually using @ref hackrf_set_operacake_ports. Both ports can be specified, but not on the same side.
    Manual = ffi::operacake_switching_mode_OPERACAKE_MODE_MANUAL,
    /// Port connections are switched automatically when the frequency is changed. Frequency ranges can be set using @ref hackrf_set_operacake_freq_ranges. In this mode, B0 mirrors A0
    Frequency = ffi::operacake_switching_mode_OPERACAKE_MODE_FREQUENCY,
    /// Port connections are switched automatically over time. dwell times can be set with @ref hackrf_set_operacake_dwell_times. In this mode, B0 mirrors A0
    Time = ffi::operacake_switching_mode_OPERACAKE_MODE_TIME,
}

/// sweep mode enum
/// 
/// Used by @ref hackrf_init_sweep, to set sweep parameters.
/// 
/// Linear mode is no longer used by the hackrf_sweep command line tool and in general the interleaved method is always preferable, but the linear mode remains available for backward compatibility and might be useful in some special circumstances.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum SweepStyle {
    /// step_width is added to the current frequency at each step.
    Linear = ffi::sweep_style_LINEAR,
    /// each step is divided into two interleaved sub-steps, allowing the host to select the best portions of the FFT of each sub-step and discard the rest.
    Interleaved = ffi::sweep_style_INTERLEAVED,
}

pub struct SupportedPlatform(pub u32);

impl SupportedPlatform {
    /// JAWBREAKER platform bit
    pub const JAWBREAKER: u32 = ffi::HACKRF_PLATFORM_JAWBREAKER;
    /// HACKRF ONE (pre r9) platform bit
    pub const HACKRF1_OG: u32 = ffi::HACKRF_PLATFORM_HACKRF1_OG;
    /// RAD1O platform bit
    pub const RAD1O     : u32 = ffi::HACKRF_PLATFORM_RAD1O;
    /// HACKRF ONE (r9 or later) platform bit
    pub const HACKRF1_R9: u32 = ffi::HACKRF_PLATFORM_HACKRF1_R9;
}
