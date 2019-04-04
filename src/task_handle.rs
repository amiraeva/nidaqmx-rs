use super::{
	callback_utils::{
		self, raw_done_callback_impl, raw_read_callback_impl, CallbackWrapper, DoneCallback,
		RawDoneCallback,
	},
	EMPTY_CSTRING,
};

use std::{
	ffi::{CStr, CString},
	os::raw::c_void,
	ptr,
};

type NonNullVoidPtr = ptr::NonNull<c_void>;

type RawReadCallback = Option<unsafe extern "C" fn(*mut c_void, i32, u32, *mut c_void) -> i32>;
type ReadCallback<T> = fn(&mut T, &mut RawTaskHandle, u32) -> Result<(), ()>;

pub struct RawTaskHandle(NonNullVoidPtr);

impl RawTaskHandle {
	// Caller must ensure pointer is a valid task handle
	pub unsafe fn from_raw(ptr: *mut c_void) -> Option<Self> {
		NonNullVoidPtr::new(ptr).map(RawTaskHandle)
	}

	// Caller must ensure pointer is set to a valid task handle
	pub unsafe fn new() -> Self {
		RawTaskHandle(NonNullVoidPtr::dangling())
	}

	pub fn get(&self) -> NonNullVoidPtr {
		self.0
	}

	// Caller must ensure pointer is a valid task handle
	unsafe fn set(&mut self, ptr: NonNullVoidPtr) {
		self.0 = ptr;
	}

	pub fn chk_err_code(&mut self, err_code: i32) {
		let failed = err_code < 0;

		if failed {
			self.handle_err();
		}
	}

	// Caller must ensure
	pub unsafe fn clear_task(&mut self) {
		nidaqmx_sys::DAQmxStopTask(self.get().as_ptr());
		nidaqmx_sys::DAQmxClearTask(self.get().as_ptr());
	}

	fn handle_err(&mut self) {
		const ERROR_CODE_MAX_LEN: usize = 2048;

		let mut buf = [0u8; ERROR_CODE_MAX_LEN];
		let buf_ptr = buf.as_ptr() as *mut i8;

		// This is safe as long as nidaqmx respects the length of this buffer
		unsafe { nidaqmx_sys::DAQmxGetExtendedErrorInfo(buf_ptr, buf.len() as u32) };

		let last_elem = buf[..].last_mut().unwrap();
		*last_elem = b'\0'; // Write out null terminator

		// This is safe since we wrote a null terminator at the end of our buffer
		let err_msg = unsafe { CStr::from_bytes_with_nul_unchecked(&buf) };
		let err_msg = err_msg.to_string_lossy();

		// This is safe since we're immediately panicing
		unsafe { self.clear_task() };
		panic!("{}", err_msg);
	}
}

pub struct TaskHandle {
	raw_handle: RawTaskHandle,
}

impl TaskHandle {
	pub fn new() -> Self {
		// This is safe since we're immediately populating it with a valid handle
		let mut task_handle = TaskHandle {
			raw_handle: unsafe { RawTaskHandle::new() },
		};
		create_task(&mut task_handle);

		task_handle
	}

	pub fn get(&mut self) -> *mut c_void {
		self.raw_handle.get().as_ptr()
	}

	pub fn chk_err_code(&mut self, err_code: i32) {
		self.raw_handle.chk_err_code(err_code);
	}

	pub fn launch(&mut self) {
		unsafe { nidaqmx_sys::DAQmxStartTask(self.raw_handle.get().as_ptr()) };
	}

	pub fn create_ai_volt_chan(&mut self, chan_desc: &str, input_span: f64) {
		let chan_name = EMPTY_CSTRING;
		let chan_desc = CString::new(chan_desc).unwrap();

		// Should be safe if the task handle is valid
		let error_code = unsafe {
			nidaqmx_sys::DAQmxCreateAIVoltageChan(
				self.get(),
				chan_desc.as_ptr(),
				chan_name,
				nidaqmx_sys::DAQmx_Val_Cfg_Default,
				-input_span,
				input_span,
				nidaqmx_sys::DAQmx_Val_Volts as i32,
				ptr::null(),
			)
		};

		self.chk_err_code(error_code);
	}

	pub fn create_co_freq_chan(&mut self, chan_desc: &str, freq: f64, duty_cycle: f64) {
		const INITIAL_DELAY: f64 = 0.0;

		let name_of_channel = EMPTY_CSTRING;
		let chan_desc = CString::new(chan_desc).unwrap();

		let err_code = unsafe {
			nidaqmx_sys::DAQmxCreateCOPulseChanFreq(
				self.get(),
				chan_desc.as_ptr(),
				name_of_channel,
				nidaqmx_sys::DAQmx_Val_Hz as i32,
				nidaqmx_sys::DAQmx_Val_Low as i32,
				INITIAL_DELAY,
				freq,
				duty_cycle,
			)
		};

		self.chk_err_code(err_code);
	}

	pub fn configure_sample_clock(&mut self, clk_src: &str, rate: f64, samples_per_batch: u64) {
		let clk_src = CString::new(clk_src).unwrap();

		// Should be safe if the task handle is valid
		let err_code = unsafe {
			nidaqmx_sys::DAQmxCfgSampClkTiming(
				self.get(),
				clk_src.as_ptr(),
				rate,
				nidaqmx_sys::DAQmx_Val_Rising as i32,
				nidaqmx_sys::DAQmx_Val_ContSamps as i32,
				samples_per_batch,
			)
		};

		self.chk_err_code(err_code);
	}

	pub unsafe fn register_read_callback<T>(
		&mut self,
		n_samps: u32,
		callback: ReadCallback<T>,
		callback_data: T,
	) {
		const SAMPLES_EVENT_TYPE: i32 = nidaqmx_sys::DAQmx_Val_Acquired_Into_Buffer as i32;

		// This raw callback bootstraps the execution of the actual callback
		let raw_callback: RawReadCallback = Some(raw_read_callback_impl::<T>);

		// The actual user defined callback
		let callback_wrapper = CallbackWrapper::new(callback_data, callback);
		let callback_wrapper_ptr = callback_wrapper.into_raw();

		let err_code = nidaqmx_sys::DAQmxRegisterEveryNSamplesEvent(
			self.get(),
			SAMPLES_EVENT_TYPE,
			n_samps,
			callback_utils::CALLBACK_OPTIONS,
			raw_callback,
			callback_wrapper_ptr,
		);

		self.chk_err_code(err_code);
	}

	pub unsafe fn register_done_callback<T>(
		&mut self,
		callback: DoneCallback<T>,
		callback_data: T,
	) {
		// This raw callback bootstraps the execution of the actual callback
		let raw_callback: RawDoneCallback = Some(raw_done_callback_impl::<T>);

		// The actual user defined callback
		let callback_wrapper = CallbackWrapper::new(callback_data, callback);
		let callback_wrapper_ptr = callback_wrapper.into_raw();

		let err_code = nidaqmx_sys::DAQmxRegisterDoneEvent(
			self.get(),
			callback_utils::CALLBACK_OPTIONS,
			raw_callback,
			callback_wrapper_ptr,
		);

		self.chk_err_code(err_code);
	}
}

impl Drop for TaskHandle {
	// This is safe becuase the task handle is going out of scope, and will no longer be used
	fn drop(&mut self) {
		unsafe { self.raw_handle.clear_task() };
	}
}

unsafe impl Send for TaskHandle {}

fn create_task(th: &mut TaskHandle) {
	let task_name = EMPTY_CSTRING;

	let mut tmp_handle: *mut c_void = ptr::null_mut();
	let tmp_handle_ptr = &mut tmp_handle as *mut _;

	// This is safe as long as NI isn't dumb
	let error_code = unsafe { nidaqmx_sys::DAQmxCreateTask(task_name, tmp_handle_ptr) };

	assert!(
		!tmp_handle.is_null(),
		"DAQmxCreateTask seems to have returned a null pointer."
	);

	// Safe since tmp_handle should be a valid task handle
	unsafe {
		th.raw_handle.set(NonNullVoidPtr::new_unchecked(tmp_handle));
	}

	th.raw_handle.chk_err_code(error_code);
}
