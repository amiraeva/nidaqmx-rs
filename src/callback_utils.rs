use super::task_handle::RawTaskHandle;

use std::os::raw::c_void;

pub const CALLBACK_OPTIONS: u32 = 0;

// type RawReadCallback = Option<unsafe extern "C" fn(*mut c_void, i32, u32, *mut c_void) -> i32>;

type ReadCallback<T> = fn(&mut T, &mut RawTaskHandle, u32) -> Result<(), ()>;
type ReadCallbackWrapper<T> = CallbackWrapper<T, ReadCallback<T>>;

pub struct CallbackWrapper<T, F> {
	pub data: T,
	pub func: F,
}

impl<T, F> CallbackWrapper<T, F> {
	pub fn new(data: T, func: F) -> Box<Self> {
		Box::new(CallbackWrapper { data, func })
	}

	pub fn into_raw(self: Box<Self>) -> *mut c_void {
		Box::into_raw(self) as *mut c_void
	}

	pub unsafe fn from_raw(ptr: *mut c_void) -> Box<Self> {
		Box::from_raw(ptr as *mut _)
	}
}

pub type DoneCallback<T> = fn(&mut T) -> ();
pub type DoneCallbackWrapper<T> = CallbackWrapper<T, DoneCallback<T>>;

pub type RawDoneCallback = Option<unsafe extern "C" fn(*mut c_void, i32, *mut c_void) -> i32>;

pub unsafe extern "C" fn raw_done_callback_impl<T>(
	task_handle: *mut c_void,
	err_code: i32,
	callback: *mut c_void,
) -> i32 {
	// Need to catch panics at ffi boundary
	let _ = std::panic::catch_unwind(|| {
		// Run user defined callback
		// Should be safe as long as no-one else touched that ptr
		let mut callback_wrapper = DoneCallbackWrapper::<T>::from_raw(callback);
		(callback_wrapper.func)(&mut callback_wrapper.data);

		// Check for errors if task handle is still alive
		let task_handle = RawTaskHandle::from_raw(task_handle);
		if let Some(mut task_handle) = task_handle {
			task_handle.chk_err_code(err_code);
		}
	});

	0
}

pub unsafe extern "C" fn raw_read_callback_impl<T>(
	task_handle: *mut c_void,
	_every_n_samples_event_type: i32,
	n_samps: u32,
	callback: *mut c_void,
) -> i32 {
	// Need to catch panics at ffi boundary
	let _ = std::panic::catch_unwind(|| {
		// We assume we were given a valid task handle
		let mut task_handle = RawTaskHandle::from_raw(task_handle).unwrap();

		// Run user defined callback
		// Should be safe as long as no one else touched the callback ptr
		let mut callback_wrapper = ReadCallbackWrapper::<T>::from_raw(callback);
		let callback_result =
			(callback_wrapper.func)(&mut callback_wrapper.data, &mut task_handle, n_samps);

		// Don't free the boxed callback if it succeeded
		if callback_result.is_ok() {
			Box::into_raw(callback_wrapper);
		} else {
			task_handle.clear_task();
			std::mem::drop(callback_wrapper);
		}
	});

	0
}
