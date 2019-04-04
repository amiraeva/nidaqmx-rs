use super::{
	co_channel::*,
	counter_generate_chan_desc, get_steady_time_nanoseconds,
	task_handle::{RawTaskHandle, TaskHandle},
	DAQ_CALLBACK_FREQ, EMPTY_CSTRING, SAMPLE_TIMEOUT_SECS, SCAN_WARNING,
};

use std::{ffi::CString, fmt, ptr};

use futures::{
	stream::Stream,
	sync::mpsc::{self, UnboundedReceiver, UnboundedSender},
	Poll,
};

const CLK_SRC_OUTPUT_PFI_ID: u8 = 13;
const CLK_SRC_COUNTER_ID: u8 = 1;
const ENCODER_COUNTER_ID: u8 = 0;

const DUTY_CYCLE: f64 = 0.5;

pub type EncoderTick = i32;
type RawScanData = Box<[EncoderTick]>;

pub struct EncoderReading {
	pub timestamp: u64,
	pub pos: EncoderTick,
}

impl EncoderReading {
	fn new(timestamp: u64, pos: EncoderTick) -> Self {
		Self { timestamp, pos }
	}
}

impl fmt::Display for EncoderReading {
	fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
		write!(f, "{},{}", self.timestamp, self.pos)
	}
}

struct BatchedScan {
	data: RawScanData,
	timestamp: u64,
}

impl BatchedScan {
	unsafe fn new_uninit(batch_size: usize) -> Self {
		use std::mem::uninitialized;

		let boxed: RawScanData = (0..batch_size).map(|_| uninitialized()).collect();

		Self {
			data: boxed,
			timestamp: 0,
		}
	}

	fn as_encoder_reading_iter<'a>(
		&'a self,
		sample_rate: usize,
	) -> impl Iterator<Item = EncoderReading> + 'a {
		const TO_NANOSEC: u64 = 1e9 as u64;

		let base_ts = self.timestamp;
		let data_len = self.data.len() as u32;

		let tstamp =
			(1..data_len).map(move |ind| base_ts - ind as u64 * TO_NANOSEC / sample_rate as u64);

		self.data
			.iter()
			.rev()
			.zip(tstamp)
			.map(|(pos, ts)| EncoderReading::new(ts, *pos))
			.rev()
	}
}

pub struct CiEncoderChannel {
	task_handle: TaskHandle,
	_co_channel: CoFreqChannel,
	sample_rate: usize,
	batch_size: usize,
}

impl CiEncoderChannel {
	pub fn new(sample_rate: usize) -> Self {
		let task_handle = TaskHandle::new();
		let _co_channel = CoFreqChannel::new(CLK_SRC_COUNTER_ID, sample_rate as f64, DUTY_CYCLE);
		let batch_size = sample_rate / DAQ_CALLBACK_FREQ;

		let mut ci_encoder_channel = CiEncoderChannel {
			task_handle,
			_co_channel,
			sample_rate,
			batch_size,
		};

		ci_encoder_channel.setup();

		ci_encoder_channel
	}

	pub fn make_async(mut self) -> AsyncEncoderChannel {
		let (snd, recv) = mpsc::unbounded();

		let internal = AsyncEncoderChanInternal {
			sender: snd,
			sample_rate: self.sample_rate,
		};

		unsafe {
			self.task_handle.register_read_callback(
				self.batch_size as u32,
				async_read_callback_impl,
				internal,
			);

			// Don't care about the done callback
			self.task_handle.register_done_callback(|_| (), ());
		}

		self.task_handle.launch();

		AsyncEncoderChannel {
			_encoder_chan: self,
			recv,
		}
	}

	fn setup(&mut self) {
		let internal_daqmx_buf_size = 10 * self.sample_rate as u64;

		self.create_channel(ENCODER_COUNTER_ID);

		let clk_src = generate_clock_src_desc(CLK_SRC_OUTPUT_PFI_ID);
		self.task_handle.configure_sample_clock(
			&clk_src,
			self.sample_rate as f64,
			internal_daqmx_buf_size,
		);
	}

	fn create_channel(&mut self, id: u8) {
		const USE_ENCODER_IDX: bool = true;
		const IDX_PULSE_POSITION: f64 = 0.0;
		const PULSE_PER_REV: u32 = 500;
		const INITIAL_POSITION: f64 = 0.0;

		let name_of_channel = EMPTY_CSTRING;
		let chan_desc = CString::new(counter_generate_chan_desc(id)).unwrap();

		let err_code = unsafe {
			nidaqmx_sys::DAQmxCreateCIAngEncoderChan(
				self.task_handle.get(),
				chan_desc.as_ptr(),
				name_of_channel,
				nidaqmx_sys::DAQmx_Val_X4 as i32,
				USE_ENCODER_IDX as u32,
				IDX_PULSE_POSITION,
				nidaqmx_sys::DAQmx_Val_ALowBLow as i32,
				nidaqmx_sys::DAQmx_Val_Ticks as i32,
				PULSE_PER_REV,
				INITIAL_POSITION,
				ptr::null_mut(),
			)
		};

		self.task_handle.chk_err_code(err_code);
	}
}

struct AsyncEncoderChanInternal {
	sender: UnboundedSender<EncoderReading>,
	sample_rate: usize,
}

pub struct AsyncEncoderChannel {
	_encoder_chan: CiEncoderChannel,
	recv: UnboundedReceiver<EncoderReading>,
}

impl Stream for AsyncEncoderChannel {
	type Item = EncoderReading;
	type Error = ();

	fn poll(&mut self) -> Poll<Option<Self::Item>, Self::Error> {
		self.recv.poll()
	}
}

fn generate_clock_src_desc(pfi_id: u8) -> String {
	let desc = format!("/Dev1/PFI{}", pfi_id);
	desc
}

unsafe fn read_digital_u32(
	task_handle: &mut RawTaskHandle,
	n_samps: u32,
) -> Result<BatchedScan, i32> {
	let mut samps_read = 0i32;
	let samps_read_ptr = &mut samps_read as *mut _;

	let mut scan = BatchedScan::new_uninit(n_samps as usize);
	scan.timestamp = get_steady_time_nanoseconds();

	let buf_len = scan.data.len();
	let buf_ptr = scan.data.as_mut_ptr() as *mut u32; // pretend the i32 is a u32

	let err_code = nidaqmx_sys::DAQmxReadCounterU32(
		task_handle.get().as_ptr(),
		n_samps as i32,
		SAMPLE_TIMEOUT_SECS,
		buf_ptr,
		buf_len as u32,
		samps_read_ptr,
		ptr::null_mut(),
	);

	match err_code {
		0 if samps_read == n_samps as i32 => Ok(scan),
		0 => Err(SCAN_WARNING),
		_ => Err(err_code),
	}
}

fn async_read_callback_impl(
	scan_chan: &mut AsyncEncoderChanInternal,
	task_handle: &mut RawTaskHandle,
	n_samps: u32,
) -> Result<(), ()> {
	let send_channel = &scan_chan.sender;
	let sample_rate = scan_chan.sample_rate;

	let batch = unsafe { read_digital_u32(task_handle, n_samps) }
		.map_err(|err_code| task_handle.chk_err_code(err_code))?;

	batch
		.as_encoder_reading_iter(sample_rate)
		.try_for_each(|enc| send_channel.unbounded_send(enc))
		.map_err(|_| ())?;

	Ok(())
}
