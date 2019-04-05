use super::{
	get_steady_time_nanoseconds,
	task_handle::{RawTaskHandle, TaskHandle},
	DAQ_CALLBACK_FREQ, SAMPLE_TIMEOUT_SECS, SCAN_WARNING,
};

use std::{fmt, ptr};

use futures::{
	stream::Stream,
	sync::mpsc::{self, UnboundedReceiver, UnboundedSender},
	Poll,
};

const NUM_CHANNELS: usize = 2;
const VOLTAGE_SPAN: f64 = 10.0;

type RawScanData = [f64; NUM_CHANNELS];

struct BatchedScan {
	data: Box<[RawScanData]>,
	timestamp: u64,
}

impl BatchedScan {
	unsafe fn new_uninit(batch_size: usize) -> Self {
		let boxed: Box<[RawScanData]> =
			(0..batch_size).map(|_| std::mem::uninitialized()).collect();

		Self {
			data: boxed,
			timestamp: 0,
		}
	}

	fn as_scan_iter<'a>(&'a self, sample_rate: usize) -> impl Iterator<Item = ScanData> + 'a {
		const TO_NANOSEC: u64 = 1e9 as u64;

		let base_ts = self.timestamp;
		let data_len = self.data.len() as u32;

		let tstamp =
			(1..data_len).map(move |ind| base_ts - ind as u64 * TO_NANOSEC / sample_rate as u64);

		self.data
			.iter()
			.rev()
			.zip(tstamp)
			.map(|(data, ts)| ScanData::new(*data, ts))
			.rev()
	}
}

#[derive(Debug)]
pub struct ScanData {
	pub data: RawScanData,
	pub timestamp: u64,
}

impl ScanData {
	fn new(data: [f64; NUM_CHANNELS], timestamp: u64) -> Self {
		ScanData { data, timestamp }
	}
}

impl fmt::Display for ScanData {
	fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
		write!(f, "{},{},{}", self.timestamp, self.data[0], self.data[1])
	}
}

pub struct AiChannel {
	task_handle: TaskHandle,
	sample_rate: usize,
	batch_size: usize,
}

impl AiChannel {
	pub fn new<S1: AsRef<str>, S2: AsRef<str>>(clk_src: S1, dev: S2, sample_rate: usize) -> Self {
		let task_handle = TaskHandle::new();

		let mut ai_channel = AiChannel {
			task_handle,
			sample_rate,
			batch_size: sample_rate / DAQ_CALLBACK_FREQ,
		};

		ai_channel.init(clk_src.as_ref(), dev.as_ref());

		ai_channel
	}

	fn init(&mut self, clk_src: &str, dev: &str) {
		let internal_buf_size = 10 * self.sample_rate as u64;

		self.task_handle
			.create_ai_volt_chan(dev, VOLTAGE_SPAN);

		self.task_handle.configure_sample_clock(
			clk_src,
			self.sample_rate as f64,
			internal_buf_size,
		);
	}

	pub fn make_async(mut self) -> AsyncAiChannel {
		let (snd, recv) = mpsc::unbounded();

		let internal = AsyncAiChanInternal {
			sender: snd,
			sample_rate: self.sample_rate,
		};

		unsafe {
			self.task_handle.register_read_callback(
				self.batch_size as u32,
				async_read_callback_impl,
				internal,
			);
			// We dont care about the done callback
			self.task_handle.register_done_callback(|_| (), ());
		}

		self.task_handle.launch();

		AsyncAiChannel {
			_ai_chan: self,
			recv,
		}
	}
}

struct AsyncAiChanInternal {
	sender: UnboundedSender<ScanData>,
	sample_rate: usize,
}

pub struct AsyncAiChannel {
	_ai_chan: AiChannel,
	recv: UnboundedReceiver<ScanData>,
}

impl Stream for AsyncAiChannel {
	type Item = ScanData;
	type Error = ();

	#[inline]
	fn poll(&mut self) -> Poll<Option<Self::Item>, Self::Error> {
		self.recv.poll()
	}
}

unsafe fn read_analog_f64(
	task_handle: &mut RawTaskHandle,
	n_samps: u32,
) -> Result<BatchedScan, i32> {
	let mut samps_read = 0i32;
	let samps_read_ptr = &mut samps_read as *mut _;

	let mut scan = BatchedScan::new_uninit(n_samps as usize);
	scan.timestamp = get_steady_time_nanoseconds();

	let buf_len = (scan.data.len() * NUM_CHANNELS) as u32;
	let buf_ptr = scan.data.as_mut_ptr() as *mut _ as *mut f64;

	let err_code = nidaqmx_sys::DAQmxReadAnalogF64(
		task_handle.get().as_ptr(),
		n_samps as i32,
		SAMPLE_TIMEOUT_SECS,
		nidaqmx_sys::DAQmx_Val_GroupByScanNumber,
		buf_ptr,
		buf_len,
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
	scan_chan: &mut AsyncAiChanInternal,
	task_handle: &mut RawTaskHandle,
	n_samps: u32,
) -> Result<(), ()> {
	let send_channel = &scan_chan.sender;
	let sample_rate = scan_chan.sample_rate;

	let batch = unsafe { read_analog_f64(task_handle, n_samps) }
		.map_err(|err_code| task_handle.chk_err_code(err_code))?;

	batch
		.as_scan_iter(sample_rate)
		.try_for_each(|scan| send_channel.unbounded_send(scan))
		.map_err(|_| ())?;

	Ok(())
}
