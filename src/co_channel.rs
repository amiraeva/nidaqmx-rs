use super::{task_handle::TaskHandle, counter_generate_chan_desc};

pub struct CoFreqChannel {
	task_handle: TaskHandle,
}

impl CoFreqChannel {
	pub fn new(counter_id: u8, freq: f64, duty_cycle: f64) -> Self {
		let task_handle = TaskHandle::new();
		let mut co_freq_channel = CoFreqChannel { task_handle };

		let chan_desc = counter_generate_chan_desc(counter_id);

		co_freq_channel.task_handle.create_co_freq_chan(&chan_desc, freq, duty_cycle);
		co_freq_channel.configure_timing();

		// We don't care about the done callback
		unsafe { co_freq_channel.task_handle.register_done_callback(|_| (), ()) };

		co_freq_channel.task_handle.launch();

		co_freq_channel
	}

	fn configure_timing(&mut self) {
		const BUF_SIZE: u64 = 0;

		let err_code = unsafe {
			nidaqmx_sys::DAQmxCfgImplicitTiming(
				self.task_handle.get(),
				nidaqmx_sys::DAQmx_Val_ContSamps as i32,
				BUF_SIZE,
			)
		};

		self.task_handle.chk_err_code(err_code);
	}
}
