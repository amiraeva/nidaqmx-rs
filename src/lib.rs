mod ai_channel;
mod task_handle;
mod co_channel;
mod callback_utils;
mod ci_encoder_channel;

const EMPTY_CSTRING: *const i8 = b"\0".as_ptr() as *const i8;
const DAQ_CALLBACK_FREQ: usize = 100; // hz
const SAMPLE_TIMEOUT_SECS: f64 = 1.0;
const SCAN_WARNING: i32 = i32::max_value();

fn counter_generate_chan_desc(counter_id: u8) -> String {
	let desc = format!("Dev1/ctr{}", counter_id);
	desc
}

fn get_steady_time_nanoseconds() -> u64 {
	const TO_NS: u64 = 1e9 as u64;
	let time = time::get_time();
	time.sec as u64 * TO_NS + time.nsec as u64
}

pub use ai_channel::*;
pub use ci_encoder_channel::*;
