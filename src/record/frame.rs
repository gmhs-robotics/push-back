use std::{
    path::Path,
    time::{Duration, Instant},
};

use bincode::{
    Decode, Encode,
    config::{Configuration, LittleEndian, NoLimit, Varint},
};
use vexide::time::sleep_until;

#[derive(Encode, Decode, Default, Clone, Debug)]
pub struct TimedFrame<F: Frameable> {
    pub delta_time_micros: u64,
    pub frame: F,
}

#[derive(Encode, Decode, Default, Clone, Debug)]
pub struct Recording<F: Frameable> {
    pub frames: Vec<TimedFrame<F>>,
}

const BINCODE_CONFIG: Configuration<LittleEndian, Varint, NoLimit> = bincode::config::standard();

#[derive(Debug)]
pub enum RecordingError {
    Io(std::io::Error),
    Encode(bincode::error::EncodeError),
    Decode(bincode::error::DecodeError),
}

impl std::fmt::Display for RecordingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RecordingError::Io(e) => write!(f, "I/O error: {e}"),
            RecordingError::Encode(e) => write!(f, "bincode encode error: {e}"),
            RecordingError::Decode(e) => write!(f, "bincode decode error: {e}"),
        }
    }
}

impl From<std::io::Error> for RecordingError {
    fn from(e: std::io::Error) -> Self {
        RecordingError::Io(e)
    }
}
impl From<bincode::error::EncodeError> for RecordingError {
    fn from(e: bincode::error::EncodeError) -> Self {
        RecordingError::Encode(e)
    }
}
impl From<bincode::error::DecodeError> for RecordingError {
    fn from(e: bincode::error::DecodeError) -> Self {
        RecordingError::Decode(e)
    }
}

pub trait Recordable {
    type Frame: Frameable;
    const UPDATE_INTERVAL: Duration;

    async fn transform_to_frame(&mut self, frame: &Self::Frame);
    async fn get_new_frame(&self) -> Self::Frame;
}

pub trait Frameable = Encode + Decode<()> + Default + Clone;

impl<F: Frameable> Recording<F> {
    #[allow(dead_code)]
    pub fn push_timed(&mut self, delta: Duration, frame: F) {
        self.frames.push(TimedFrame {
            delta_time_micros: delta.as_micros() as u64,
            frame,
        });
    }

    #[allow(dead_code)]
    pub fn save<P: AsRef<Path>>(&self, path: P) -> Result<(), RecordingError> {
        let bytes = bincode::encode_to_vec(self, BINCODE_CONFIG)?;
        std::fs::write(path, bytes)?;
        Ok(())
    }

    #[allow(dead_code)]
    pub fn load<P: AsRef<Path>>(path: P) -> Result<Self, RecordingError> {
        let bytes = std::fs::read(path)?;
        let (recording, _) = bincode::decode_from_slice::<Self, _>(&bytes, BINCODE_CONFIG)?;
        Ok(recording)
    }

    #[allow(dead_code)]
    pub async fn playback<R: Recordable<Frame = F>>(self, robot: &mut R) {
        let mut deadline = Instant::now();

        for tf in self.frames {
            deadline += Duration::from_micros(tf.delta_time_micros);

            robot.transform_to_frame(&tf.frame).await;

            sleep_until(deadline).await;
        }
    }
}
