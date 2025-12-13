use bincode::{Decode, Encode};

#[derive(Decode, Encode, Debug)]
pub struct RecordedPath {
    pub frames: Vec<Frame>,
}

#[derive(Decode, Encode, Debug)]
pub struct Frame {
    pub left: f64,
    pub right: f64,
    pub intake: f64,
    pub outake: f64,
    pub piston: bool,

    pub delay: u64,
}
