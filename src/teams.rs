#[derive(PartialEq)]
pub enum Alliance {
    Red,
    Blue,
}

#[derive(PartialEq)]
pub enum Side {
    Left,
    Right,
}

pub const ALLIANCE: Alliance = Alliance::Blue;
pub const SIDE: Side = Side::Left;
