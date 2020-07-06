/// An enumeration of possible errors with the device.
#[derive(Debug, Copy, Clone)]
pub enum Error {
    Invalid,
    NotPresent,
    Interface,
    Bounds,
}
