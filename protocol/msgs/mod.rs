pub mod sensor {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/src/protocol/msgs/asp_sensor.rs"
    ));
}
pub use sensor::*;

pub mod foxglove {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/src/protocol/msgs/foxglove.rs"
    ));
}
