#[derive(Debug)]
pub enum RecordType {
  Imu = 1 << 4,
  Baro = 1 << 5,
  FlightInfo = 1 << 6,
  OrientationInfo = 1 << 7,
  FilteredDataInfo = 1 << 8,
  FlightState = 1 << 9,
  EventInfo = 1 << 10,
  ErrorInfo = 1 << 11,
  GNSSInfo = 1 << 12,
  VoltageInfo = 1 << 13,
  UnknownError
}
impl From<u32> for RecordType {
  fn from(x: u32) -> Self {
      match x {
          0x10 => Self::Imu,
          0x20 => Self::Baro,
          0x40 => Self::FlightInfo,
          0x80 => Self::OrientationInfo,
          0x100 => Self::FilteredDataInfo,
          0x200 => Self::FlightState,
          0x400 => Self::EventInfo,
          0x800 => Self::ErrorInfo,
          0x1000 => Self::GNSSInfo,
          0x2000 => Self::VoltageInfo,
          _ => Self::UnknownError
      }
  }
}
