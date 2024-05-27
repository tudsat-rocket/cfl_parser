use serde_json::error::Category;
use shared_types::FlightMode;

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

// Taken from
// https://github.com/catsystems/cats-configurator/blob/main/src/modules/plots.js#L86
pub enum CATSFlightState {
  Calibrating = 1,
  Ready = 2,
  Thrust = 3,
  Coast = 4,
  Drogue = 5,
  Main = 6,
  Touchdown = 7
}

// Manual mapping from CATS Flight States to the ones used by SAM
impl From<CATSFlightState> for FlightMode {
  fn from(x: CATSFlightState) -> Self {
    match x {
      CATSFlightState::Calibrating => Self::Idle,
      CATSFlightState::Ready => Self::Armed,
      CATSFlightState::Thrust => Self::Burn,
      CATSFlightState::Coast => Self::Coast,
      CATSFlightState::Drogue => Self::RecoveryDrogue,
      CATSFlightState::Main => Self::RecoveryMain,
      CATSFlightState::Touchdown => Self::Landed
    }
  }
}

impl From<u8> for CATSFlightState {
  fn from(x: u8) -> Self {
    match x {
      1 => Self::Calibrating,
      2 => Self::Ready,
      3 => Self::Thrust,
      4 => Self::Coast,
      5 => Self::Drogue,
      6 => Self::Main,
      7 => Self::Touchdown,
      _ => Self::Calibrating // This seems to be the default state?
    }
  }
}