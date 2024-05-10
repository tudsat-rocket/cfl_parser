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

pub struct ImuRecord {
  pub timestamp: u32,
  pub id: String,
  pub acc_x: f32,
  pub acc_y: f32,
  pub acc_z: f32,
  pub gyro_x: f32,
  pub gyro_y: f32,
  pub gyro_z: f32
}

pub struct BaroRecord {
  pub timestamp: u32,
  pub id: String,
  pub temp: f32,
  pub pressure: u32
}

pub struct FlightInfoRecord {
  pub timestamp: u32,
  pub height: f32,
  pub velocity: f32,
  pub acceleration: f32
}

pub struct OrientationInfoRecord {
  pub timestamp: u32,
  pub q0_estimated: f32,
  pub q1_estimated: f32,
  pub q2_estimated: f32,
  pub q3_estimated: f32
}

pub struct FilteredDataInfoRecord {
  pub timestamp: u32,
  pub filtered_altitude_agl: f32,
  pub filtered_acceleration: f32
}

pub struct FlightStateInfoRecord {
  pub timestamp: u32,
  pub state: u32
}

pub struct EventInfoRecord {
  pub timestamp: u32,
  pub event: u32, // TODO: do something to map these to event names
  pub action: u16,
  pub argument: u16
}

pub struct ErrorInfoRecord {
  pub timestamp: u32,
  pub error: u32
}

pub struct GNSSInfoRecord {
  pub timestamp: u32,
  pub latitude: f32,
  pub longitude: f32,
  pub sattelites: u8
}

pub struct VoltageInfoRecord {
  pub timestamp: u32,
  pub voltage: f32
}

pub struct FlightLog {
  pub imu_records: Vec<ImuRecord>,
  pub baro_records: Vec<BaroRecord>,
  pub flight_info_records: Vec<FlightInfoRecord>,
  pub orientation_info_records: Vec<OrientationInfoRecord>,
  pub filtered_data_info_records: Vec<FilteredDataInfoRecord>,
  pub gnss_info_records: Vec<GNSSInfoRecord>,
  pub flight_state_records: Vec<FlightStateInfoRecord>,
  pub event_info_records: Vec<EventInfoRecord>,
  pub error_info_records: Vec<ErrorInfoRecord>,
  pub voltage_info_records: Vec<VoltageInfoRecord>,
  pub first_timestamp: u32,
  pub last_timestamp: u32
}
impl FlightLog {
  pub fn offset_timestamps(&mut self, zero_timestamp: u32) -> () {
    // TODO: Is there a nicer way to do this??
    for record in self.imu_records.iter_mut() {
      record.timestamp = (record.timestamp - zero_timestamp) / 1000;
    }
    for record in self.baro_records.iter_mut() {
      record.timestamp = (record.timestamp - zero_timestamp) / 1000;
    }
    for record in self.flight_info_records.iter_mut() {
      record.timestamp = (record.timestamp - zero_timestamp) / 1000;
    }
    for record in self.orientation_info_records.iter_mut() {
      record.timestamp = (record.timestamp - zero_timestamp) / 1000;
    }
    for record in self.filtered_data_info_records.iter_mut() {
      record.timestamp = (record.timestamp - zero_timestamp) / 1000;
    }
    for record in self.gnss_info_records.iter_mut() {
      record.timestamp = (record.timestamp - zero_timestamp) / 1000;
    }
    for record in self.flight_state_records.iter_mut() {
      record.timestamp = (record.timestamp - zero_timestamp) / 1000;
    }
    for record in self.event_info_records.iter_mut() {
      record.timestamp = (record.timestamp - zero_timestamp) / 1000;
    }
    for record in self.error_info_records.iter_mut() {
      record.timestamp = (record.timestamp - zero_timestamp) / 1000;
    }
    for record in self.voltage_info_records.iter_mut() {
      record.timestamp = (record.timestamp - zero_timestamp) / 1000;
    }
  }
}

impl Default for FlightLog {
  fn default() -> Self {
      FlightLog {
        imu_records: Vec::new(),
        baro_records: Vec::new(),
        flight_info_records:Vec::new(),
        orientation_info_records: Vec::new(),
        filtered_data_info_records: Vec::new(),
        gnss_info_records: Vec::new(),
        flight_state_records: Vec::new(),
        event_info_records: Vec::new(),
        error_info_records: Vec::new(),
        voltage_info_records: Vec::new(),
        first_timestamp: 0,
        last_timestamp: 0
      }
  }
}