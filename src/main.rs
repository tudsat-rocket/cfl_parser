// See https://github.com/catsystems/cats-configurator/blob/main/src/modules/logparser.js#L150

use std::process::exit;
use std::{env, io};
use std::io::prelude::*;
use std::fs::File;

mod datatypes;
use datatypes::*;

// Scaling factors for data points
const GYRO_DIV: f32 = 14.28;
const ACC_DIV: f32 = 1024.0 / 9.81;
const Q_DIV: f32 = 1000.0;
const TEMP_DIV: f32 = 100.0;
const VOLT_DIV: f32 = 1000.0;

const REC_ID_MASK: u32 = 0x0000000F;

fn read_u32_le(buffer: &Vec<u8>, index: usize) -> u32 {
    let buf_bytes = &buffer[index..index+4];
    return u32::from_le_bytes(buf_bytes.try_into().unwrap());
}

fn read_u16_le(buffer: &Vec<u8>, index: usize) -> u16 {
    let buf_bytes = &buffer[index..index+2];
    return u16::from_le_bytes(buf_bytes.try_into().unwrap());
}

fn read_u8_le(buffer: &Vec<u8>, index: usize) -> u8 {
    let buf_bytes = &buffer[index..index+1];
    return u8::from_le_bytes(buf_bytes.try_into().unwrap());
}

fn read_f32_le(buffer: &Vec<u8>, index: usize) -> f32 {
    let buf_bytes = &buffer[index..index+4];
    return f32::from_le_bytes(buf_bytes.try_into().unwrap());
}
fn main() -> io::Result<()> {
    let args: Vec<String> = env::args().collect();
    if args.len() != 2 {
        println!("Missing filename");
        exit(1)
    }
    let mut f = File::open(args[1].to_string())?;

    // Read file into buffer for further processing
    let mut buffer: Vec<u8> = Vec::new();
    f.read_to_end(&mut buffer)?;

    let mut i = 0;
    let mut version_bytes: Vec<u8> = Vec::new();
    
    let mut byte = buffer[i];
    while byte != 0x00{
        version_bytes.push(byte);
        i += 1;
        byte = buffer[i];
    }
    i += 1;
   
    // Parse version bytes as string
    let version_string: String = version_bytes.iter()
        .map(|&b| b as char)
        .collect();

    println!("Detected version {:?}", version_string);

    let mut first_timestamp: Option<u32> = None;
    let mut last_timestamp: u32 = 0;

    // Initialize empty flight log
    let mut flight_log: FlightLog = FlightLog::default();    

    // iterate over the rest of the file until its done
    //  -8 because some .cfl files have corrupt endings
    //  so this guarantees that at least a timestamp + type can be parsed
    // TODO: In theory the file can end with a valid TS/T Header but no data, how to handle that?
    while i < buffer.len() - 8 {
        let ts = read_u32_le(&buffer, i);
        let t = read_u32_le(&buffer, i + 4);
        i += 8;

        // Get sensor id and type field separately
        let sensor_id = t & REC_ID_MASK;
        let t_without_id = t & !REC_ID_MASK;

        // How does if let work
        if first_timestamp == None {
            first_timestamp = Some(ts);
        }

        let record_type = RecordType::from(t_without_id);
        match record_type {
            RecordType::Imu => {
                flight_log.imu_records.push(ImuRecord { 
                    timestamp: ts, 
                    id: format!("IMU{:?}", sensor_id), 
                    acc_x: read_u16_le(&buffer, i) as f32 / ACC_DIV,
                    acc_y: read_u16_le(&buffer, i + 2) as f32 / ACC_DIV,
                    acc_z: read_u16_le(&buffer, i + 4) as f32 / ACC_DIV, 
                    gyro_x: read_u16_le(&buffer, i + 6) as f32 / GYRO_DIV, 
                    gyro_y: read_u16_le(&buffer, i + 8) as f32 / GYRO_DIV, 
                    gyro_z: read_u16_le(&buffer, i + 10) as f32 / GYRO_DIV 
                });
                i += 12;
            }
            RecordType::Baro => {
                flight_log.baro_records.push(BaroRecord { 
                    timestamp: ts,
                    id: format!("BARO{:?}", sensor_id),
                    pressure: read_u32_le(&buffer, i),
                    temp: read_u32_le(&buffer, i + 4) as f32 / TEMP_DIV
                });
                i += 8;
            }
            RecordType::FlightInfo => {
                flight_log.flight_info_records.push(FlightInfoRecord {
                    timestamp: ts,
                    height: read_f32_le(&buffer, i),
                    velocity: read_f32_le(&buffer, i + 4), 
                    acceleration: read_f32_le(&buffer, i + 8) 
                });
                i += 12
            }
            RecordType::OrientationInfo => {
                flight_log.orientation_info_records.push(OrientationInfoRecord {
                    timestamp: ts,
                    q0_estimated: read_u16_le(&buffer, i) as f32 / Q_DIV,
                    q1_estimated: read_u16_le(&buffer, i + 2) as f32 / Q_DIV,
                    q2_estimated: read_u16_le(&buffer, i + 4) as f32 / Q_DIV,
                    q3_estimated: read_u16_le(&buffer, i + 6) as f32 / Q_DIV 
                });
                i += 8;
            }
            RecordType::FilteredDataInfo => {
                flight_log.filtered_data_info_records.push(FilteredDataInfoRecord { 
                    timestamp: ts, 
                    filtered_altitude_agl: read_f32_le(&buffer, i), 
                    filtered_acceleration: read_f32_le(&buffer, i + 4) 
                });
                i += 8;
            }
            RecordType::FlightState => {
                flight_log.flight_state_records.push(FlightStateInfoRecord { 
                    timestamp: ts,
                    state:  read_u32_le(&buffer, i)
                });
                i += 4;
            }
            RecordType::EventInfo => {
                flight_log.event_info_records.push(EventInfoRecord { 
                    timestamp: ts,
                    event: read_u32_le(&buffer, i), 
                    action: read_u16_le(&buffer, i + 4), 
                    argument: read_u16_le(&buffer, i + 6) 
                });
                i += 8;
            }
            RecordType::ErrorInfo => {
                flight_log.error_info_records.push(ErrorInfoRecord { 
                    timestamp: ts,
                    error: read_u32_le(&buffer, i) 
                });
                i += 4;
            }
            RecordType::GNSSInfo => {
                flight_log.gnss_info_records.push(GNSSInfoRecord { 
                    timestamp: ts,
                    latitude: read_f32_le(&buffer, i), 
                    longitude: read_f32_le(&buffer, i + 4), 
                    sattelites: read_u8_le(&buffer, i + 8) 
                });
                i += 9;
            }
            RecordType::VoltageInfo => {
                flight_log.voltage_info_records.push(VoltageInfoRecord { 
                    timestamp: ts,
                    voltage: read_u16_le(&buffer, i) as f32 / VOLT_DIV
                });
                i += 2;
            }
            RecordType::UnknownError => {
                println!("Encountered unknown Record ID {:?} at index {:?}", t_without_id, i - 4);
                println!("Continuing with {:?}% of the provided data..", (i as f32 / buffer.len() as f32) * 100.0);
                break;
            }
        }
        last_timestamp = ts;
    }
    println!("Done parsing.");

    // Find start of launch and offset timestamps
    let zero_timestamp = first_timestamp.unwrap_or(0);
    /*
    Weird transformation where t=0 is liftoff and everything before is negative time
    We're not gonna do that so we can keep using u32's :)
    for ev in flight_log.event_info_records.iter() {
        // Check if we have a liftoff event (code 2)
        if ev.event == 2 {
            zero_timestamp = ev.timestamp as u32;
            println!("Found liftoff point: {:?}", ev.timestamp);
            break;
        }
    }
    */ 

    // Adjust log's first&last timestamp
    flight_log.first_timestamp = zero_timestamp / 1000;
    flight_log.last_timestamp = last_timestamp / 1000;

    // Offset and scale all timestamps accordingly
    flight_log.offset_timestamps(zero_timestamp);

    println!("Done offsetting timestamps.");  
    
    // TODO: Actually do something with the data..

    Ok(())
}
