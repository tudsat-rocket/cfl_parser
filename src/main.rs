// See https://github.com/catsystems/cats-configurator/blob/main/src/modules/logparser.js#L150

use std::process::exit;
use std::{env, io};
use std::io::prelude::*;
use std::fs::File;

mod datatypes;
use datatypes::*;
use shared_types::{GPSDatum, VehicleState};
use nalgebra::{UnitQuaternion, Vector3};

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

    // Initialize empty flight log
    let mut vehicle_states: Vec<VehicleState> = Vec::new();  

    // iterate over the rest of the file until its done
    //  -8 because some .cfl files have corrupt endings
    //  so this guarantees that at least a timestamp + type can be parsed
    // TODO: In theory the file can end with a valid TS/T Header but no data, how to handle that?
    while i < buffer.len() - 8 {
        let ts = read_u32_le(&buffer, i);
        let t = read_u32_le(&buffer, i + 4);
        i += 8;

        // Get sensor id and type field separately
        // unused? // let sensor_id = t & REC_ID_MASK;
        let t_without_id = t & !REC_ID_MASK;

        let record_type = RecordType::from(t_without_id);
        match record_type {
            RecordType::Imu => {
                // Read out acceleration
                let acc_x = read_u16_le(&buffer, i) as f32 / ACC_DIV;
                let acc_y = read_u16_le(&buffer, i + 2) as f32 / ACC_DIV;
                let acc_z = read_u16_le(&buffer, i + 4) as f32 / ACC_DIV;

                // Read out gyro
                let gyro_x = read_u16_le(&buffer, i + 6) as f32 / GYRO_DIV;
                let gyro_y = read_u16_le(&buffer, i + 8) as f32 / GYRO_DIV;
                let gyro_z = read_u16_le(&buffer, i + 10) as f32 / GYRO_DIV;
                
                // Increment index
                i += 12;

                // Push a new vehicle state into the list
                vehicle_states.push(VehicleState {
                    accelerometer1: Some(Vector3::new(acc_x, acc_y, acc_z)),
                    gyroscope: Some(Vector3::new(gyro_x, gyro_y, gyro_z)),
                    time: ts,
                    ..Default::default()
                });
                
            }
            RecordType::Baro => {
                // Read out pressure and temperature
                let pressure = read_u32_le(&buffer, i) as f32; // TODO: this cast is wrong, look into sensor output differences
                let temp = read_u32_le(&buffer, i + 4) as f32 / TEMP_DIV;

                // Increment index
                i += 8;

                // Push new vehicle state into the list
                vehicle_states.push(VehicleState {
                    pressure_baro: Some(pressure),
                    temperature_baro: Some(temp),
                    time: ts,
                    ..Default::default()
                });
            }
            RecordType::FlightInfo => {
                // Read out height / velocity / acceleration (again?)
                let height = read_f32_le(&buffer, i);
                let velocity = read_f32_le(&buffer, i + 4); 
                let acceleration = read_f32_le(&buffer, i + 8);

                // Increment index
                i += 12;

                // And push to vehicle states again
                //   these fields are very much guesswork
                vehicle_states.push(VehicleState {
                    altitude_asl: Some(height),
                    vertical_speed: Some(velocity),
                    vertical_accel: Some(acceleration),
                    time: ts,
                    ..Default::default()
                });
            }
            RecordType::OrientationInfo => {
                // Read out quaternion fields
                let q0_estimated = read_u16_le(&buffer, i) as f32 / Q_DIV;
                let q1_estimated = read_u16_le(&buffer, i + 2) as f32 / Q_DIV;
                let q2_estimated = read_u16_le(&buffer, i + 4) as f32 / Q_DIV;
                let q3_estimated = read_u16_le(&buffer, i + 6) as f32 / Q_DIV; // TODO: these are four values, don't quaternions only have three?!

                // Increment i again
                i += 8;

                // Push to vehicle states
                vehicle_states.push(VehicleState {
                    orientation: Some(UnitQuaternion::new(Vector3::new(q0_estimated, q1_estimated, q2_estimated))),
                    time: ts,
                    ..Default::default()
                });
            }
            RecordType::FilteredDataInfo => {
                // WHERE should these go?? 
                let filtered_altitude_agl = read_f32_le(&buffer, i);
                let filtered_acceleration = read_f32_le(&buffer, i + 4);
                
                // Da incrementy
                i += 8;

                // Push vehicleground
                vehicle_states.push(VehicleState {
                    vertical_accel_filtered: Some(filtered_acceleration),
                    altitude_asl: Some(filtered_altitude_agl),
                    time: ts,
                    ..Default::default()
                });
            }
            RecordType::FlightState => {
                // TODO: This needs more complex mapping
                let state =  read_u32_le(&buffer, i);

                // I-ncrement
                i += 4;

                // No push, print for now
                println!("Flight State: {:?}", state);
            }
            RecordType::EventInfo => {
                // Again, kinda CATS specific
                let event = read_u32_le(&buffer, i);
                let action = read_u16_le(&buffer, i + 4);
                let argument = read_u16_le(&buffer, i + 6);

                // Increment accordingly
                i += 8;

                // And print as well for now..
                println!("Event Info: {:?} / {:?} / {:?}", event, action, argument);
            }
            RecordType::ErrorInfo => {
                // Erreur                
                let error = read_u32_le(&buffer, i);
                
                // Inc. rement
                i += 4;
                
                // print again
                println!("Error: {:?}", error);
            }
            RecordType::GNSSInfo => {
                // Read out fields
                let latitude = read_f32_le(&buffer, i);
                let longitude = read_f32_le(&buffer, i + 4);
                let sattelites = read_u8_le(&buffer, i + 8);

                // Increment index
                i += 9;

                let gps_datum = GPSDatum {
                    latitude: Some(latitude),
                    longitude: Some(longitude),
                    num_satellites: sattelites,  
                    ..Default::default()
                };
                // Put it into vehicle states
                vehicle_states.push(VehicleState {
                    gps: Some(gps_datum),
                    time: ts,
                    ..Default::default()
                });
            }
            RecordType::VoltageInfo => {
                // Get the volts
                let voltage = read_u16_le(&buffer, i) as f32 / VOLT_DIV;
                
                // Add the bytes
                i += 2;

                // Push the state
                vehicle_states.push(VehicleState {
                    battery_voltage: Some(voltage as u16), // TODO: check this cast
                    time: ts,
                    ..Default::default()
                });
            }
            RecordType::UnknownError => {
                println!("Encountered unknown Record ID {:?} at index {:?}", t_without_id, i - 4);
                println!("Continuing with {:?}% of the provided data..", (i as f32 / buffer.len() as f32) * 100.0);
                break;
            }
        }
    }
    println!("Done parsing.");

    // TODO: serialize collected vehicle states in json output file
    println!("{:?}", vehicle_states.len());

    Ok(())
}
