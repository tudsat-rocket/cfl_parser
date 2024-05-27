// See https://github.com/catsystems/cats-configurator/blob/main/src/modules/logparser.js#L150
// Also see https://github.com/catsystems/cats-embedded/blob/main/flight_computer/src/flash/recorder.hpp
use std::process::exit;
use std::{env, io};
use std::io::{prelude::*, BufWriter};
use std::fs::File;
use std::path::Path;

mod datatypes;
use datatypes::*;
use shared_types::{DownlinkMessage, GPSDatum, TelemetryDiagnostics, TelemetryGPS, TelemetryMain, TelemetryRawSensors, VehicleState};
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
    let mut downlink_messages: Vec<DownlinkMessage> = Vec::new();

    // Initialize mutable struct templates that can be copied with only the updated data
    let mut telemetry_main_template = TelemetryMain {
        ..Default::default()
    };
    let mut telemetry_raw_sensors_template = TelemetryRawSensors {
        ..Default::default()
    };
    let mut telemetry_diagnostic_template = TelemetryDiagnostics {
        ..Default::default()
    };
    let mut telemetry_gps_template = TelemetryGPS {
        ..Default::default()
    };
    // Whenever a new data point is found in the input file, adjust the template and push a copy

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

                // Update last TelemetryRawSensors
                telemetry_raw_sensors_template.time = ts;
                telemetry_raw_sensors_template.gyro = Vector3::new(gyro_x, gyro_y, gyro_z);
                telemetry_raw_sensors_template.accelerometer1 = Vector3::new(acc_x, acc_y, acc_z);
                
                // Push a copy to the list of downlink messages
                let msg = DownlinkMessage::TelemetryRawSensors(telemetry_raw_sensors_template.clone());
                downlink_messages.push(msg);
            }
            RecordType::Baro => {
                // Read out pressure and temperature
                let pressure = read_u32_le(&buffer, i) as f32; // TODO: this cast looks wrong, but the two sensors are very similar?
                let temp = read_u32_le(&buffer, i + 4) as f32 / TEMP_DIV;

                // Increment index
                i += 8;

                // Update TelemetryRawSensors with pressure value
                telemetry_raw_sensors_template.pressure_baro = pressure;
                telemetry_raw_sensors_template.time = ts;
                let sensor_msg = DownlinkMessage::TelemetryRawSensors(telemetry_raw_sensors_template.clone());
                downlink_messages.push(sensor_msg);
                

                // Update TelemetryDiagnostics with temperature
                telemetry_diagnostic_template.temperature_baro = temp as i8;
                telemetry_diagnostic_template.time = ts;
                let diagnostic_msg = DownlinkMessage::TelemetryDiagnostics(telemetry_diagnostic_template.clone());
                downlink_messages.push(diagnostic_msg);

                
            }
            RecordType::FlightInfo => {
                // Read out height / velocity / acceleration (again?)
                let height = read_f32_le(&buffer, i);
                let velocity = read_f32_le(&buffer, i + 4); 
                let acceleration = read_f32_le(&buffer, i + 8);

                // Increment index
                i += 12;

                // Add these values to the telemetrymain template
                telemetry_main_template.altitude = height;
                telemetry_main_template.vertical_speed = velocity;
                telemetry_main_template.vertical_accel = acceleration;
                telemetry_main_template.time = ts;

                // Push a copy to downlink messages
                let msg = DownlinkMessage::TelemetryMain(telemetry_main_template.clone());
                downlink_messages.push(msg);
            }
            RecordType::OrientationInfo => {
                // Read out quaternion fields
                let q0_estimated = read_u16_le(&buffer, i) as f32 / Q_DIV;
                let q1_estimated = read_u16_le(&buffer, i + 2) as f32 / Q_DIV;
                let q2_estimated = read_u16_le(&buffer, i + 4) as f32 / Q_DIV;
                let q3_estimated = read_u16_le(&buffer, i + 6) as f32 / Q_DIV; // TODO: these are four values, don't quaternions only have three?!

                // Increment i again
                i += 8;

                // Put value into Telemetry Main
                telemetry_main_template.orientation = Some(UnitQuaternion::new(Vector3::new(q0_estimated, q1_estimated, q2_estimated)));
                telemetry_main_template.time = ts;

                // Push to downlink messages
                let msg = DownlinkMessage::TelemetryMain(telemetry_main_template.clone());
                downlink_messages.push(msg);
            }
            RecordType::FilteredDataInfo => {
                // WHERE should these go?? 
                let filtered_altitude_agl = read_f32_le(&buffer, i);
                let filtered_acceleration = read_f32_le(&buffer, i + 4);
                
                // Da incrementy
                i += 8;

                // Update TelemetryMain template
                telemetry_main_template.altitude = filtered_altitude_agl;
                telemetry_main_template.vertical_accel_filtered = filtered_acceleration;
                telemetry_main_template.time = ts;

                // Push copy as always
                let msg = DownlinkMessage::TelemetryMain(telemetry_main_template.clone());
                downlink_messages.push(msg);
            }
            RecordType::FlightState => {
                let state: CATSFlightState = CATSFlightState::from(read_u32_le(&buffer, i) as u8);

                // I-ncrement
                i += 4;

                // Map state to FlightMode used by SAM
                telemetry_main_template.mode = state.into();
                telemetry_main_template.time = ts;

                // Push copy
                let msg = DownlinkMessage::TelemetryMain(telemetry_main_template.clone());
                downlink_messages.push(msg);
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

                println!("{:?}, {:?}", latitude, longitude);

                // Hacky way but I don´t want to re-implement the f32 <-> [u8; 3] conversion logic..
                let gps: GPSDatum = GPSDatum {
                    latitude: Some(latitude),
                    longitude: Some(longitude),
                    num_satellites: sattelites,
                    // TODO: could include a GPSFixType but which one would be best?
                    ..Default::default()
                };
                let temp_state: TelemetryGPS = VehicleState {
                    latitude: Some(latitude),
                    longitude: Some(longitude),
                    gps: Some(gps),
                    ..Default::default()
                }.into();
                
                telemetry_gps_template.fix_and_sats = sattelites;
                telemetry_gps_template.latitude = temp_state.latitude;
                telemetry_gps_template.longitude = temp_state.longitude;
                telemetry_gps_template.time = ts;

                println!("Lat: {:?}, Long: {:?}", temp_state.latitude, temp_state.longitude);

                // Copy and push
                let msg = DownlinkMessage::TelemetryGPS(telemetry_gps_template.clone());
                downlink_messages.push(msg);
            }
            RecordType::VoltageInfo => {
                // Get the volts
                let voltage = read_u16_le(&buffer, i) as f32 / VOLT_DIV;
                
                // Add the bytes
                i += 2;

                // Update the fields
                telemetry_diagnostic_template.battery_voltage = voltage as u16;
                telemetry_diagnostic_template.time = ts;

                // Push the state
                let msg = DownlinkMessage::TelemetryDiagnostics(telemetry_diagnostic_template.clone());
                downlink_messages.push(msg);
            }
            RecordType::UnknownError => {
                println!("Encountered unknown Record ID {:?} at index {:?}", t_without_id, i - 4);
                println!("Continuing with {:?}% of the provided data..", (i as f32 / buffer.len() as f32) * 100.0);
                break;
            }
        }
    }
    println!("Done parsing.");

    // Create output file name as input_without_ending.json
    let out_file_name = Path::new(&args[1]).with_extension("json");

    // Just dump it to a file
    let out_file = File::create(out_file_name)?;
    let mut writer = BufWriter::new(out_file);
    serde_json::to_writer(&mut writer, &downlink_messages).unwrap();
    writer.flush()?;

    println!("Output written successfully.");

    Ok(())
}
