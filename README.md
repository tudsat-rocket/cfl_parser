# Parser for CATS CFL files

A parser for reading in [Euroc 2023 Team Data](https://github.com/catsystems/euroc23-team-data) for further analysis.
In its own repository for now, but may be integrated into sam in the future.

## Build

Run `cargo build`

## Usage

Run `cargo run [filename]` where *filename* points to a .cfl file. Will output *filename*.json.

## To Do

- fix quaternion orientation to point up and not sideways
- maybe add a GPSFixType to the GPSDatum
- Include `filtered_altitude_agl` field somewhere in the telemetry messages
