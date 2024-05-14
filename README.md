# Parser for CATS CFL files

A parser for reading in [Euroc 2023 Team DAta](https://github.com/catsystems/euroc23-team-data) for further analysis.
In its own repository for now, but may be integrated into sam in the future.

## Build

Run `cargo build`

## Usage

Run `cargo run [filename]` where *filename* points to a .cfl file.

## To Do

- Output generated list of vehicle states as json file
- Some sensor value types mismatched, check these
- Maybe include the Flight State and Event Info records somehow?
