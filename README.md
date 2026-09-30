# Luna

_Lightweight Unified Nodal Architecture_

Luna is the avionics monorepo for the Yellow Jacket Space Program, holding all
avionics software, including firmware.

For details on Servo’s API, documentation exists in `servo/README.md`
and `servo/API.md`.

## Testing

Rust crates are covered by `cargo test`. Firmware is covered by SITL: the
binary runs inside an emulated board and its behaviour is asserted, with no
hardware attached.

```bash
./sitl/tests/sw_onboarding_board/run.sh
```

Peripheral and platform models live in `sitl/peripherals` and
`sitl/platforms`. Isolab, the system-level harness that runs the real
flight-computer and servo binaries in a NixOS VM, lives in `sitl/isolab`.

## Deprecated: AHRS

The standalone AHRS board and its firmware have been deprecated. The sensors
(IMU, magnetometer, barometer) are now integrated directly onto the flight
computer board. The last version with AHRS support can be found at:
https://github.com/gt-space/luna/tree/9481a5df4ceb58f5aac9113015def1d31ba01178

## Copyright

Copyright to the Luna source code is reserved by the Georgia Institute of
Technology and the Yellow Jacket Space Program. If you are interested in using
our code in your project, then please reach out and we can give you a license
for free.
