# Bench results

Tested 2026-09-23 in ESM 303 on a SW Onboarding Board, STLINK-V3 on J3 (STDC14),
SW1 set to the STM32 side. Output is `printf` over semihosting, which the ST
extension prints in the "STM32Cube TCP Console" tab.

Every line is one report, printed once per second by the 200 Hz loop:

```
rate=<loop passes in the last second> overruns=<passes that missed the 5 ms deadline>
read_fail=<failed IMU reads> WHO_AM_I=<id byte> | Accel [mg] | Gyro [mdps]
```

## Flat on the table (checkpoint 2 gravity test)

```
rate=200 Hz overruns=0 read_fail=0 WHO_AM_I=0x6B | Accel [mg] x=-35 y=-43 z=1017 | Gyro [mdps] x=0   y=-490 z=70
rate=200 Hz overruns=0 read_fail=0 WHO_AM_I=0x6B | Accel [mg] x=-36 y=-44 z=1016 | Gyro [mdps] x=0   y=-560 z=0
rate=200 Hz overruns=0 read_fail=0 WHO_AM_I=0x6B | Accel [mg] x=-35 y=-44 z=1018 | Gyro [mdps] x=0   y=-490 z=70
```

z reads about +1 g, x and y are within a few tens of mg of zero. The tens-to-hundreds
of mdps on the gyro is the expected zero-rate bias of a still sensor.

## On its edge

```
rate=200 Hz overruns=0 read_fail=0 WHO_AM_I=0x6B | Accel [mg] x=110 y=973 z=22 | Gyro [mdps] x=-280 y=-1050 z=140
rate=200 Hz overruns=0 read_fail=0 WHO_AM_I=0x6B | Accel [mg] x=109 y=974 z=20 | Gyro [mdps] x=140  y=-1470 z=140
```

Gravity moves from z to y, and z drops to near zero.

## Upside down

```
rate=200 Hz overruns=0 read_fail=0 WHO_AM_I=0x6B | Accel [mg] x=-144 y=-39 z=-970 | Gyro [mdps] x=-70 y=-560 z=70
rate=200 Hz overruns=0 read_fail=0 WHO_AM_I=0x6B | Accel [mg] x=-143 y=-38 z=-969 | Gyro [mdps] x=0   y=-420 z=0
```

z reads about -1 g, so the sign is right.

## While being rotated by hand

```
rate=200 Hz overruns=0 read_fail=0 WHO_AM_I=0x6B | Accel [mg] x=5   y=889  z=-395 | Gyro [mdps] x=-11480 y=-9730   z=8960
rate=200 Hz overruns=0 read_fail=0 WHO_AM_I=0x6B | Accel [mg] x=111 y=-574 z=830  | Gyro [mdps] x=170730 y=-11410  z=-6160
```

The gyro jumps to tens or hundreds of dps while the board is moving, then settles
back to its bias as soon as it is still.

## Loop timing (checkpoint 3)

`rate` held at 200 Hz (occasionally 201) with `overruns=0` and `read_fail=0` for the
whole session. The scheduler starts each pass 5 ms after the previous scheduled
start, and restarts the schedule after the once-per-second report, since a
semihosting `printf` takes longer than one 5 ms period.

## Note: the IMU on this board is not the one in the BOM

The schematic and BOM list **LSM6DSMTR** (U3), whose WHO_AM_I is `0x6A`. This board
reports **`0x6B`**, which is the **LSM6DSR**. The scaling used here is unaffected:
both parts use 0.244 mg/LSB at +/-8 g and 70 mdps/LSB at 2000 dps, and the same
CTRL1_XL / CTRL2_G / CTRL3_C layout. Only the expected ID differs.

The Avionics onboarding page on Notion also says 0x6B, so the page matches the
hardware and the BOM is the odd one out.

## Screenshots

Live console during the session:

| File | What it shows |
|---|---|
| [docs/vscode-run-2.webp](docs/vscode-run-2.webp) | Steady state: rate=200 Hz, overruns=0, flat board |
| [docs/vscode-flat-then-edge.webp](docs/vscode-flat-then-edge.webp) | Flat, then tipped onto its edge (z to y) |
| [docs/console-edge-then-inverted.webp](docs/console-edge-then-inverted.webp) | Edge, then inverted (z goes negative), with motion spikes between |
| [docs/vscode-inverted-then-flat.webp](docs/vscode-inverted-then-flat.webp) | Inverted, then back to flat |
