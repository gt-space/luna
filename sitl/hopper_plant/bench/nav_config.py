#!/usr/bin/env python3
"""The navigation filter's config (shadow-nav JSON) for the plant that was built.

    nav_config.py <plant>/plant.json > nav.json
    nav_config.py <plant>/plant.json --lever      # the controller's reference point
                                                  # relative to the IMU, for --gnc-imu-lever

The filter is tuned to the plant's sensors: its measurement noise, GPS velocity
(whose noise is white in this plant), the Earth's field it uses, and the sensor
geometry. The filter's own defaults are for the receiver and IMU on the vehicle.

The filter's origin is the IMU: the GPS antenna and the LiDAR beams are placed
relative to the IMU, and the vehicle state the controller flies is the filter's
transformed to the controller's reference point (the centre of mass) by the lever
`--lever` gives. Hopper's geometry is from the base, in body axes with x up, and
its sensor models work from the centre of mass; the IMU's place is given from the
centre of mass (hopper's IMU lever, zero in its sensor models: the IMU is at the
CG).
"""
import json, math, os, sys

info = json.load(open(sys.argv[1]))
p = info["sensor_params"]
ant = p["gps"]["antenna"]

# The CG's height above the base on the pad, body x up (plant.json's cg_height;
# older plants do not carry it)
cg_height = float(info.get("cg_height", os.environ.get("CTV_CG_HEIGHT_M", "0.3498263441694766")))
# Where the IMU is, from the CG, body axes, m. Hopper's sensor models put it at the CG.
imu_from_cg = [float(v) for v in os.environ.get("CTV_IMU_FROM_CG", "0,0,0").split(",")]
# the base-referenced point `r` in body axes, relative to the IMU
def from_imu(r):
    return [r[0] - cg_height - imu_from_cg[0], r[1] - imu_from_cg[1], r[2] - imu_from_cg[2]]

if "--lever" in sys.argv:
    # the controller's reference point (the CG) relative to the IMU
    print(",".join(repr(-v) if v else "0.0" for v in imu_from_cg))
    sys.exit(0)

# The beams as hopper's sensor model lays them out: tilted out from the vehicle's
# down axis (body -x) by `cant`, at four azimuths about it, on a ring of `radius`
# at `mount_x` from the base.
lidar = p["lidar"]
cant = math.radians(lidar["cant"])
# the IMU's height above the ground on the pad: the filter's down coordinate is zero there
height_imu = cg_height + imu_from_cg[0]
lidar_mounts = []
for i, az in enumerate(math.radians(a) for a in lidar["azimuth"]):
    lidar_mounts.append({
        "id": i + 1,
        "beam_dir_body": [-math.cos(cant), math.sin(cant) * math.cos(az), math.sin(cant) * math.sin(az)],
        "position_body": from_imu([lidar["mount_x"], lidar["radius"] * math.cos(az), lidar["radius"] * math.sin(az)]),
        "height_imu": height_imu,
    })

# The IMU's sigmas are sample SDs at `noise_reference_period` (500 Hz); at another
# IMU period they scale by sqrt(reference / period). Older plants have no reference.
imu = p["imu"]
imu_scale = math.sqrt(imu.get("noise_reference_period", imu["period"]) / imu["period"])

config = {
    # per body axis, as hopper's own filter takes them (its Qc is these squared)
    "accel_std": [s * imu_scale for s in imu["accel_sigma"]],
    "gyro_std_dps": [math.degrees(s) * imu_scale for s in imu["gyro_sigma"]],
    "baro_std": p["baro"]["sigma"],
    "mag_std": p["mag"]["sigma"] / 100.0,
    "mag_field_ned": [b / 100.0 for b in p["mag"]["field_ned"]],
    "gps_position_std": p["gps"]["sigma"],
    "gps_velocity_std": p["gps"]["vel_sigma"],
    "use_gps_velocity": True,
    "gps_lever_arm": from_imu([ant[0][0], ant[1][0], ant[2][0]]),
    "use_lidar": True,
    "lidar_mounts": lidar_mounts,
    # the CTV hops to about 4.5 m; the filter's calibrated range ends at 4 m
    "lidar_max_range_m": 6.0,
    # the beams read about 10 cm on the ground, below the filter's calibrated 20 cm:
    # without them the vehicle at rest is measured by the barometer and GPS alone
    "lidar_min_range_m": lidar["r_min"],
    "status_every_s": 5.0,
}
# The bias random walks hopper's filter is given (the plant does not inject them;
# they only say how fast the filter lets its bias estimates move). Plants before
# the Qc change do not carry them, and the filter keeps its own defaults.
if "accel_bias_rw_psd" in imu:
    config["accel_bias_psd"] = imu["accel_bias_rw_psd"]
if "gyro_bias_rw_psd" in imu:
    config["gyro_bias_psd"] = imu["gyro_bias_rw_psd"]
if "bias_noise_psd" in p["baro"]:
    config["baro_bias_psd"] = p["baro"]["bias_noise_psd"]
print(json.dumps(config, indent=2))
