# PX4 Autopilot — Project Guide for Claude

This is a development fork of [PX4 Autopilot](https://px4.io/), used primarily for **Gazebo (gz-harmonic) simulation**. The main use case is a quadrotor (x500) with a gimbal camera flying over a moving platform, where a second PX4 instance provides GPS telemetry for the platform.

## Key Simulation Entry Points

```bash
# Gimbal drone, default world
make px4_sitl gz_x500_gimbal

# Gimbal drone, baylands world (pretty scenery)
make px4_sitl gz_x500_gimbal_baylands

# Baylands world + moving platform + platform EKF drone (no gimbal)
PX4_GZ_WORLD=baylands PX4_GZ_PLATFORM_VEL=0.5 PX4_GZ_PLATFORM_HEADING_DEG=120 make px4_sitl gz_x500

# Baylands world + moving platform + gimbal drone (full setup)
PX4_GZ_WORLD=baylands PX4_GZ_PLATFORM_VEL=0.5 PX4_GZ_PLATFORM_HEADING_DEG=120 make px4_sitl gz_x500_gimbal

# Second PX4 instance — attaches to the platform_ekf model for GPS telemetry
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_NAME=platform_ekf ./build/px4_sitl_default/bin/px4 -i 2
```

**Simulation env vars:**
- `PX4_GZ_WORLD` — world SDF to load (default: `default`)
- `PX4_GZ_PLATFORM_VEL` — moving platform speed in m/s
- `PX4_GZ_PLATFORM_HEADING_DEG` — platform heading in degrees
- `PX4_GZ_MODEL_POSE` — spawn pose `x,y,z` for the main drone
- `PX4_GZ_MODEL_NAME` — attach to an existing Gazebo model instead of spawning
- `PX4_GZ_SIM_RENDER_ENGINE` — override Gazebo render engine (e.g., `ogre`)
- `HEADLESS=1` — skip launching Gazebo GUI

## Repo Structure

```
PX4-Autopilot/
├── ROMFS/px4fmu_common/
│   └── init.d-posix/
│       ├── rcS                    # Main SITL startup script
│       ├── px4-rc.simulator       # Dispatches to gz/jmavsim/etc.
│       ├── px4-rc.gzsim           # Gazebo startup, model spawn, gz_bridge start
│       └── airframes/             # One file per vehicle config (numbered)
│           ├── 4001_gz_x500       # Base x500 quad — all x500 variants source this
│           ├── 4019_gz_x500_gimbal # x500 + gimbal; sets MNT_* params
│           └── ...
│
├── Tools/simulation/gz/
│   ├── worlds/                    # Gazebo world SDF files
│   │   ├── default.sdf            # Flat ground plane
│   │   ├── baylands.sdf           # Real-world scenery + moving_platform + platform_ekf
│   │   └── ...
│   └── models/                    # Gazebo model SDF files
│       ├── x500/                  # Base quad (motor plugins only)
│       ├── x500_base/             # Geometry + sensors (IMU, GPS, baro, mag)
│       ├── x500_gimbal/           # x500 + gimbal (merge includes)
│       ├── gimbal/                # 3-axis gimbal with camera + JPC plugins
│       ├── moving_platform/       # Moving platform with libMovingPlatformController.so
│       └── ...
│
├── src/modules/simulation/
│   ├── gz_bridge/                 # C++ PX4↔Gazebo bridge (GZBridge, GZGimbal, etc.)
│   │   ├── GZBridge.cpp/hpp       # Main bridge: subscribes to all sensor topics
│   │   ├── GZGimbal.cpp/hpp       # Gimbal control: advertises command topics, reads MNT_* params
│   │   ├── GZMixingInterface*.cpp # ESC/servo/wheel output mixing
│   │   └── gz_env.sh.in           # Template for environment setup script
│   └── gz_plugins/                # Custom Gazebo system plugins (compiled to .so)
│       ├── moving_platform_controller/  # MovingPlatformController: reads PX4_GZ_PLATFORM_VEL/HEADING
│       └── optical_flow/
│
└── build/px4_sitl_default/
    ├── rootfs/                    # SITL working directory (px4 runs here)
    │   ├── gz_env.sh              # Sets GZ_SIM_RESOURCE_PATH, PX4_GZ_MODELS, etc.
    │   └── etc/init.d-posix/      # Built copies of ROMFS init scripts
    └── src/modules/simulation/
        └── gz_plugins/            # Compiled plugin .so files (libMovingPlatformController.so, etc.)
```

## How Simulation Startup Works

1. `make px4_sitl gz_<model>` runs the cmake target which invokes `px4` binary with `PX4_SIM_MODEL=gz_<model>`
2. `rcS` finds the matching airframe file by number (e.g., `4019_gz_x500_gimbal`) and sources it
3. `px4-rc.gzsim` is sourced via `px4-rc.simulator`:
   - Sources `gz_env.sh` to set resource paths
   - Starts `gz sim` server with the world SDF (background)
   - Waits for world to be ready (`gz service -i /world/.../scene/info`)
   - Spawns the drone model via `gz service /world/.../create`
   - Verifies the model appeared (polls `gz topic -l`)
   - Starts `gz_bridge` which subscribes to all sensor topics and bridges them to uORB

## Model Naming Convention

The cmake targets for Gazebo follow: `gz_<model>` or `gz_<model>_<world>`.

At runtime, the model instance name is: `<model>_<instance>` (e.g., `x500_gimbal_0` for instance 0).

The `MovingPlatformController` reads `PX4_SIM_MODEL` to determine the vehicle name it should wait for before starting to move the platform.

## baylands.sdf — Key Structure

The custom baylands world includes:
- Online Fuel models: baylands park, Coast Water
- `moving_platform` — flat 5×5m platform with `MovingPlatformController` plugin
- `platform_ekf` — an `x500` model **fixed** to the platform via a world joint (simulates a second GPS/PX4 mounted on the platform)

The main flying drone is **dynamically spawned** (not in the SDF), controlled by the first PX4 instance. The `platform_ekf` drone is controlled by a second PX4 instance (`-i 2`) that attaches via `PX4_GZ_MODEL_NAME=platform_ekf`.

## Known Issues / Notes

- **Spawn timeout**: Complex models (x500_gimbal) in busy worlds (baylands) previously timed out at 5000ms. Fixed by increasing to 15000ms + adding a model verification loop in `px4-rc.gzsim`.
- **libEGL warnings**: These come from the Gazebo server initializing rendering for camera sensors. They are warnings only and do not indicate a spawn failure.
- **Build copies**: `build/px4_sitl_default/etc/init.d-posix/` contains copies of the ROMFS init scripts. After editing source files in `ROMFS/`, either rebuild or manually copy to keep in sync.
- **MNT_* params**: Registered at compile time (in `src/modules/gimbal/gimbal_params.c`), always available regardless of module startup order.
