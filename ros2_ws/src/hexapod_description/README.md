# hexapod_description

URDF of the 18-DOF hexapod, built from the real SolidWorks assembly, with RViz and joint sliders.

## Build

```bash
cd ~/ros2_ws/src            # put this folder here
sudo apt install ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui
cd ~/ros2_ws && colcon build --packages-select hexapod_description
source install/setup.bash
```

## Run

```bash
ros2 launch hexapod_description display.launch.py                        # laptop: RViz + sliders
ros2 launch hexapod_description display.launch.py use_meshes:=false      # boxes only (faster)
ros2 launch hexapod_description display.launch.py gui:=false rviz:=false # Pi: TF only, for Nav2
```

## Digital twin (model moves with the real robot)

```bash
ros2 launch hexapod_bringup hexapod.launch.py        # Pi: bridge + robot_state_publisher
ros2 launch hexapod_description rviz.launch.py       # laptop: RViz only
```

The ESP32 reports every joint angle it commands; the bridge turns them into
`/joint_states`. Don't run `display.launch.py` at the same time (its sliders
would fight the real joints). It shows the commanded pose: the servos have
no position feedback.

## How it matches the firmware

- Joint value = firmware logical angle − 90°, in radians. 0 everywhere = the `h` stance.
- `+` coxa = toward front, `+` femur = foot up, `+` tibia = foot in (same on both sides).
- Leg numbers 0–5 and sides as in `hexapod_gripper.ino`.
- `gripper_joint`: 0 = open (150°), 0.436 = closed (100°).

## Measured from the CAD

| | |
|---|---|
| coxa (coxa axis → femur axis) | 70 mm |
| femur (femur axis → tibia axis) | 80 mm |
| tibia (tibia axis → foot tip) | 162 mm |
| leg directions at stance | ±48°, ±90°, ±132° |
| body height at stance | 162 mm (base_link above floor) |
| mass without battery | ≈ 2.0 kg |

Not in the CAD, placed by hand: the SG90 gripper, `imu_link` (MPU6050), the battery.

## Changing the CAD

Export the assembly again as STL (one file per part, "Do not translate STL output
data to positive space" ticked, millimetres), then:

```bash
pip install numpy scipy
python3 tools/stl_to_urdf.py <stl_folder>
```

This rewrites `meshes/` and `urdf/hexapod.urdf.xacro`. Don't hand-edit those.

## Frames

`base_footprint` → `base_link` → `legN_coxa_link` → `legN_femur_link` → `legN_tibia_link` → `legN_foot`,
plus `laser`, `imu_link`, `camera_link`, `camera_optical_frame`, `gripper_base`.

If another launch file already publishes a static `base_link → laser` transform,
remove it — this model now publishes it.
