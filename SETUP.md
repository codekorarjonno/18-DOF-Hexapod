# Setup: from `git clone` to a running hexapod

Two computers run the software:

| Machine | OS | What runs there | Packages in this repo |
|---|---|---|---|
| **Laptop** (operator) | Ubuntu 24.04 | RViz, console (`hexcon`), Gazebo simulation, digital twin | `ros2_ws/src/` |
| **Raspberry Pi 5** (on the robot) | Ubuntu 24.04 Server (arm64) | lidar, SLAM, Nav2, camera, link to the ESP32 | `raspberry_pi/ros2_ws/src/` |

Both use **ROS 2 Jazzy**. The ESP32 firmware is flashed once from the Arduino IDE.

---

## 1. Laptop

```bash
cd ~
git clone https://github.com/codekorarjonno/18-DOF-Hexapod.git
bash 18-DOF-Hexapod/scripts/install_laptop.sh
```

The script:
1. installs ROS 2 Jazzy (desktop) and Gazebo Harmonic, if missing,
2. links the hexapod packages into `~/ros2_ws/src`,
3. downloads `sllidar_ros2` and `rf2o_laser_odometry` (listed in `hexapod.repos`),
4. installs every other dependency with `rosdep` and builds with `colcon`,
5. adds the shortcuts (`scripts/laptop_aliases.sh`) and `ROS_DOMAIN_ID=42` to `~/.bashrc`,
6. copies the saved maps to `~/maps`.

Open a **new terminal** afterwards. Try the simulation (no robot needed):

```bash
hexgz          # Gazebo + RViz, robot in a small room
hexgzcon       # second terminal: type w, a, d, s, x ...
```

## 2. Raspberry Pi 5

Install **Ubuntu 24.04 Server (64-bit)** with Raspberry Pi Imager, set the hostname to `hexapod`,
user `ariyan`, and your Wi-Fi. Then, over SSH (`ssh ariyan@hexapod.local`):

```bash
cd ~
git clone https://github.com/codekorarjonno/18-DOF-Hexapod.git
bash 18-DOF-Hexapod/scripts/install_pi.sh
```

The script does the same steps as on the laptop with the Pi packages, plus:
- downloads `camera_ros` and the Raspberry Pi fork of `libcamera` (building libcamera takes about 20–30 minutes),
- gives your user access to the serial port (ESP32 and lidar),
- installs the Pi shortcuts and `~/hexrun.sh` (camera stream).

Log out and back in, then:

```bash
hexmap         # lidar + odometry + SLAM
```

## 3. ESP32 firmware

1. Open `firmware/Hexapod_ESP32_v2.ino` in the Arduino IDE (ESP32 board package installed).
2. Install the libraries **Adafruit PWM Servo Driver** and **ESP32Servo** (Library Manager).
3. **Enter your own Wi-Fi name and password** near the top of the sketch.
4. Board: *ESP32 Dev Module*, then Upload.

Check it in the Serial Monitor (115200 baud): type `?` for the command list.

## 4. Everyday use

| Where | Command | What it does |
|---|---|---|
| Pi | `hexmap` | lidar + SLAM, build a map |
| Pi | `hexnav` / `hexnavmap` | Nav2 on a saved map / with a live map |
| Pi | `hexstream` | camera stream |
| Pi | `hexsave` | save the map to `~/maps/my_room` |
| Laptop | `hexcon` | console: drive the robot, type `go` to approve a Nav2 plan |
| Laptop | `hexmapview` | RViz with the map and the robot |
| Laptop | `hexgz`, `hexgznav`, `hexgzbignav` | Gazebo: small room, with Nav2, big building |
| Both | `hexkill` | stop everything |

The full list of shortcuts is in `scripts/laptop_aliases.sh` (laptop) and
`raspberry_pi/scripts/pi_bashrc.sh` (Pi).

## Things to set for your own setup

- **Wi-Fi** name and password in the firmware.
- **ROS_DOMAIN_ID**: 42 for the real robot (laptop and Pi must match), 43 for Gazebo.
- **Hostname**: the laptop reaches the Pi as `hexapod.local`.

## Updating later

```bash
cd ~/18-DOF-Hexapod && git pull
cd ~/ros2_ws && colcon build
```

The packages in `~/ros2_ws/src` are links to the repo, so a `git pull` is enough to get new code.
