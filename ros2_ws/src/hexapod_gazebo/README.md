# hexapod_gazebo

The hexapod in Gazebo Harmonic, walking with the same gait as the ESP32.

```bash
ros2 launch hexapod_gazebo gazebo.launch.py              # Gazebo + RViz
ros2 launch hexapod_gazebo gazebo.launch.py slam:=true   # + live mapping
ros2 run hexapod_bringup console                         # type w, a, d, s, x, g, r, t ...
```

- Robot model: `hexapod_description` + `urdf/hexapod_gz.urdf.xacro` (servos, lidar, IMU, foot friction).
- Servos: `position_controller` takes 19 positions (legs 0-5 coxa/femur/tibia, then gripper).
- Odometry: Gazebo ground truth on `/odom` and `odom -> base_footprint`.
- World: `worlds/hexapod_world.sdf`, a 6 x 4 m room with walls, crates and a small red cube.

## Navigation (Nav2)

```bash
ros2 launch hexapod_gazebo gazebo.launch.py slam:=true      # 1. make a map, drive around
ros2 run nav2_map_server map_saver_cli -f ~/maps/sim_map --ros-args -p use_sim_time:=true
ros2 launch hexapod_gazebo nav.launch.py                    # 2. navigate on it
```

In RViz click **Nav2 Goal** and drag to set where and which way to face.

- Settings: Nav2's own defaults + `config/nav2_overrides.yaml`.
- Controller: Rotation Shim (turn in place to face the path) + Regulated Pure Pursuit (walk along it).
- `cmd_vel_bridge` turns Nav2's velocity into the gait's w / s / a / d / x.
