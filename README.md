cd ~/dev/phd_assignment_ws

cat > README.md << 'EOF'

# LTU-RAI PhD Assignment — Ref. 3568-2026
Daniel Cantón Toro

## Environment
- Ubuntu 22.04 (ARM64), ROS 2 Humble, Gazebo Ignition Fortress
- Run inside a Parallels VM on Apple Silicon (M2 Pro)

## Reproduce

### 1. Launch Gazebo (once)
```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch clearpath_gz gz_sim.launch.py world:=warehouse
```

### 2. Spawn each robot separately
Wait for each robot's controllers to activate
(`ros2 control list_controllers -c /rX/controller_manager`) before spawning
the next one.

r0:
```bash
ros2 launch clearpath_gz robot_spawn.launch.py \
  setup_path:=clearpath/r0 \
  world:=warehouse x:=0.0 y:=0.0 z:=0.3 yaw:=0.0
```

r1:

```bash
ros2 launch clearpath_gz robot_spawn.launch.py \
  setup_path:=/home/user/dev/phd_assignment_ws/clearpath/r1 \
  x:=2.0 y:=0.0 z:=0.3 yaw:=0.0
```

### 3. Bring up Nav2 for each robot
Note: `use_namespace:=true` is required — without it, Nav2 silently falls
back to empty/default parameters for a namespaced robot.

r0:
```bash
ros2 launch nav2_bringup bringup_launch.py \
  namespace:=r0 use_namespace:=true use_sim_time:=true \
  map:=r0_warehouse_map.yaml \
  params_file:=nav2_config/nav2_params_r0.yaml \
  autostart:=true use_composition:=False
```

r1: same command with `namespace:=r1` and
`params_file:=nav2_config/nav2_params_r1.yaml`.

### 4. Publish an initial pose for each robot before navigating
```bash
ros2 topic pub -1 /r0/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

(same for r1, using its own spawn pose)

### 5. Run the mission planner

```bash
cd ~/dev/phd_assignment_ws
ros2 launch mission_planner_pkg mission_planner.launch.py
```

Robots, waypoints, and timing tolerances are configured in

`src/mission_planner_pkg/config/mission_config.yaml`

