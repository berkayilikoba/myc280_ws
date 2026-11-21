# myc280_ws

A ROS 2 Humble workspace for working with the Elephant Robotics **myCobot 280** arm mounted on an AgileX **LIMO** mobile base. The current focus is **self-collision checking and collision-free motion testing with MoveIt 2**.

The workspace is built on top of [HSGunes/limo_ws](https://github.com/HSGunes/limo_ws). 

---

## 1. Requirements

| Component | Version / Description |
| --- | --- |
| Operating System | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Simulator | Gazebo 11 (Classic) |
| Motion planning | MoveIt 2 (2.5.x) |
| Python | 3.10+ |

Install the required ROS packages:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-moveit ros-humble-moveit-setup-assistant ros-humble-moveit-configs-utils \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-rviz2 liburdfdom-tools \
  python3-colcon-common-extensions python3-rosdep
```

---

## 2. Installation and Build

1. Clone the repository:

```bash
cd ~
git clone https://github.com/berkayilikoba/myc280_ws.git
cd ~/myc280_ws
```

2. Build. Use a clean terminal that has **only** `/opt/ros/humble` sourced. Sourcing another workspace with the same package names (for example another `limo_ws` copy) causes conflicts.

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to limo_description limo_car mycobot_description
source install/setup.bash
```

3. Verify:

```bash
echo $AMENT_PREFIX_PATH        # should list only myc280_ws and /opt/ros/humble
ros2 pkg list | grep -E "limo_description|mycobot_description"
xacro $(ros2 pkg prefix --share limo_description)/urdf/limo_mycobot.xacro > /tmp/limo_mycobot.urdf
check_urdf /tmp/limo_mycobot.urdf
```

`check_urdf` should report `root Link: base_footprint` and the arm chain `joint1 -> ... -> joint6_flange`.

---

## 3. Robot Model

The combined model is `limo_description/urdf/limo_mycobot.xacro`. It includes the LIMO base and the arm from `mycobot_description`.

| Item | Value |
| --- | --- |
| Root link | `base_footprint` |
| Arm chain | `joint1` -> `joint6_flange` |
| Arm joints | `joint2_to_joint1`, `joint3_to_joint2`, `joint4_to_joint3`, `joint5_to_joint4`, `joint6_to_joint5`, `joint6output_to_joint6` |
| Wheel joints | `front_left_wheel`, `front_right_wheel`, `rear_left_wheel`, `rear_right_wheel` |
| Xacro arguments | `mode` (default `diff`), `kol_x` (default `-0.01`), `kol_z` (default `0.04`) |

Collision geometry is made of simple cylinders and boxes, which the MoveIt collision matrix is generated from.

---

## 4. Self-Collision Setup (MoveIt Setup Assistant)

The MoveIt configuration package `myc280_moveit_config` is generated with the MoveIt Setup Assistant.

```bash
source /opt/ros/humble/setup.bash
source ~/myc280_ws/install/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

Settings used:

1. **Load** `src/limo_description/urdf/limo_mycobot.xacro`.
2. **Self-Collisions:** sampling density `10000`, then *Generate Collision Matrix*.
3. **Virtual Joints:** `virtual_joint`, child `base_footprint`, parent `world`, type `fixed`.
4. **Planning Group:** `arm_group`, solver `kdl_kinematics_plugin/KDLKinematicsPlugin`, kinematic chain from `joint1` to `joint6_flange`.
5. **Robot Poses:** `home`, all joints at 0.
6. **Passive Joints:** the four wheel joints.
7. **ROS 2 Controllers:** `arm_group_controller` (`joint_trajectory_controller/JointTrajectoryController`, position interface).
8. **Output package:** `src/myc280_moveit_config`.

Build and open the demo:

```bash
cd ~/myc280_ws
colcon build --symlink-install --packages-select myc280_moveit_config
source install/setup.bash
ros2 launch myc280_moveit_config demo.launch.py
```

In RViz, open the MotionPlanning panel and move the arm with the interactive marker or the Joints tab. Links in collision turn red. If the arm never turns red, untick **Use Collision-Aware IK** in the Planning tab, which otherwise rejects colliding goals before they are shown.

---

## 5. Random Collision Test

`random_collision_test.py` samples random joint configurations and checks each one with the MoveIt `/check_state_validity` service:

- If the target pose or the straight joint-space path to it is in collision, an error with the colliding link pair is printed to the terminal.
- If it is collision-free, the arm moves to that pose in RViz.

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/myc280_ws/install/setup.bash
ros2 launch myc280_moveit_config demo.launch.py
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/myc280_ws/install/setup.bash
python3 ~/myc280_ws/random_collision_test.py
```

Parameters:

| Parameter | Default | Description |
| --- | --- | --- |
| `fake` | `true` | Node publishes `/joint_states` itself (no controller needed). Set `false` to send trajectories to `controller_topic`. |
| `controller_topic` | `/arm_group_controller/joint_trajectory` | Used when `fake:=false`. |
| `limit_ratio` | `0.9` | Fraction of each joint limit used for sampling. |
| `speed` | `0.5` | Motion speed in rad/s. |
| `hold_time` | `1.0` | Wait at each target in seconds. |
| `path_steps` | `15` | Number of interpolation points checked along the path. |

Example:

```bash
python3 ~/myc280_ws/random_collision_test.py --ros-args -p speed:=0.3 -p limit_ratio:=0.8
```

---

## 6. Project Structure

```text
myc280_ws/
├── src/
│   ├── limo_description/      # LIMO + myCobot URDF/xacro, launch, RViz configs
│   ├── mycobot_description/   # myCobot 280 URDF and meshes
│   ├── myc280_moveit_config/  # MoveIt config (generated with Setup Assistant)
│   ├── limo_car/              # LIMO simulation models and worlds
│   ├── limo_base/             # LIMO base driver
│   ├── limo_bringup/          # Bringup, navigation and mapping
│   ├── limo_autonomy/         # Autonomy and grasping nodes
│   ├── limo_kinematics/       # LIMO kinematics tools
│   ├── limo_localization/     # EKF and RTAB-Map localization
│   ├── limo_interfaces/       # Custom actions
│   └── limo_msgs/             # Custom messages
├── random_collision_test.py   # Random pose collision test
├── docs/ docker/ saha/        # Documentation, container files, field tools
└── README.md
```

---

## 7. Troubleshooting

| Problem | Fix |
| --- | --- |
| `AMENT_PREFIX_PATH` lists another workspace | Open a new terminal and source only `/opt/ros/humble` and `myc280_ws`. |
| RViz shows `No tf data` for Fixed Frame | Set Fixed Frame to `base_footprint` (or `world` with the virtual joint). Make sure `/joint_states` is being published. |
| RobotModel shows an error | Set *Description Topic -> Durability Policy* to `Transient Local`. |
| `/joint_states` has no publisher | The demo has no controllers. Run the test with `fake:=true`, or add a ROS 2 control hardware interface in the Setup Assistant. |
| No red links on collision | Untick *Use Collision-Aware IK*, check the collision matrix in `myc280_moveit_config/config/*.srdf`. |
| Gazebo hangs on exit | `pkill -f gzserver; pkill -f gzclient` |

---

## 8. Credits

- Base workspace: [HSGunes/limo_ws](https://github.com/HSGunes/limo_ws)
- Robot: [Elephant Robotics myCobot 280](https://www.elephantrobotics.com/en/mycobot-280-pi-en/)
