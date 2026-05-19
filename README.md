# TurtleBot3 Tour Workspace

This workspace contains a TurtleBot3 simulation/navigation setup plus a small tour system. The tour system can save waypoints, retrieve saved tours, follow all saved waypoints, follow an optimized subset of waypoints, react to speech intent messages, and navigate to saved dock poses.

## Packages You Will Use Most

- `tour_manager`: launch files, tour database service, and waypoint saver.
- `robot_tour`: Nav2 waypoint clients and the `TalkAtWaypoint` Nav2 waypoint task plugin.
- `speech_locomotion_interface`: turns JSON speech intent messages into tour, TSP, dock, and stop commands.
- `docking`: listens for dock commands and sends the robot to the nearest saved dock pose.
- `social_robot_interfaces`: custom tour message and service types.
- `turtlebot3_*`: TurtleBot3 simulation, navigation, bringup, description, and message packages.

## Build From A Fresh Terminal

Open a terminal in the workspace root:

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
```

Install missing ROS package dependencies:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
colcon build --symlink-install
```

Source the newly built workspace:

```bash
source install/setup.bash
```

You need to source both ROS and this workspace in every new terminal before running commands:

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
```

## Run The Locomotion Test Launch

The launch file in this workspace is named `locomotion_test.launch.py`.

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tour_manager locomotion_test.launch.py
```

This starts:

- Gazebo TurtleBot3 world from `turtlebot3_gazebo`.
- Nav2 from `turtlebot3_navigation2`.
- Tour manager nodes from `tour_manager_launch.py`.

Useful launch arguments:

```bash
ros2 launch tour_manager locomotion_test.launch.py model:=waffle_pi use_sim_time:=true
```

If you are looking for `test_locomotion.launch.py`, that file does not currently exist. Use `locomotion_test.launch.py`.

## Custom Messages And Services

`social_robot_interfaces/msg/TspCommand`:

```text
int64[] waypoints
```

`social_robot_interfaces/srv/Tours`:

```text
int64 idx
---
geometry_msgs/PoseStamped[] tour
```

## Main Executables And Interfaces

| Package | Executable | Node name | What it does |
| --- | --- | --- | --- |
| `tour_manager` | `tour_manager` | `tour_manager` | Stores and retrieves saved tour waypoints in `tours.db`. |
| `tour_manager` | `tour_saver` | `tour_saver` | Saves the current robot pose as a tour waypoint. |
| `robot_tour` | `tour_guide_start` | `waypoint_action_client` | Sends the full saved tour to Nav2 waypoint following. |
| `robot_tour` | `subtour_start` | `subtour` in launch, internally `subtour_action_node` | Optimizes a requested subset of tour waypoints and sends them to Nav2. |
| `docking` | `dock_listener` | `dock_listener` | Navigates to the nearest dock pose stored in `docks.db`. |
| `speech_locomotion_interface` | `listening` | `speech_listener` | Converts speech intent JSON into tour, TSP, dock, and stop actions. |

### `tour_manager`

Publishes: none.

Subscribes:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/save_tour` | `geometry_msgs/msg/PoseStamped` | Pose to append to `tours.db`. |

Services:

| Service | Type | Meaning |
| --- | --- | --- |
| `/tour_retrieve` | `social_robot_interfaces/srv/Tours` | Returns saved tour poses. The current code reads all saved poses regardless of `idx`. |

Run by itself:

```bash
ros2 run tour_manager tour_manager
```

### `tour_saver`

Publishes:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/save_tour` | `geometry_msgs/msg/PoseStamped` | Current `map -> base_link` pose to save. |

Subscribes:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/save_tour_command` | `std_msgs/msg/String` | Any received string triggers a save attempt. |

TF expected:

| Transform | Meaning |
| --- | --- |
| `map -> base_link` | Used to find the robot pose to save. |

Example:

```bash
ros2 topic pub --once /save_tour_command std_msgs/msg/String "{data: save}"
```

### `tour_guide_start`

Publishes: none directly.

Subscribes:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/tour_command` | `std_msgs/msg/String` | Any received command requests the saved tour and starts waypoint following. |

Clients:

| Interface | Type | Meaning |
| --- | --- | --- |
| `/tour_retrieve` service | `social_robot_interfaces/srv/Tours` | Gets the saved tour. |
| `/follow_waypoints` action | `nav2_msgs/action/FollowWaypoints` | Sends the tour to Nav2. |

Example:

```bash
ros2 topic pub --once /tour_command std_msgs/msg/String "{data: start}"
```

### `subtour_start`

Parameters from `src/tour_manager/config/tour_manager_params.yaml`:

| Parameter | Default in launch config | Meaning |
| --- | --- | --- |
| `command_topic` | `/tsp_command` | Topic to receive waypoint index lists. |
| `action_name` | `/follow_waypoints` | Nav2 action name. |
| `current_pose_topic` | `/amcl_pose` | Current localized pose topic. |
| `max_2opt_iterations` | `10000` | Limit for 2-opt tour improvement. |

Subscribes:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/tsp_command` | `social_robot_interfaces/msg/TspCommand` | Waypoint indices to visit, for example `[0, 3, 1]`. |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Current pose used to choose the closest starting waypoint. |

Clients:

| Interface | Type | Meaning |
| --- | --- | --- |
| `/tour_retrieve` service | `social_robot_interfaces/srv/Tours` | Gets all saved tour poses. |
| `/follow_waypoints` action | `nav2_msgs/action/FollowWaypoints` | Sends optimized selected poses to Nav2. |

Example:

```bash
ros2 topic pub --once /tsp_command social_robot_interfaces/msg/TspCommand "{waypoints: [0, 1, 2]}"
```

### `dock_listener`

Parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `database_path` | `docks.db` | SQLite database containing dock poses. |

Subscribes:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/dock_command` | `std_msgs/msg/String` | Any command triggers docking. |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Current pose used to choose nearest dock. |

Clients:

| Interface | Type | Meaning |
| --- | --- | --- |
| Nav2 Simple Commander | `geometry_msgs/msg/PoseStamped` goal through Nav2 | Sends the chosen dock pose to Nav2. |

Example:

```bash
ros2 topic pub --once /dock_command std_msgs/msg/String "{data: dock}"
```

The database table expected by the code is `docks(px, py, pz, qx, qy, qz, qw)`.

### `listening`

Subscribes:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/speech/intent` | `std_msgs/msg/String` | JSON string describing a navigation intent. |

Publishes:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/tour_command` | `std_msgs/msg/String` | Starts the full saved tour. |
| `/tsp_command` | `social_robot_interfaces/msg/TspCommand` | Starts an optimized subset tour. |
| `/dock_command` | `std_msgs/msg/String` | Starts docking. |

Clients:

| Interface | Type | Meaning |
| --- | --- | --- |
| `/tour_retrieve` service | `social_robot_interfaces/srv/Tours` | Used by the `navigate` intent. |
| Nav2 Simple Commander | `geometry_msgs/msg/PoseStamped` goal through Nav2 | Used by `navigate`, `dock`, and stop/cancel behavior. |

Speech intent examples:

```bash
ros2 topic pub --once /speech/intent std_msgs/msg/String \
  "{data: '{\"intent\":\"start_tour\"}'}"
```

```bash
ros2 topic pub --once /speech/intent std_msgs/msg/String \
  "{data: '{\"intent\":\"tsp\", \"waypoints\":[0, 1, 2]}'}"
```

```bash
ros2 topic pub --once /speech/intent std_msgs/msg/String \
  "{data: '{\"intent\":\"dock\"}'}"
```

```bash
ros2 topic pub --once /speech/intent std_msgs/msg/String \
  "{data: '{\"intent\":\"stop_navigation\"}'}"
```

For `navigate`, the current code expects a `location` string whose last character is a waypoint index:

```bash
ros2 topic pub --once /speech/intent std_msgs/msg/String \
  "{data: '{\"intent\":\"navigate\", \"location\":\"waypoint0\"}'}"
```

## Nav2 Waypoint Talk Plugin

`robot_tour::TalkAtWaypoint` is configured in `src/turtlebot3/turtlebot3_navigation2/param/humble/waffle_pi.yaml` under `waypoint_follower`.

Publishes:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/talk_command` by default | `std_msgs/msg/String` | Text command to speak when a waypoint is reached. |

Subscribes:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/done_talking` | `std_msgs/msg/String` | Signal that speech is finished and waypoint following can continue. |

Parameters:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `talk_at_waypoint.enabled` | `true` | Enables or disables the plugin behavior. |
| `talk_at_waypoint.waypoint_pause_duration` | `0` in code, `200` in launch YAML | Sleep interval while waiting. |
| `talk_at_waypoint.talk_topic` | `/talk_command` | Topic used for talk commands. |
| `talk_at_waypoint.default_message` | `Arrived at waypoint ` | Prefix when no per-waypoint message is configured. |
| `talk_at_waypoint.waypoint_messages` | empty list | Optional message per waypoint index. |
| `talk_at_waypoint.max_wait_duration` | `30000` ms | Maximum wait time for `/done_talking`. |

Example completion signal:

```bash
ros2 topic pub --once /done_talking std_msgs/msg/String "{data: done}"
```

## TurtleBot Support Interfaces

The locomotion launch also depends on standard TurtleBot3 and Nav2 topics:

| Node/executable | Key inputs | Key outputs |
| --- | --- | --- |
| `turtlebot3_gazebo` world | Gazebo simulation clock and model state | `/scan`, `/odom`, `/tf`, camera/sensor topics depending on model |
| `turtlebot3_navigation2` Nav2 stack | `/goal_pose`, `/initialpose`, `/map`, `/scan`, `/odom`, `/tf` | `/cmd_vel`, `/follow_waypoints`, planner/controller/status topics |
| `turtlebot3_fake_node` / `turtlebot3_fake_node` | `/cmd_vel` as `geometry_msgs/msg/Twist` | `/odom`, `/joint_states`, `/tf` |
| `turtlebot3_node` / `turtlebot3_ros` | `/cmd_vel` or stamped command velocity, OpenCR hardware | `/odom`, `/imu`, `/magnetic_field`, `/battery_state`, `/joint_states`, `/sensor_state` |

## Quick Checks

List active nodes:

```bash
ros2 node list
```

List active topics:

```bash
ros2 topic list
```

Check that the custom service exists:

```bash
ros2 service list | grep tour_retrieve
```

Check that Nav2 waypoint following exists:

```bash
ros2 action list | grep follow_waypoints
```

Watch talk commands:

```bash
ros2 topic echo /talk_command
```

## Data Files

- `tours.db`: created/used by `tour_manager`; stores saved tour poses.
- `docks.db`: created/used by `dock_listener`; stores dock poses.

Both files are SQLite databases and are created in the process working directory unless you change the code or parameters.
