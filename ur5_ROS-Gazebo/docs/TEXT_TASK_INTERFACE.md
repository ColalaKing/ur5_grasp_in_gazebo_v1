# Text Task Interface

`text_task_interface.py` parses natural text task commands and sends structured
pick-and-place tasks to `grasp_executor`.

Supported command examples:

```text
抓取 3 号方块放到 5 号格子
把三号方块放到五号格
3放5
3 -> 5
pick 3 to grid 5
```

Parsed tasks are published as JSON on:

```text
/grasp/task_command
```

Example payload:

```json
{"pick_digit": 3, "place_grid": 5, "raw_text": "抓取 3 号方块放到 5 号格子"}
```

`grasp_executor` still supports the old `/grasp/target_class_id` `Int32` API.
The old API maps class `N` to grid `N`. The new JSON API supports `pick_digit`
and `place_grid` separately.

## Start Runtime

Start the usual simulation, detection/localization, and executor first:

```bash
cd /root/arm_ws
source /opt/ros/kinetic/setup.bash
source /root/arm_ws/devel/setup.bash
roslaunch ur5_notebook assignment_env.launch seed:=42 gui:=true
```

```bash
cd /root/arm_ws
source /opt/ros/kinetic/setup.bash
source /root/arm_ws/devel/setup.bash
roslaunch ur5_notebook detection_localization.launch
```

```bash
cd /root/arm_ws
source /opt/ros/kinetic/setup.bash
source /root/arm_ws/devel/setup.bash
roslaunch ur5_notebook grasp_executor.launch
```

## Terminal Input

Use `rosrun` for interactive terminal input:

```bash
cd /root/arm_ws
source /opt/ros/kinetic/setup.bash
source /root/arm_ws/devel/setup.bash
rosrun ur5_notebook text_task_interface.py
```

Then type:

```text
抓取 3 号方块放到 5 号格子
```

For a one-shot terminal command:

```bash
rosrun ur5_notebook text_task_interface.py "抓取 3 号方块放到 5 号格子"
```

## ROS Topic Input

Run the interface node:

```bash
roslaunch ur5_notebook text_task_interface.launch enable_stdin:=false
```

Publish raw text commands:

```bash
rostopic pub -1 /task/text_command std_msgs/String "data: '抓取 3 号方块放到 5 号格子'"
```

Optional status output:

```bash
rostopic echo /task/text_status
```

## Structured Command Topic

Advanced users can skip the text parser and publish directly to
`/grasp/task_command`:

```bash
rostopic pub -1 /grasp/task_command std_msgs/String "data: '{\"pick_digit\": 3, \"place_grid\": 5}'"
```
