# 测试与文字控制接口交付报告

## 1. 交付内容

本次交付完成两个基础功能：

1. 数据记录与重复实验测试：编写实验记录脚本，完成 5 次重复抓取实验，统计识别成功率、规划成功率、抓取成功率和平均任务耗时。
2. 文字控制接口：支持通过终端或 ROS 话题输入自然语言任务，例如“抓取 3 号方块放到 5 号格子”，并下发给抓取执行器执行。

相关文件：

| 文件 | 说明 |
|------|------|
| `experiment_recorder.py` | 5 次重复实验记录脚本 |
| `launch/experiment_recorder.launch` | 实验记录启动文件 |
| `experiments/results/experiment_20260601_141654.csv` | 5 次实验 CSV 明细 |
| `experiments/results/experiment_20260601_141654.json` | 5 次实验 JSON 明细与状态事件 |
| `text_task_interface.py` | 文字任务解析与转发节点 |
| `launch/text_task_interface.launch` | 文字接口启动文件 |
| `grasp_executor.py` | 新增 `/grasp/task_command` 结构化任务接口 |
| `task_manager.py` | 支持 `pick_digit` 与 `place_grid` 分离下发 |
| `docs/TEXT_TASK_INTERFACE.md` | 文字接口使用说明 |

## 2. 实验测试功能

### 2.1 测试目标

实验记录脚本用于自动执行多次抓取任务，并记录每次任务的关键结果：

- 识别成功率：目标数字方块是否在超时时间内被稳定识别。
- 规划成功率：抓取执行器是否进入规划后的执行阶段。
- 抓取成功率：抓取执行器是否最终返回 `done`。
- 平均任务耗时：从单次任务开始到成功、失败或超时的墙钟时间。

### 2.2 统计口径

| 指标 | 判定方式 |
|------|----------|
| `recognition_success` | `/localization/grasp_targets` 中出现目标 `class_id` 且 `stable=True` |
| `planning_success` | `/grasp/status` 进入 `descending`、`contacting`、`vacuum_on`、`retreating`、`placing`、`returning_home` 或 `done` |
| `grasp_success` | `/grasp/status` 返回 `state=done` |
| `task_time_sec` | 从本轮开始到终止状态的耗时 |

### 2.3 运行命令

启动仿真、检测定位和抓取执行器后，运行：

```bash
cd /root/arm_ws
source /opt/ros/kinetic/setup.bash
source /root/arm_ws/devel/setup.bash

roslaunch ur5_notebook experiment_recorder.launch \
  target_class_id:=6 \
  repetitions:=5 \
  reset_between_trials:=true \
  inter_trial_delay:=8.0 \
  execution_timeout:=180.0 \
  output_dir:=/root/arm_ws/experiment_results
```

### 2.4 测试结果

测试目标为 6 号方块，共执行 5 次重复实验。

| 指标 | 结果 |
|------|------|
| 目标方块 | 6 |
| 重复次数 | 5 |
| 识别成功率 | 100% |
| 规划成功率 | 100% |
| 抓取成功率 | 100% |
| 平均任务耗时 | 18.77 s |

单次实验明细：

| 次数 | 识别成功 | 规划成功 | 抓取成功 | 耗时 |
|------|----------|----------|----------|------|
| 1 | True | True | True | 18.81 s |
| 2 | True | True | True | 18.70 s |
| 3 | True | True | True | 18.39 s |
| 4 | True | True | True | 19.35 s |
| 5 | True | True | True | 18.60 s |

结果文件：

```text
ur5_ROS-Gazebo/experiments/results/experiment_20260601_141654.csv
ur5_ROS-Gazebo/experiments/results/experiment_20260601_141654.json
```

## 3. 文字控制接口

### 3.1 功能说明

文字控制接口用于将自然语言任务解析为结构化抓放命令。用户可以通过终端输入，也可以通过 ROS 话题输入。

支持示例：

```text
抓取 3 号方块放到 5 号格子
把三号方块放到五号格
3放5
3 -> 5
pick 3 to grid 5
```

解析结果示例：

```json
{
  "pick_digit": 3,
  "place_grid": 5,
  "raw_text": "抓取 3 号方块放到 5 号格子"
}
```

### 3.2 ROS 接口

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/task/text_command` | `std_msgs/String` | 输入 | 原始文字任务 |
| `/task/text_status` | `std_msgs/String` | 输出 | 文字解析状态 |
| `/grasp/task_command` | `std_msgs/String` | 输出 | 结构化 JSON 抓放任务 |
| `/grasp/status` | `std_msgs/String` | 输入/监控 | 抓取执行器状态 |

新增结构化命令格式：

```json
{"pick_digit": 3, "place_grid": 5}
```

其中：

- `pick_digit` 范围为 0 到 9。
- `place_grid` 范围为 1 到 6。
- 旧接口 `/grasp/target_class_id` 仍保留，语义为抓取 N 号方块并放到 N 号格子。

### 3.3 终端输入方式

启动文字接口：

```bash
cd /root/arm_ws
source /opt/ros/kinetic/setup.bash
source /root/arm_ws/devel/setup.bash

rosrun ur5_notebook text_task_interface.py
```

输入：

```text
抓取 3 号方块放到 5 号格子
```

### 3.4 ROS 话题输入方式

启动文字接口节点：

```bash
roslaunch ur5_notebook text_task_interface.launch enable_stdin:=false
```

发布文字任务：

```bash
rostopic pub -1 /task/text_command std_msgs/String "data: '抓取 3 号方块放到 5 号格子'"
```

查看解析状态：

```bash
rostopic echo /task/text_status
```

查看抓取执行状态：

```bash
rostopic echo /grasp/status
```

## 4. 集成流程

文字接口的整体链路如下：

```text
终端输入 / ROS 话题输入
        |
        v
/task/text_command
        |
        v
text_task_interface.py
        |
        v
/grasp/task_command
        |
        v
grasp_executor.py
        |
        v
MoveIt + Gazebo 执行抓放
```

启动完整测试环境时，需要运行以下模块：

1. `assignment_env.launch`：启动 Gazebo、MoveIt、机械臂、方块和目标格。
2. `detection_localization.launch`：启动识别和定位。
3. `grasp_executor.launch`：启动抓取执行器。
4. `text_task_interface.launch` 或 `text_task_interface.py`：启动文字接口。

## 5. 验证情况

已完成以下验证：

| 验证项 | 结果 |
|--------|------|
| Python 语法检查 | 通过 |
| ROS Kinetic Python 2 兼容性检查 | 通过 |
| 中文任务解析测试 | 通过 |
| 英文/简写任务解析测试 | 通过 |
| `/task/text_command` 到 `/grasp/task_command` 话题链路 | 通过 |
| 5 次重复抓取实验 | 通过 |

解析测试样例：

```text
抓取 3 号方块放到 5 号格子 -> pick_digit=3, place_grid=5
3放5 -> pick_digit=3, place_grid=5
pick 3 to grid 5 -> pick_digit=3, place_grid=5
```

## 6. 注意事项

1. 重启 Gazebo 场景后，需要重新启动检测定位、抓取执行器和文字接口，否则只会看到场景，输入任务不会执行。
2. 如果用户在 Gazebo GUI 中拖动了方块或机械臂，建议重启场景，而不是手动摆回。
3. VNC 录屏时建议先确认 `/localization/grasp_targets` 有稳定目标，再发布文字任务。
4. 若跨格子放置失败，应优先检查目标方块和目标格是否在机械臂可达范围内。
