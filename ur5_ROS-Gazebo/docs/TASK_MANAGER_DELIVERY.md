# E 模块交付说明：顶层任务与状态机（task_manager）

**角色**：顶层任务与状态机工程师（Top-level Task Engineer）  
**模块**：`ur5_ROS-Gazebo/task_manager.py`  
**Launch**：`ur5_ROS-Gazebo/launch/task_manager.launch`  
**依赖**：仿真环境、`detection_localization`、`grasp_executor`（C 模块）

---

## 1. 模块职责

`task_manager` 是系统的 **任务编排层（E 模块）**，不直接控制机械臂运动，而是通过 ROS 话题驱动下游 `grasp_executor` 完成抓放，并订阅检测定位与执行状态，实现：

- 从 YAML **任务表**读取多步抓放任务；
- **顺序执行**或按 **优先级 + 距离** 调度下一步；
- 每步执行前做 **可执行性预检**（映射合法、stable 目标、抓取/放置可达）；
- **全局失败处理**：任一步失败则停止后续任务，并在终端输出结构化失败报告；
- **终端实时日志**：当前步骤、识别快照、目标坐标、规划/抓取状态等。

---

## 2. 与作业要求的对应关系

| 作业条目 | 实现情况 | 说明 |
|----------|----------|------|
| **基础功能 5**：连续任务序列（如 0放1、3放3、1放2） | **已实现序列框架** | 任务表支持多步 `pick_digit` / `place_grid` 顺序执行；见下文「放置约束」 |
| **基础功能 6（部分）**：终端实时输出任务/识别/坐标/规划/夹爪状态 | **已实现** | `task_manager` 日志 + 失败 JSON；规划/夹爪细节由 `grasp_executor` 的 `/grasp/status` 提供 |
| **要求项**：全局失败处理 | **已实现** | 默认 `unreachable_policy:=fail`；检测/规划/抓取失败均停止并输出 `failure_type` 与原因 |
| **拓展功能 1**：多目标按任务表顺序连续抓放 | **已实现** | `enable_extension1_multi:=true`（默认开启） |
| **拓展功能 7**：按距离或优先级调度顺序 | **已实现** | `enable_extension7_scheduler:=true`，公式 `score = priority - alpha × distance3D` |

### 2.1 关于「0放1、3放3、1放2」的说明

当前 **C 模块 `grasp_executor` 的放置规则**为：抓取 `digit_N` 后 **固定放到 `grid_N`**（`class_id == grid_id`）。

因此可执行步骤必须满足：

```text
pick_digit == place_grid == final_class_id   （且 digit 在 1~6）
```

- **3放3**：合法，可执行；
- **0放1、1放2**：`pick_digit ≠ place_grid`，`task_manager` 在**下发前**判为映射不合法，输出 `不可放置失败` 并 **全局停止**（fail 策略下）。

若作业演示需要「跨格子放置」，需 C 模块扩展放置映射；E 模块已预留映射校验与失败上报。

---

## 3. 系统架构与话题

```text
detection + block_localization
        │
        ▼
/localization/grasp_targets  ──►  task_manager  ──►  /grasp/target_class_id
        │                              ▲                    │
        │                              │                    ▼
        └──────────────────────  /grasp/status  ◄──  grasp_executor
```

| 话题 | 方向（相对 task_manager） | 含义 |
|------|---------------------------|------|
| `/localization/grasp_targets` | 订阅 | 稳定检测目标（含 `pose_base` / `pose_world`） |
| `/grasp/target_class_id` | 发布 | 下发抓取命令（`Int32`，值为 `place_grid`） |
| `/grasp/status` | 订阅 | 执行状态 JSON（`idle/approaching/.../done/failed`） |

---

## 4. 状态机流程

单步任务逻辑（简化）：

```text
BUILD_TASK_QUEUE
    →（可选）WAIT_INITIAL_STABLE_TARGETS
    → SELECT_NEXT_TASK          # 顺序模式或拓展7调度
    →（可选）EXECUTABILITY_PRECHECK  # skip/fail
    → WAIT_STABLE_TARGET        # 等待 pick_digit 出现在 stable 集合
    → SEND_PICK_AND_PLACE       # 发布 /grasp/target_class_id
    → WAIT_GRASP_STATUS_DONE    # 等待 state==done
    →（成功）下一步 /（失败）GLOBAL_FAIL_STOP
```

`grasp_executor` 内部状态机（由 `/grasp/status` 反映）：

```text
idle → approaching → descending → contacting → vacuum_on
         → retreating → placing → returning_home → done
```

失败时可能出现：`failed` / `rejected` / 超时。

---

## 5. 已实现功能详述

### 5.1 基础功能 5：连续任务序列

- 任务来源：`config/task_table.yaml` 或 launch 指定 `task_table_yaml:=...`
- 多步任务按 `order` 字段排序（拓展1 关闭调度时）；
- 每步完成后自动进入下一步，直至队列空或失败退出。

**任务表示例**（`config/task_table.yaml`）：

```yaml
task_table:
  - order: 1
    pick_digit: 6
    place_grid: 6
    priority: 10
  - order: 2
    pick_digit: 5
    place_grid: 5
    priority: 8
```

### 5.2 基础功能 6（部分）：终端实时输出

**正常运行时**，D 终端（task_manager）典型输出：

```text
[INFO] task_manager started: ... steps=2 multi=True scheduler=False
[INFO] Executing step: step_idx=0 pick_digit=6 place_grid=6 -> final_class_id=6
[INFO] Step finished: step_idx=0 executed=1
[INFO] inter_grasp_delay: waiting 5.0s for controller settle before next step
```

**失败时**，输出 JSON 报告（同时 `rospy.logerr`），字段包括：

| 字段 | 含义 |
|------|------|
| `failure_type` | `检测失败` / `规划失败` / `抓取失败` / `不可放置失败` |
| `step` | `step_index`, `pick_digit`, `place_grid`, `final_class_id` |
| `latest_recognition_snapshot_stable` | 当前 stable 目标列表（class_id、confidence、yaw_deg） |
| `latest_target_pose` | 本步抓取目标的 `pose_base` / `pose_world` |
| `grasp_status` | 来自 `/grasp/status` 的 `state` 与 `message` |

**skip 策略**（`unreachable_policy:=skip`）时，不可执行步骤输出 `event: skipped` 及 `reason_code`，不触发全局失败。

C 终端（grasp_executor）可查看更细的规划/夹爪状态，例如：

```text
grasp_executor [approaching]: moving to class 6
grasp_executor [vacuum_on]: vacuum ON for class 6
grasp_executor [done]: grasp complete for class 6
```

### 5.3 要求项：全局失败处理

| 失败类型 | 典型触发条件 |
|----------|----------------|
| **检测失败** | 规定时间内 `pick_digit` 未出现在 stable 目标中 |
| **不可放置失败** | `pick_digit≠place_grid`；digit=0；放置格/抓取点预检不可达（fail 策略） |
| **规划失败** | `/grasp/status` 含 planning/cartesian 失败，或等待 `done` 超时 |
| **抓取失败** | 执行/控制器/vacuum/attach 失败 |

默认 **`unreachable_policy:=fail`**：任一步执行失败 → 停止剩余任务。  
演示可用 **`unreachable_policy:=skip`**：跳过不可达/不可执行步骤，继续后续任务。

### 5.4 拓展功能 1：多目标顺序抓放

- Launch 参数：`enable_extension1_multi:=true`（默认）
- 行为：按任务表 `order` 依次执行全部 step
- 演示配置：`config/task_table_demo_sequence.yaml`（推荐顺序 **6→5**，先抓较远目标）

### 5.5 拓展功能 7：简单任务调度

- Launch 参数：`enable_extension7_scheduler:=true`
- 调度公式：

```text
score = priority - alpha × distance3D
distance3D = sqrt(x² + y² + z²)   # 使用 pose_base 三维坐标
alpha 默认 0.6，越大越偏向「近的优先」
```

- 仅在 **当前 stable 且通过可执行性预检** 的 pending 步骤中选 score 最高者；
- 演示配置：`config/task_table_demo_scheduler.yaml`（含 skip 不可达 step 2 的示例）

### 5.6 可达性预检（辅助 demo 稳定）

| 参数 | 默认 | 说明 |
|------|------|------|
| `reach_radius_xy` | 0.80 | base 系 `hypot(x,y)` 抓取/放置半径 |
| `require_place_reachable` | true | 预检放置格是否可达 |
| `unreachable_policy` | fail | `skip` 时跳过不可达步骤 |

放置侧：**grid 4/5/6（底排）可达**；grid 1/2/3（上排）在当前几何下不可达。

---

## 6. 运行环境

- Docker 容器：`ur5_grasp_kinetic`
- 工作空间：`/root/arm_ws`（宿主机目录挂载：`ur5_grasp_in_gazebo_v1`）
- 启动前需 source：

```bash
source /opt/ros/kinetic/setup.bash
source /root/arm_ws/devel/setup.bash
```

---

## 7. 终端启动命令（4 终端）

以下命令在 **Windows PowerShell** 下执行；每行一条 `docker exec`。**四个终端须全程保持运行**，尤其不要中途 Ctrl+C 掉 `grasp_executor`。

### 终端 A：仿真（Gazebo + MoveIt）

**方式 1 — VNC 图形界面（推荐录制）**

宿主机先确保容器已启动，再在容器内：

```bash
docker exec -it ur5_grasp_kinetic bash -lc "GAZEBO_SEED=42 GAZEBO_GUI=true /root/arm_ws/src/ur5_grasp_in_gazebo/scripts/start_gazebo_vnc.sh"
```

VNC 连接：`127.0.0.1:5901`（或 README 中配置的端口）。

**方式 2 — 命令行 roslaunch**

```bash
docker exec -it ur5_grasp_kinetic bash -lc "cd /root/arm_ws && source /opt/ros/kinetic/setup.bash && source /root/arm_ws/devel/setup.bash && roslaunch ur5_notebook assignment_env.launch seed:=42 gui:=true"
```

等待日志出现场景就绪（如 `Assignment scene ready` 或 move_group ready）。

### 终端 B：检测 + 定位

```bash
docker exec -it ur5_grasp_kinetic bash -lc "cd /root/arm_ws && source /opt/ros/kinetic/setup.bash && source /root/arm_ws/devel/setup.bash && roslaunch ur5_notebook detection_localization.launch"
```

### 终端 C：抓取执行器（grasp_executor）

```bash
docker exec -it ur5_grasp_kinetic bash -lc "cd /root/arm_ws && source /opt/ros/kinetic/setup.bash && source /root/arm_ws/devel/setup.bash && roslaunch ur5_notebook grasp_executor.launch"
```

等待：`grasp_executor ready`

### 终端 D：任务管理器（task_manager，E 模块）

**Demo A — 拓展功能 1：顺序多步抓放**

```bash
docker exec -it ur5_grasp_kinetic bash -lc "cd /root/arm_ws && source /opt/ros/kinetic/setup.bash && source /root/arm_ws/devel/setup.bash && roslaunch ur5_notebook task_manager.launch task_table_yaml:=/root/arm_ws/src/ur5_grasp_in_gazebo/ur5_ROS-Gazebo/config/task_table_demo_sequence.yaml enable_extension7_scheduler:=false unreachable_policy:=skip"
```

**Demo B — 拓展功能 7：优先级 + 距离调度**

```bash
docker exec -it ur5_grasp_kinetic bash -lc "cd /root/arm_ws && source /opt/ros/kinetic/setup.bash && source /root/arm_ws/devel/setup.bash && roslaunch ur5_notebook task_manager.launch task_table_yaml:=/root/arm_ws/src/ur5_grasp_in_gazebo/ur5_ROS-Gazebo/config/task_table_demo_scheduler.yaml enable_extension7_scheduler:=true alpha:=0.6 unreachable_policy:=skip"
```

**默认任务表 + 全局 fail 策略（严格失败即停）**

```bash
docker exec -it ur5_grasp_kinetic bash -lc "cd /root/arm_ws && source /opt/ros/kinetic/setup.bash && source /root/arm_ws/devel/setup.bash && roslaunch ur5_notebook task_manager.launch unreachable_policy:=fail"
```

---

## 8. 可选参数说明

| Launch 参数 | 默认值 | 说明 |
|-------------|--------|------|
| `task_table_yaml` | `config/task_table.yaml` | 任务表 YAML 路径 |
| `enable_extension1_multi` | true | 是否执行多步（false 时只跑第一步） |
| `enable_extension7_scheduler` | false | 是否启用调度 |
| `alpha` | 0.6 | 距离权重 |
| `unreachable_policy` | fail | `fail` 或 `skip` |
| `reach_radius_xy` | 0.80 | 可达半径（米） |
| `inter_grasp_delay` | 5.0 | 连抓步骤间隔（秒） |
| `status_wait_timeout` | 90.0 | 单步等待 `done` 超时（秒） |

修改任务表：编辑 `ur5_ROS-Gazebo/config/task_table.yaml` 或 demo YAML，无需改 Python 代码。

---

## 9. 对照：无 task_manager 的单步抓取（C 模块基线）

用于验证底层抓放是否正常（README 第 13 节）：

```bash
# 终端 A/B/C 同上，终端 D 改为：
docker exec -it ur5_grasp_kinetic bash -lc "cd /root/arm_ws && source /opt/ros/kinetic/setup.bash && source /root/arm_ws/devel/setup.bash && rostopic pub -1 /grasp/target_class_id std_msgs/Int32 'data: 6'"

docker exec -it ur5_grasp_kinetic bash -lc "cd /root/arm_ws && source /opt/ros/kinetic/setup.bash && source /root/arm_ws/devel/setup.bash && rostopic echo /grasp/status"
```

`-1` 表示自动选择当前可达的 1~6 中一个目标。

---

## 10. 交付文件清单

| 路径 | 说明 |
|------|------|
| `ur5_ROS-Gazebo/task_manager.py` | E 模块主程序 |
| `ur5_ROS-Gazebo/launch/task_manager.launch` | 启动与参数 |
| `ur5_ROS-Gazebo/config/task_table.yaml` | 默认任务表 |
| `ur5_ROS-Gazebo/config/task_table_demo_sequence.yaml` | 拓展1 演示任务表 |
| `ur5_ROS-Gazebo/config/task_table_demo_scheduler.yaml` | 拓展7 演示任务表 |
| `README.md` 第 14 节 | 简要 demo 说明 |

---

## 11. 调试过程：失败原因与处理总结

本节汇总 E 模块联调过程中遇到的典型问题。**多数失败并非 task_manager 调度逻辑错误**，而是环境、任务表、下游执行器或仿真控制器导致；E 模块负责正确编排、预检与失败上报。

### 11.1 问题一览

| # | 现象 | 主要原因 | 处理方式 |
|---|------|----------|----------|
| 1 | 启动后 0.02s 内全部 `skipped`，`stable_missing`，机械臂不动 | `unreachable_policy=skip` 在**首帧检测未到**时就做可达性判断 | `task_manager` 增加 `wait_for_initial_targets()`；顺序模式下对 `stable_missing` 先 `wait_for_stable_pick` 再决定是否 skip |
| 2 | 只执行 1 步就结束 | `task_table.yaml` 里**只配了一行**任务 | 在 YAML 中增加多步；确认 `enable_extension1_multi:=true` |
| 3 | 6/5/4 中只成功 6，其余 `pick_unreachable` skip | 方块 spawn 位置远，`hypot(x,y)>0.8`；**换 seed 不能保证** 4/5/6 都近 | 收窄 `config/assignment_scene.yaml` 的 spawn 区间；`unreachable_policy:=skip`；任务表优先底排 4/5/6；不适配 seed 时换 seed 重开仿真 |
| 4 | `timeout waiting /grasp/status done` | **C 终端 grasp_executor 被 Ctrl+C** 或未启动 | 四终端全程保持；先等 C 出现 `grasp_executor ready` 再启 D |
| 5 | `规划失败` + timeout，但日志显示曾 `done` | 同上，或 D 与 C 不同 rosmaster | 同一容器内启动；勿混杀进程 |
| 6 | 第一次抓成功，第二次 `approach execution failed` | **连抓 + 第二目标更远**；Gazebo PID 未收敛；非调度 bug | `inter_grasp_delay`（默认 5s）；demo 顺序改为 **6→5**（先抓远后抓近）；单步 `rostopic pub 6` 验证底层能力 |
| 7 | `Invalid Trajectory: start point deviates > 0.01` | MoveIt 默认 `allowed_start_tolerance=0.01` 过严，停稳后仍有 ~0.01rad 抖动 | `assignment_env.launch` 设 `allowed_start_tolerance:=0.1`；**须重启仿真**生效 |
| 8 | `pre_descend` / `descend` 的 `drift` 大 → cartesian ABORT | approach 后关节仍在振荡；`move_to_pose` 原先仅 `sleep(1.0)` | `grasp_executor` 中 cartesian/approach 增加 `wait_until_settled`、漂移重规划、`recover_to_up`；approach 与 cartesian 之间额外等待 |
| 9 | 机械臂「碰撞/乱扭」，`elbow_joint ≈ -42 rad` | 控制器 ABORT 后关节**绕圈**，进入非法姿态 | **重启仿真**复位；失败重试前 `recover_to_up`；避免在非法姿态上继续发令 |
| 10 | `pre_descend cartesian fraction=0.00` | 上一条原因导致起点非法，笛卡尔路径算不出 | 重启仿真；不要在中途 Ctrl+C 后继续 |
| 11 | 开录制时易失败，关录制较稳 | 录屏/VNC 负载高，Gazebo 控制器跟不上 | 验收时先关录制；或只录单步成功片段 |
| 12 | 调度 demo 先抓 5 再抓 6 易崩 | `priority` 相同时公式选**更近的 5**；第二步 6 更远 | 调度表设 **6 的 priority 高于 5**；或顺序 demo 用 `6→5` |
| 13 | digit 2 总被 skip | **grid 2 在上排**，放置点 `place_unreachable`（与 seed 无关） | 预期行为；用于演示 skip；演示抓放请用 grid 4/5/6 |
| 14 | 「加了 task_manager 就不行」 | task_manager 只发 `/grasp/target_class_id`；**固定序列 + 连抓** 暴露仿真极限 | 对比 README §13 单步 `rostopic pub`；确认 E 与 C 分工 |
| 15 | 容器反复试验后整体变抖 | 长时间试验 + ABORT 累积 | `docker restart ur5_grasp_kinetic` 后按 A→B→C→D 干净启动 |

### 11.2 关键结论（答辩可用）

1. **E 模块（task_manager）**：任务表驱动、状态等待、失败 JSON、拓展1 顺序 / 拓展7 调度、可达性预检 — **逻辑已实现**。
2. **执行层（grasp_executor + Gazebo）**：单步、从 home 出发抓 digit 6 **可以成功**；**连续多步且第二步目标较远**时，仿真控制器易 ABORT — 属于仿真/连抓工程问题，不是 task_manager 算错顺序。
3. **推荐稳定演示**：`seed:=42` + 顺序 demo（`task_table_demo_sequence.yaml`，**6→5**）+ `unreachable_policy:=skip` + 四终端不关 + 必要时 `inter_grasp_delay:=5` + **重启仿真后再录**。

### 11.3 相关代码/配置改动（联调期，供追溯）

| 位置 | 改动要点 |
|------|----------|
| `task_manager.py` | 启动预热、`skip` 前等待 stable、`inter_grasp_delay`、失败/skip JSON |
| `task_manager.launch` | 暴露 `unreachable_policy`、`reach_radius_xy`、`inter_grasp_delay` 等 |
| `assignment_env.launch` | `allowed_start_tolerance:=0.1` |
| `assignment_scene.yaml` | spawn 区间收窄（更靠近 base） |
| `grasp_executor.py` | 停稳等待、笛卡尔漂移重试、`recover_to_up`、`go_up_before_grasp` / `return_home_after_grasp` 开关 |
| `config/task_table_demo_*.yaml` | 顺序 6→5；调度 6 优先于 5 |
| `sequential_grasp_5_6.py` | 可选：抓 5 不回 home 直连 6（实验脚本，非主交付路径） |

---

## 12. 已知限制与说明（答辩/报告可引用）

1. **放置映射**：当前流水线仅支持 `digit_N → grid_N`；跨格任务（如 1放2）由 E 模块拦截并报告，非 E 模块缺陷。
2. **仿真连抓**：Gazebo 下连续多步抓放受控制器跟踪精度影响；单步抓取通常比连抓更稳。可通过 `inter_grasp_delay`、换 `seed`、任务表只选可达 digit 改善。
3. **digit 0 / 7~9**：无对应放置格，`grasp_executor` 会 `rejected`。
4. **检测依赖**：任务开始前需 B 终端检测定位正常，`/localization/grasp_targets` 有 stable 目标。

---

## 13. 验收检查清单

- [ ] 四终端启动顺序：A → B → C（ready）→ D
- [ ] D 终端出现 `task_manager started` 且 `steps=N`
- [ ] 至少一步出现 `Executing step: ...` 且 C 终端机械臂有动作
- [ ] 成功时 D 终端 `Step finished` / `All steps done`
- [ ] 人为制造失败（如错误 task 映射）时 D 终端输出含 `failure_type` 的 JSON 并停止
- [ ] 拓展7：`scheduler=True` 且执行顺序与 `priority`/距离一致（见 demo_scheduler 配置）

---

*文档版本：与当前仓库 `task_manager.py` / `task_manager.launch` 及 config 目录一致。*
