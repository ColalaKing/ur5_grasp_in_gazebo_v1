# Experiment Recording

This package includes `experiment_recorder.py` for the required five repeated
grasp trials. It drives the existing pipeline through `/grasp/target_class_id`
and records results from `/localization/grasp_targets` and `/grasp/status`.

## Metrics

For each trial, the recorder writes:

- `recognition_success`: the target class appears as a stable localization target
  before `detection_timeout`.
- `planning_success`: `grasp_executor` reaches a post-planning execution phase
  such as `descending`, `contacting`, `vacuum_on`, `retreating`, `placing`,
  `returning_home`, or `done`.
- `grasp_success`: `grasp_executor` reports `/grasp/status` state `done`.
- `task_time_sec`: wall time from trial start to terminal status or timeout.

The summary reports recognition success rate, planning success rate, grasp
success rate, and average task time across all trials.

## Run

Start the existing simulation, perception, and executor terminals first:

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

Then run five repeated trials:

```bash
cd /root/arm_ws
source /opt/ros/kinetic/setup.bash
source /root/arm_ws/devel/setup.bash
roslaunch ur5_notebook experiment_recorder.launch target_class_id:=6 repetitions:=5
```

If each repetition should reset Gazebo to the initial scene, enable reset:

```bash
roslaunch ur5_notebook experiment_recorder.launch target_class_id:=6 repetitions:=5 reset_between_trials:=true
```

Results are written by default to:

```text
/root/arm_ws/experiment_results/experiment_YYYYmmdd_HHMMSS.csv
/root/arm_ws/experiment_results/experiment_YYYYmmdd_HHMMSS.json
```

The CSV is suitable for the report table. The JSON contains the same summary
plus stable target snapshots and all `/grasp/status` events for failure analysis.

## Latest Recorded Result

Run command:

```bash
roslaunch ur5_notebook experiment_recorder.launch target_class_id:=6 repetitions:=5 reset_between_trials:=true inter_trial_delay:=8.0 execution_timeout:=180.0 output_dir:=/root/arm_ws/experiment_results
```

Result files:

```text
experiments/results/experiment_20260601_141654.csv
experiments/results/experiment_20260601_141654.json
```

| Metric | Value |
|--------|-------|
| Target class | 6 |
| Repetitions | 5 |
| Recognition success rate | 100% |
| Planning success rate | 100% |
| Grasp success rate | 100% |
| Average task time | 18.77 s |

Trial times:

| Trial | Recognition | Planning | Grasp | Time (s) |
|-------|-------------|----------|-------|----------|
| 1 | True | True | True | 18.81 |
| 2 | True | True | True | 18.70 |
| 3 | True | True | True | 18.39 |
| 4 | True | True | True | 19.35 |
| 5 | True | True | True | 18.60 |
