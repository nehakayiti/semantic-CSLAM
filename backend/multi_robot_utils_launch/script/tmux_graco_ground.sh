#!/bin/bash

SESSION_NAME=graco_ground
BAG_PATH='/opt/bags/graco/ground-01.bag'
BAG_PLAY_RATE=0.5
WS='/opt/slideslam_docker_ws'

CURRENT_DISPLAY=${DISPLAY}
if [ -z ${DISPLAY} ]; then
  CURRENT_DISPLAY=:0
fi

if [ -z ${TMUX} ]; then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

SOURCE="source $WS/devel/setup.bash"

tmux setw -g mouse on

# ── Main window ─────────────────────────────────────────────────────────────
tmux new-window -t $SESSION_NAME -n "Main"
tmux split-window -h -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:1.0
tmux split-window -v -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:1.2
tmux split-window -v -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:1.0
tmux split-window -v -t $SESSION_NAME
tmux select-layout -t $SESSION_NAME tiled

# Pane 0 — fake segmentation (infer_node replaced)
tmux select-pane -t $SESSION_NAME:1.0
tmux send-keys -t $SESSION_NAME "$SOURCE; sleep 2; roslaunch scan2shape_launch infer_node.launch" Enter

# Pane 1 — frame adapter + static TF body->lidar (identity for now)
tmux select-pane -t $SESSION_NAME:1.1
tmux send-keys -t $SESSION_NAME "$SOURCE; sleep 3; rosrun scan2shape_launch graco_frame_adapter.py &
sleep 3; rosrun tf static_transform_publisher 0 0 0 0 0 0 body lidar 100" Enter

# Pane 2 — process cloud node (object extraction)
tmux select-pane -t $SESSION_NAME:1.2
tmux send-keys -t $SESSION_NAME "$SOURCE; sleep 5; roslaunch scan2shape_launch process_cloud_node_outdoor_with_ns.launch odom_topic:=/Odometry" Enter

# Pane 3 — sync measurements with odometry
tmux select-pane -t $SESSION_NAME:1.3
tmux send-keys -t $SESSION_NAME "$SOURCE; sleep 5; roslaunch object_modeller sync_semantic_measurements.launch robot_name:=robot0 odom_topic:=/Odometry" Enter

# Pane 4 — SLOAM backend
tmux select-pane -t $SESSION_NAME:1.4
tmux send-keys -t $SESSION_NAME "$SOURCE; sleep 7; roslaunch sloam single_robot_sloam_test_LiDAR.launch enable_rviz:=true" Enter

# ── roscore window ───────────────────────────────────────────────────────────
tmux new-window -t $SESSION_NAME -n "roscore"
tmux split-window -h -t $SESSION_NAME
tmux select-pane -t $SESSION_NAME:2.0
tmux send-keys -t $SESSION_NAME "roscore" Enter
tmux select-pane -t $SESSION_NAME:2.1
tmux send-keys -t $SESSION_NAME "sleep 2; rosparam set /use_sim_time true" Enter

# ── bag window ───────────────────────────────────────────────────────────────
tmux new-window -t $SESSION_NAME -n "bag"
tmux send-keys -t $SESSION_NAME "sleep 10; rosbag play $BAG_PATH --clock -r $BAG_PLAY_RATE --topics /velodyne/points /gnss/imu /gnss/ground_truth /gnss/ground_truth:=/Odometry" Enter

# ── kill window ──────────────────────────────────────────────────────────────
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"

tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME
