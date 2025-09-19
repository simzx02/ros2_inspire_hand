#!/bin/bash

SESSION_NAME="123"
KILL_FLAG="/tmp/${SESSION_NAME}_kill.flag"

# Cleanup function
cleanup() {
    echo "Cleaning up tmux session '$SESSION_NAME'..."
    tmux kill-session -t "$SESSION_NAME" 2>/dev/null
    rm -f "$KILL_FLAG"
}
trap cleanup EXIT

# Source ROS
source /opt/ros/noetic/setup.bash
source ~/simzx_ws/ros1_inspire_hand/devel/setup.bash

# Start new tmux session
tmux new-session -d -s "$SESSION_NAME" "roscore"

# Split and run ROS nodes
tmux split-window -h -t "$SESSION_NAME:0" "rosrun ros1_inspire_hand driver_ros1.py"
tmux split-window -v -t "$SESSION_NAME:0" "rosrun ros1_inspire_hand visualizer_ros1.py"
tmux split-window -v -t "$SESSION_NAME:0" "rosrun ros1_inspire_hand pick_object_node_pid_ros1_open_close.py"

# Split for user interaction (input commands)
tmux split-window -v -t "$SESSION_NAME:0" "bash -c '
while true; do
    echo \"------------------------------------\"
    echo \"Choose a command:\"
    echo \"1) Open the fingers\"
    echo \"2) Close the fingers\"
    echo \"Type 'exit' to quit.\"
    read -p \"Enter your choice (1, 2, or exit): \" input
    case \$input in
        1) echo \"Sending 'open'...\"; rostopic pub --once /finger_command std_msgs/String \"open\";;
        2) echo \"Sending 'close'...\"; rostopic pub --once /finger_command std_msgs/String \"close\";;
        exit) echo \"Exiting...\"; touch \"$KILL_FLAG\"; break;;
        *) echo \"Invalid input.\";;
    esac
done
'"

# Add kill monitor (watch kill flag)
tmux split-window -v -t "$SESSION_NAME:0" "bash -c '
while true; do
    if [ -f \"$KILL_FLAG\" ]; then
        echo \"Kill flag detected. Terminating tmux session '$SESSION_NAME'.\"
        tmux kill-session -t \"$SESSION_NAME\"
        break
    fi
    sleep 1
done
'"

# Layout fix
tmux select-layout -t "$SESSION_NAME:0" tiled

# Allow ROS to initialize
sleep 3

# Attach to the tmux session
tmux attach -t "$SESSION_NAME"
