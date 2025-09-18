#!/bin/bash

# Define the tmux session name
SESSION_NAME="inspire"

# Function to kill the tmux session and all its processes
kill_tmux_session() {
    echo "Caught exit signal. Killing tmux session '$SESSION_NAME'..."
    # Check if the session exists before trying to kill it
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        tmux kill-session -t "$SESSION_NAME"
        echo "Tmux session '$SESSION_NAME' killed successfully."
    else
        echo "Tmux session '$SESSION_NAME' not found."
    fi
}

# Trap termination signals (Ctrl+C, kill, etc.) to run the cleanup function
trap kill_tmux_session EXIT SIGINT SIGTERM

# --- ROS Environment Setup ---
# Activate the ROS environment if not already activated
source /opt/ros/noetic/setup.bash

# Adjust this path to your workspace install/setup.bash
WORKSPACE_SETUP=~/simzx_ws/ros1_inspire_hand/devel/setup.bash
if [ -f "$WORKSPACE_SETUP" ]; then
    source "$WORKSPACE_SETUP"
else
    echo "Workspace setup.bash not found at $WORKSPACE_SETUP"
    exit 1
fi

# --- Tmux Session Management ---
# Check if the tmux session already exists
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "Session '$SESSION_NAME' already exists. Attaching to it."
    tmux attach-session -t "$SESSION_NAME"
else
    echo "Starting a new tmux session named '$SESSION_NAME'..."
    
    # Start a new detached tmux session with roscore
    tmux new-session -d -s "$SESSION_NAME" "roscore"

    # Split the window and run other ROS nodes
    tmux split-window -h -t "$SESSION_NAME:0" "rosrun ros1_inspire_hand driver_ros1.py"
    tmux split-window -v -t "$SESSION_NAME:0" "rosrun ros1_inspire_hand visualizer_ros1.py"
    tmux split-window -v -t "$SESSION_NAME:0" "rosrun ros1_inspire_hand pick_object_node_pid_ros1_open_close.py"
    
    # New: Add another pane for the user input loop
    tmux split-window -v -t "$SESSION_NAME:0" "bash -c 'while true; do echo \"------------------------------------\"; echo \"Please choose a command:\"; echo \"1) Open the fingers\"; echo \"2) Close the fingers\"; echo \"Enter 'exit' to quit the program.\"; read -p \"Enter your choice (1, 2, or 'exit'): \" user_input; case \$user_input in 1) echo \"Publishing 'open' command to /finger_command\"; rostopic pub --once /finger_command std_msgs/String \"open\";; 2) echo \"Publishing 'close' command to /finger_command\"; rostopic pub --once /finger_command std_msgs/String \"close\";; exit) echo \"Exiting program.\"; break;; *) echo \"Invalid input. Please enter 1, 2, or 'exit'.\";; esac; echo \"------------------------------------\"; done'"

    # Adjust the pane layout for better visibility
    tmux select-layout -t "$SESSION_NAME:0" tiled

    # Add a delay to ensure nodes have time to initialize and establish connections
    echo "Waiting for ROS nodes to initialize (5 seconds)..."
    sleep 5
    
    # Attach to the tmux session to make it visible
    tmux attach -t "$SESSION_NAME"
fi

# The 'trap' command will automatically handle cleanup upon script exit.
echo "Script finished. Tmux session will be killed."
exit 0
