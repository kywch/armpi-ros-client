#!/bin/bash
# ros_setup.sh

# Can add additional ROS configurations
export ROS_IP=10.0.0.239

# Check the ROS host's IP, /etc/hosts
export ROS_MASTER_URI=http://10.0.0.78:11311

# Can add workspace validations
if ! command -v roscore >/dev/null 2>&1; then
    echo "Warning: roscore not found"
fi

# Can add error checking
# Run setup.bash if exists, otherwise show warning
if [ -f "$PIXI_PROJECT_ROOT/catkin_ws/devel/setup.bash" ]; then
    source "$PIXI_PROJECT_ROOT/catkin_ws/devel/setup.bash"
else
    echo "Warning: ROS workspace not built. Please run 'catkin_make' first."
fi
