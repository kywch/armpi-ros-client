#!/bin/bash
# ros_setup.sh

# Can add error checking
if [ ! -f "$PIXI_PROJECT_ROOT/catkin_ws/devel/setup.bash" ]; then
    echo "Error: ROS workspace not built"
    exit 1
fi

# Can add more complex logic
source "$PIXI_PROJECT_ROOT/catkin_ws/devel/setup.bash"

# Can add additional ROS configurations
export ROS_IP=10.0.0.239

# Check the ROS host's IP, /etc/hosts
export ROS_MASTER_URI=http://10.0.0.78:11311

# Can add workspace validations
if ! command -v roscore >/dev/null 2>&1; then
    echo "Warning: roscore not found"
fi
