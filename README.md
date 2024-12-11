# armpi-ros-client

ROS Noetic client for [Hiwonder Armpi Pro](https://www.hiwonder.com/products/armpi-pro)

## Getting Started

1. Clone this repository.
    ```
    git clone https://github.com/kywch/armpi-ros-client.git
    ```

    Then, go to the cloned directory.
    ```
    cd armpi-ros-client
    ```

2. Using pixi, set up the virtual environment and install dependencies.
    Install pixi, if you haven't already. See [pixi documentation](https://pixi.sh/latest/#installation) for more details. The following command is for linux.
    ```
    curl -fsSL https://pixi.sh/install.sh | bash
    ```

    The following command sets up the virtual environment and installs the dependencies.
    ```
    pixi install
    ```

3. Build the catkin workspace. If ROS is correctly installed, you should be able to run `catkin_make` in the terminal.
    ```
    cd catkin_ws
    catkin_make
    ```

4. Put the correct `ROS_MASTER_URI` and `ROS_IP` in `ros_setup.sh`. 
    The `ROS_MASTER_URI` and `ROS_IP` are determined by the network configuration of the Armpi Pro and the computer running the ROS client. Armpi Pro has its own WiFi hotspot so that the ROS client can connect to it. Or, Armpi Pro and the client computer can be in the same network. Either way, you must check the ip addresses (e.g., `ip a`) and set the `ROS_MASTER_URI` and `ROS_IP` accordingly.

    To check if ROS is correctly set up, run `rostopic list` in the client terminal. If you see the list of topics, ROS is set up correctly.
    ```
    rostopic list
    ```

5. Run the ROS client.
    Set up the camera viewer.
    ```
    rosrun camera_viewer camera_viewer.py
    ```

    Set up the PS5 controller teleop. The controller must be connected to the client computer (via bluetooth) before running the teleop.
    ```
    rosrun kbd_teleop dsw_teloop.py
    ```
