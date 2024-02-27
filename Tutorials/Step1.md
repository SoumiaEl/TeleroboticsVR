## ROS TCP Connector Installation

In this step, we will explain how to install the ROS TCP Connector package, enabling the connection between ROS and your Unity project.

### <img src="images/unity-tab-square-white.png" alt="unity" width="24" height="24"/> Unity Setup

First, you need to add the ROS TCP Connector package to Unity by following the installation instructions provided in the link below:

[ROS TCP Connector Package Installation on Unity](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md#-unity-setup)

### <img src="images/ros1_icon.png" alt="ros1" width="28" height="28"/> ROS Environment Setup

Once the ROS TCP Connector package is installed in Unity, you also need to configure your ROS environment. Start by cloning the repository from the following URL:

```
https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

**Important:** Ensure that the version of the Unity package matches exactly with the ROS package version. Otherwise, the integration will not work.

Execute the following commands in a terminal to clone the repository and build your ROS environment:

```bash
cd catkin_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ..
catkin_make
```

#### Setting Up the IP Address:

Start the ROS core by running:

```bash
~/catkin_ws$ roscore
```

Once the ROS Core is launched, it will display `started core service [/rosout]` in the terminal. Open another terminal and run the following command to retrieve your own IP address:

```bash
hostname -I
```

Then, set your IP address in the ROS environment and start the ROS TCP endpoint server by running:

```bash
rosparam set ROS_IP <your IP address>
rosrun ros_tcp_endpoint default_server_endpoint.py
```

Finally, return to Unity and navigate to `Robotics > ROS Settings`. Replace the default "ROS IP Address" in the field with your own IP address. Make sure the protocol is set to "ROS1".

