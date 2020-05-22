[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

# RoboND-simple_arm

A mini-project to better explain pub-sub architecture in ROS.

![Simulation interface](images/simulation.png)

## How to Launch the simulation?

#### Create a catkin_ws, feel free to skip if you already have one!
```bash
cd /home/workspace/
mkdir -p /home/workspace/catkin_ws/src/
cd catkin_ws/src/
catkin_init_workspace
cd ..
```

#### Clone the package in catkin_ws/src/
```bash
cd /home/workspace/catkin_ws/src/
git clone https://github.com/udacity/RoboND-simple_arm.git simple_arm
```

#### Build the `simple_arm` package
```bash
cd /home/workspace/catkin_ws/
catkin_make
```

#### After building the package, source your environment
```bash
cd /home/workspace/catkin_ws/
source devel/setup.bash
```

#### Make sure to check and install any missing dependencies
```bash
rosdep install -i simple_arm
```

#### Once the `simple_arm` package has been built, you can launch the simulation environment using
```bash
roslaunch simple_arm robot_spawn.launch
```

#### Interact with the arm using the safe_move service
Open a new terminal and type the following:
```bash
cd /home/workspace/catkin_ws/
source devel/setup.bash
rosservice call /arm_mover/safe_move "joint_1: 0.0 joint_2: 0.0"
```

When trying to move the arm using the following command (line break required!),

```bash
rosservice call /arm_mover/safe_move "joint_1: 1.57
joint_2: 1.57"
```

the ROS server will report an error:

```
[ WARN] [1590169890.483779630, 524.840000000]: j2 is out of bounds, valid range (0.00,1.00), clamping to: 1.00
```

We can update the joint limits by reaching out to the parameter server …

```bash
rosparam set /arm_mover/max_joint_2_angle 1.57
```

… after which the `rosservice` call above will succeed as intended.

## How to view image stream from the camera?
Camera image stream is published to the following topic:
```
/rgb_camera/image_raw
```

This stream can be viewed by following command in separate terminal:
```bash
rosrun image_view image_view image:=/rgb_camera/image_raw
```


---

## Fixing issues for Kinetic

### Error: `legacyModeNS`

Originally, the code produced the following output:

```
[ERROR] [1590091780.648877316, 261.772000000]: GazeboRosControlPlugin missing <legacyModeNS> while using DefaultRobotHWSim, defaults to true.
This setting assumes you have an old package with an old implementation of DefaultRobotHWSim, where the robotNamespace is disregarded and absolute paths are used instead.
If you do not want to fix this issue in an old package just set <legacyModeNS> to true.
```

Following the answers to [this](https://answers.ros.org/question/292444/gazebo_ros_control-plugin-gazeboroscontrolplugin-missing-legacymodens-defaultrobothwsim/) question, we can find the file that needs changes by running

```bash
grep robotSimType -R .
```

This gives

```
./src/simple_arm/urdf/simple_arm.gazebo.xacro
```

Changing the section

```xml
<gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
        <robotNamespace>/simple_arm</robotNamespace>
        <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
</gazebo>
```

to contain `<legacyModeNS>true</legacyModeNS>` like so suppresses the error:

```xml
<gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
        <legacyModeNS>true</legacyModeNS>
        <robotNamespace>/simple_arm</robotNamespace>
        <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
</gazebo>
```
