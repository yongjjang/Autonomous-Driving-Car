# Turtlebot Camera Description File

## Finally, Turtlebot get Two Eyes..!

## Usage
```Shell
$ sudo mv /opt/ros/kinetic/share/kobuki_description/urdf/kobuki_gazebo.urdf.xacro /opt/ros/kinetic/share/kobuki_description/urdf/origin.kobuki_gazebo.urdf.xacro
$ sudo mv /opt/ros/kinetic/share/kobuki_description/urdf/kobuki.urdf.xacro /opt/ros/kinetic/share/kobuki_description/urdf/origin.kobuki.urdf.xacro

$ sudo mv kobuki_gazebo.urdf.xacro /opt/ros/kinetic/share/kobuki_description/urdf/
$ sudo mv kobuki.urdf.xacro /opt/ros/kinetic/share/kobuki_description/urdf/

```

## Advertise Topic name
- Right Camera :
  - /my_right_camera/rgb/image_raw
- Left Camera :
  - /my_left_camera/rgb/image_raw

## alias roslaunch deu_car car_test_map.launch
```Shell
$ gedit .bashrc

// ADD this line
alias ros_car='roscd deu_car && source gazebo_env.sh && roslaunch deu_car car_test_map.launch'

$ source .bashrc
$ ros_car

```
