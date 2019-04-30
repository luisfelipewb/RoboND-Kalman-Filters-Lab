#RoboND-Kalman-Filters-Lab

This project is part of the Robotics Software Engineering Nanodegree from Udacity. 
The goal is to experiment with the extended kalman filters in the ROS envrionment. 


## Turtlebot Gazebo Package
Provide the environment and robot simulation
http://wiki.ros.org/turtlebot_gazebo

Installation:
```bash
cd $YOURPATH/catkin_ws/src
git clone https://github.com/turtlebot/turtlebot_simulator
cd .. 
source devel/setup.bash
rosdep -i install turtlebot_gazebo
catkin_make
source devel/setup.bash
```

Launch
```bash
roslaunch turtlebot_gazebo turtlebot_world.launch
```

## Robot Pose EKF Package
Sensor fusion subscribed to odom and imu and publisher of odom_combined using EKF
http://wiki.ros.org/robot_pose_ekf

Installation:
```bash
cd $YOURPATH/catkin_ws/src
git clone https://github.com/udacity/robot_pose_ekf 
#Adjust robot_pose_ekf.launch file
cd ..
catkin_make
source devel/setup.bash
```

Launch
```bash
roslaunch robot_pose_ekf robot_pose_ekf.launch
```


## Odometry to Trajectory Package
Plot path based on odometry output

Installation:
```bash
cd $YOURPATH/catkin_ws/src
git clone https://github.com/udacity/odom_to_trajectory
cd ..
catkin_make
source devel/setup.bash
```

Launch
```
roslaunch odom_to_trajectory create_trajectory.lauch
```

## Turtlebot Teleop Package
Control turtlebot from keyboard
http://wiki.ros.org/turtlebot_teleop

Installation:
```bash
cd $YOURPATH/catkin_ws/src
git clone https://github.com/turtlebot/turtlebot
cd ..
source devel/setup.bash
rosdep -i install turtlebot_teleop
catkin_make
source devel/setup.bash
```

Launch:
```bash
roslaunch turtlebot_teleop keyboard_teleop.launch
```



### Useful commands
```bash
rostopic list
rosbag record /topic
rosbag play bagfile.bag
rosrun rqt_graph rqt_graph
rosrun rviz rviz -d configfile.rviz
rosrun rqt_multiplot rqt_multiplot
```
