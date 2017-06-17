# realsense_sp
ROS package for the RealSense SP middleware under Intel Euclid.

# Quick hack by RoboJay.us
Hacked in occupancy map code from here:
https://github.com/IntelRealSense/realsense_samples_ros/blob/kinetic-devel/realsense_ros_slam/src/slam_nodelet.cpp

To experiment with...

cd /intel/euclid/euclid_ws/src
git clone https://github.com/robojay/realsense_sp
cd ..
catkin_make
source devel/setup.bash

In one terminal:
roslaunch realsense_camera lr200m_nodelet_default.launch

In original terminal (path = /intel/euclid/euclid_ws):
roslaunch realsense_sp sp.launch

In a third terminal:
rviz
(add by message topic, find map, add it, should see an occupancy map now)


