# ros-blue

some ROS nodes for the Beaglbone Blue

## Nodes overview

* differential drive controller node using the onboard motor ports
* publisher for IMU messages from MPU9250



## IMU node

<<<<<<< HEAD
rosrun imu_filter_madgwick imu_filter_node
rosrun tf static_transform_publisher 0.0 0.0 0.0 0 0 0 map imu_link 10

red - x green - y blue -z 
=======
Start the IMU node

`rosrun ros-blue imu_pub_node`

### Visualizing IMU with rviz
A static transformation is required:

`rosrun tf static_transform_publisher 0.0 0.0 0.0 0 0 0 map imu_link 10`

The Beaglebone Blue has no display port. So for visualization an aditional system is required.

`export ROS_MASTER_URI=http://rosbot:11311`

Starting rviz after exporting the MASTER usxbrix

`rviz`

### optional IMU tools
`sudo apt-get install ros-melodic-imu-tools`

`rosrun imu_filter_madgwick imu_filter_node`
>>>>>>> 46213feb59fdd0f1c26bfcc1fde00a322715c104
