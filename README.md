SLAM Object Search
---

Common commands:

1. To try motion model (some commands or code for motionModel.cpp may be deprecated and incorrect now.):

* Launch in turtlebot3\_world:

```
roslaunch  turtlebot3_gazebo turtlebot3_world_test.launch
```

Call sequence: turtlebot3\_world\_test.launch >> turtlebot3\_world.world, turtlebot, motionModel.cpp OR

Call sequence: turtlebot3\_empty\_world\_test.launch >> empty.world, turtlebot, motionModel.cpp

Location of motionModel.cpp is: ```catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/src```

* Launch in an empty world:

```
roslaunch  turtlebot3_gazebo turtlebot3_world.launch
```
Similar call sequence but world file comments out "turtelbot3_world" so that empty world opens up.


2. To test april tags:
```
roslaunch  turtlebot3_gazebo turtlebot3_empty_world.launch
```

Call sequence: turtlebot3_empty_world.launch >> empty.world >> models/box_color/model.sdf >> apriltag.dae >> marker.png

3. To try Lidar only in EKF code (default is without motion):

```
roslaunch turtlebot3_gazebo turtlebot3_empty_world_lidarTest.launch
```
Call sequence: turtlebot3_empty_world_lidarTest.launch >> empty\_lidarTest.world, turtlebot, ekfLidarTest.cpp

Location of ekfLidarTest.cpp is: ```catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/src```


4. To try Full EKF code with single obstacle:

```
roslaunch turtlebot3_gazebo turtlebot3_empty_world_ekf.launch
```
Call sequence: turtlebot3_empty_world_lidarTest.launch >> empty\_lidarTest.world, turtlebot, ekfTest.cpp

Location of ekfTest.cpp is: ```catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/src```

5. Aruco Markers

```
roslaunch turtlebot3_gazebo turtlebot3_empty_world_arucoTest.launch
roslaunch aruco_ros marker_publisher.launch ref_frame:=base_footprint
rostopic echo /aruco_marker_publisher/markers
```

Location of model and SDF files for markers: catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/aruco_visual_marker_X/ --- include these as a URI in the world files
Location of aruco source repo/project: catkin_ws/src/aruco_ros/aruco_ros
