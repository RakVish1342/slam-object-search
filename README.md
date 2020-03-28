SLAM Object Search
---

Common commands:

1. To try motion model:

* Launch in turtlebot3\_world:

```
roslaunch  turtlebot3_gazebo turtlebot3_world_test.launch
```

Call sequence: turtlebot3\_world.launch >> turtlebot3\_world.world, turtlebot, motionModel.cpp
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

