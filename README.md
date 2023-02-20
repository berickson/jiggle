<h1>Jiggle SLAM</h1>
<img src="media/blue-crash-avatar.jpg" style="float:right"/>


Jiggle Slam is a ROS2 package targeting services for mobile robots with 2D lidar.  It targets high speed robots with modest processing power where there are unique challenges like fast response times and severe lidar warping due to dynamic robot movement.

Warning: Jiggle SLAM is in early development, is only partially implemented in and definitely isn't ready for your robot.

<a href='https://www.youtube.com/watch?v=_Qd8YkFvXkg'> <img src="https://i.ytimg.com/vi/_Qd8YkFvXkg/maxresdefault.jpg" width="50%"/><br/>video demonstrates process_bag from Humble branch</a>
<br/>
<br/>

Installing g2o
```
conda install -c conda-forge g2o
```

Converting ROS bags to ROS 2
```
pip install rosbag
rosbag-convert <bag name>
```

Useful commands
```
colcon build --packages-select jiggle --cmake-args -DCMAKE_BUILD_TYPE=Release

rm -rf ~/jiggle_ws/data/a3-lab-bar-kitchen-2020-07-14-21-38-09.out.bag && ros2 run jiggle process_bag
```

<h2>ROS2 Packages</h2>

**lidar_odom**

Provides live odometry from 2d lidar.  Publishes the TF transformation lidar_odom->lidar_odom_laser.


<h2>Feature Status</h2>

| Module | Feature |  Implemented |
|-|-|-|
| lidar_odom | | 
|| 2-D Scan Matching | X 
|| Dewarp | X 
|| Config Dewarp | X 
|| Publish to lidar TF | X 
|| Publish Untwisted Scans | X
|| Skip computing pointcloud2 if there are no subscribers | X
|| Config TF Frames
|| Integrate with ODOM speed for initial match guess
|| Detect Slippage (ODOM/LIDAR/IMU Discrepencies)
|| Prune Scans for Performance, remove interior points on straight lines
|| Merge frames
|| Option to disable null map->lidar_odom transform
| processs_bag ||Needs Humble port
|| Loop Closure
|| Publish map->lidar_odom to TF





