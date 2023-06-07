# robot4ws_slam

## Cartographer

Cartographer is a system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations.

Cartographer subscribes to _/clock_, _/tf_, _/Archimede/scan_ (if used), _/Archimede/odom_ (if used), _/Archimede/imu_ (if used, necessary for 3D SLAM) and _/Archimede/d435i\_camera/depth/color/points_ (if used).

Cartographer publishes to _/tf_, _/rosout_, _/submap\_list_, _/trajectory\_node\_list_(array containing ordered xyz of the robot), _/scan\_matched\_points2_, _/landmark\_poses\_list_, _/constraint\_list_. \
In 2D mode, also _/cartographer\_occupancy\_grid\_node_ runs publishing nav_msgs/OccupancyGrid messages to _/map_, generating an occupancy grid. \
In 3D mode, only x-ray submaps are published.

### Installation

Install Cartographer by executing the following code.

``` 
sudo apt-get update
sudo apt-get install ros-melodic-cartographer*
```

### Usage

Two different approaches are implemented:

* 2D SLAM, online and offline;

* 3D SLAM, online and offline.

#### 2D SLAM
 It performs simultaneous localization and mapping in a 2D map. 
 
 Our default configuration exploits the laser scan, pointcloud and odometry. 

To perform 2D slam online, execute within a terminal:

```
source ~/catkin_ws/devel/setup.bash
roslaunch robot4ws_slam archimede_cartographer_2dslam.launch
```
An rviz is also automatically launched, with an appropriate configuration, to track the mapping process' progress.

To perform 2D slam from a recorded bag, execute:

```
source ~/catkin_ws/devel/setup.bash
roslaunch robot4ws_slam archimede_cartographer_2dslam_offline.launch bag_filenames:=/path/to/your/bag.bag
```

#### 3D SLAM
 It performs simultaneous localization and mapping in a 3D map, with obstacles depicted in a 2D fashion. 
 
 Our default configuration exploits just the laser scan, point cloud, imu and odometry.

To perform 3D SLAM online, execute within a terminal:

```
source ~/catkin_ws/devel/setup.bash
roslaunch robot4ws_slam archimede_cartographer_3dslam.launch
```
An rviz is also automatically launched, with an appropriate configuration, to track the mapping process' progress.

To finish the first trajectory. No further data will be accepted on it.
```
rosservice call /finish_trajectory 0
```

Then ask Cartographer to serialize its current state.
```
rosservice call /write_state filename: '${HOME}/Downloads/b3-2016-04-05-14-14-00.bag.pbstream'
```

To perform 3D SLAM from a recorded bag, execute:

```
source ~/catkin_ws/devel/setup.bash
roslaunch robot4ws_slam archimede_cartographer_3dslam_offline.launch bag_filenames:=/path/to/your/bag.bag
```
The trajectory and .pbstream file will be automatically stopped and created respectively once the bag will have finished playing.

#### 2D Pure Localization
Given a map previously recorded by cartographer (in form of .pbstream file), it can also perform just localization within the 2D map.

To do it online:

```
roslaunch robot4ws_slam archimede_cartographer_2dlocalization.launch \
   load_state_filename:=/path/to/pbstream/file.pbstream 
```

If you are using a bag file instead:

```
roslaunch robot4ws_slam archimede_cartographer_2dlocalization_offline.launch \
   load_state_filename:=/path/to/pbstream/file.pbstream \
   bag_filename:=/path/to/bag/file.bag
```

#### 3D Pure Localization
Given a map previously recorded by cartographer (in form of .pbstream file), it can also perform just localization within the map.

To do it online:

```
roslaunch robot4ws_slam archimede_cartographer_3dlocalization.launch \
   load_state_filename:=/path/to/pbstream/file.pbstream 
```

If you are using a bag file instead:

```
roslaunch robot4ws_slam archimede_cartographer_3dlocalization_offline.launch \
   load_state_filename:=/path/to/pbstream/file.pbstream \
   bag_filename:=/path/to/bag/file.bag
```


#### 3D Point Cloud Reconstruction
As sensor data come in, the state of a SLAM algorithm such as Cartographer evolves to stay the current best estimate of a robot’s trajectory and surroundings. The most accurate localization and mapping Cartographer can offer is therefore the one obtained when the algorithm finishes. Cartographer can serialize its internal state in a .pbstream file format which is essentially a compressed protobuf file containing a snapshot of the data structures used by Cartographer internally.

To run efficiently in real-time, Cartographer throws most of its sensor data away immediately and only works with a small subset of its input, the mapping used internally (and saved in .pbstream files) is then very rough. However, when the algorithm finishes and a best trajectory is established, it can be recombined a posteriori with the full sensors data to create a high resolution map.

Cartographer makes this kind of recombination possible using archimede_cartographer_3dassets_writer. The assets writer takes as input
* the original sensors data that has been used to perform SLAM (in a ROS .bag file),
* a cartographer state captured while performing SLAM on this sensor data (saved in a .pbstream file),
* the sensor extrinsics (i.e. TF data from the bag or an URDF description),
* a pipeline configuration, which is defined in a .lua file.

The assets writer runs through the .bag data in batches with the trajectory found in the .pbstream. The pipeline can be used to color, filter and export SLAM point cloud data into a variety of formats. 

To use it, launch cartographer offline with a previously recorded bagfile. This will create a .pbstream file automatically once ended. Launch then archimede_cartographer_3dassets_writer:

```
roslaunch robot4ws_slam archimede_cartographer_3dassets_writer.launch bag_filenames:=/path/to/bag/file.bag pose_graph_filename:=/path/to/pbstream/file.pbstream
```

Archimede_cartographer_3dassets_writer gives you in output the maps along XY axes, XZ axes and YZ axes, each of them both in grayscale and color. Moreover, the processed point clouds are stored in .ply and .pcd format. 

To create a octree from the point cloud, install first point_cloud_viewer from its [repository](https://github.com/cartographer-project/point_cloud_viewer). Once installed, you can generate an octree by running from point_cloud_viewer directory(Set as output directory a new empty directory, since a lot of files will be created):

```
target/release/build_octree /path/to/points.ply--output-directory /path/to/output/directory/
```

To launch sdl viewer and visualize the created map:
```
target/release/sdl_viewer /path/to/directory/
```


### Further Information

To change SLAM configuration, by adding on removing some input sensors or to tune the algorithm, edit _/configuration\_files/archimede\_2d.lua_ for 2D SLAM or _/configuration\_files/archimede\_3d.lua_ for 3D SLAM. More parameters to be edited can be found in other _.lua_ files in the _/configuration\_files/backup_ directory. Whereas you find a parameter you'd rather change, copy and paste it inside the _archimede\_2d.lua_ or _archimede\_3d.lua_ configuration file, then edit it appropriately.

Launch Cartographer after launching the rover and its sensors, otherwise an error will occur.

Note: for optimal performance, keep the robot still for 5-10 seconds before moving it. It enhances SLAM abilities.

#### Map Saving

To save a map in .pgm format from /map topic:
```
rosrun map_server map_saver -f map_file_name map:=/map
```
Note that also a .yaml header file will be saved.

## OctoMap
Octomap is an open source library for generating volumetric 3D environment models from sensor data. Octomap is a mapping approach to build map made of 3D voxels.

Octomap server subscribes to _/clock_, _/tf_, and _/Archimede/d435i\_camera/depth/color/points_.

Octomap server publishes to _/rosout_, _/projected\_map_ (downprojected 2D occupancy map from the 3D map), _/occupied\_cells\_vis\_array_ (all occupied voxels as "box" markers for visualization in RViz), _/free\_cells\_vis\_array_, _/octomap\_point\_cloud\_centers_ (the centers of all occupied voxels as point cloud, useful for visualization), _/octomap\_binary_ (the complete maximum-likelihood occupancy map as compact OctoMap binary stream, encoding free and occupied space. The binary message only distinguishes between free and occupied space but is smaller) and _/octomap\_full_ (The complete maximum-likelihood occupancy map as compact OctoMap binary stream, encoding free and occupied space. The full message contains the complete probabilities and all additional data stored in the tree).

### Installation

Install OctoMap library
```
sudo apt-get install ros-melodic-octomap*
```

### Usage
Octomap is not a SLAM algorithm, therefore it needs either odometry or a SLAM process to be running. We have implemented two different versions. The former takes in input just the odometry, the latter it also exploits SLAM performed by cartographer.

To map the environment with octomap based on odometry, run:
```
roslaunch robot4ws_slam archimede_octomap.launch
```

To map the environment with octomap based on odometry and SLAM, run:
```
roslaunch robot4ws_slam archimede_slam_and_voxel_mapping.launch
```



### Further Information

#### Map Saving

To request a compressed binary octomap via service call and save it to octomap_map.bt:
```
rosrun octomap_server octomap_saver octomap_map.bt
```

To request a full probability octomap via service call and save it to octomap_map.ot:
```
rosrun octomap_server octomap_saver -f octomap_map.ot
```

To save a 2D map in .pgm format from /projected_map topic:
```
rosrun map_server map_saver -f map_file_name map:=/projected_map
```
Note that also a .yaml header file will be saved.