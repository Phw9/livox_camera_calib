# lidar_camera_calib
**lidar_camera_calib** is a robust, high accuracy extrinsic calibration tool between high resolution LiDAR (e.g. Livox) and camera in targetless environment. Our algorithm can run in both indoor and outdoor scenes, and only requires edge information in the scene. If the scene is suitable, we can achieve pixel-level accuracy similar to or even beyond the target based method.

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation) and its additional ROS pacakge:

```
    sudo apt-get install ros-XXX-cv-bridge ros-xxx-pcl-conversions
```

### 1.2 **Eigen**
Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page)

### 1.3 **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.4 **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html). (Our code is tested with PCL1.7)

## 2. Build
Clone the repository and catkin_make:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
source /opt/ros/(ros-version)/setup.sh
git clone this repo
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```
## 3. Run on your own sensor set
### 3.1 Record data
Record the point cloud and image with rosbag.
You need to record about 20 seconds of an environment that is not moving and has no dynamic objects.
```
rosbag record <topic_name1> <topic_name2> ...
```

### 3.2 Convert bag to pcd
Convert the data in the rosbag file to pcd and the image to (png or bmp) files.
**bag_to_pcd.launch file**
Change the file path and topic name.
Lidar points must be filtered by the FoV of the camera.
You need to use the min_angle, max_angle parameter to filter your horizon angle of pointcloud.
Blind is a parameter that removes surrounding lidar points with a radius of x(m).
```
roslaunch livox_camera_calib bag_to_pcd.launch
```

### 3.3 Modify the **calib.yaml**
Change the data path to your local data path.  
Provide the instrinsic matrix and distort coeffs for your camera.

### 3.4 Use multi scenes calibration
Change the params in **multi_calib.yaml**, name the image file and pcd file from 0 to (data_num-1).


## 4. Run 
### 4.1 Single scene calibration
```
roslaunch livox_camera_calib single_calib.launch
```

### 4.2 Multi scenes calibration
```
roslaunch livox_camera_calib multi_calib.launch
```
The projected images obtained by initial extrinsic parameters.
<div align="center">
    <img src="pics/initial_extrinsic.png" width = 100% >
    <font color=#a0a0a0 size=2>An example of multi scenes calibration. The projected image obtained by theinitial extrinsic parameters</font>
</div>
Rough calibration is used to deal with the bad extrinsic.
<div align="center">
    <img src="pics/after_rough_calib.png" width = 100% >
    <font color=#a0a0a0 size=2>The projected image obtained by the extrinsic parameters after rough calibration</font>
</div>
Then we finally get a fine extrinsic after final optimization.
<div align="center">
    <img src="pics/fine_extrinsic.png" width = 100% >
    <font color=#a0a0a0 size=2>The projected image obtained by the extrinsic parameters after fine calibration</font>
</div>

