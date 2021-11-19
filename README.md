# HandEyeCalib-ROS

## Introduction
This repo provides a detailed tutorial to eye-in-han calibration, eye-to-hand calibration, and downstream grasp execution. The ultimate goal is to build a handy toolbox for either desktop or mobile grasping/manipulation. We will update this repo from time to time to make sure the latest progress is included. Feel free to leave any questions or comments, we are willing to discuss.

## Progress
- [X] Eye-hand calibration with built-in aruco detecter (for opencv version after 3.2.0)  
- [X] Eye-in-hand calibration with `aruco_ros` package (for ubuntu 18.04 with default opencv 3.2.0)
- [ ] Eye-to-hand calibration with `aruco_ros` package (for ubuntu 18.04 with default opencv 3.2.0)
- [ ] open-loop planar grasp execution
- [ ] open-loop 6-DOF grasp execution
- [ ] close-loop planar grasp execution
- [ ] close-loop 6-DoF grasp execution

### Setup
* system: Ubuntu 18.04
* ROS version: melodic
* OpenCV version: 3.2.0
* Robot: KINOVA Gen3 & Gen3 Lite
* Camera: Intel Realsense D435i

### Pre-requisites
* [moveit_calibration](https://github.com/ros-planning/moveit_calibration): master branch  
* [rviz_visual_tools](https://github.com/PickNikRobotics/rviz_visual_tools/tree/master): master branch  
* [geometric_shapes](https://github.com/ros-planning/geometric_shapes/tree/melodic-devel): melodic-devel  
* [moveit_visual_tools](https://github.com/ros-planning/moveit_visual_tools): melodic-devel  

Make sure you git clone the right branch and put all the packages under the same ROS worksapce.

## Eye-hand calibration with built-in aruco detecter (for opencv version after 3.2.0)  

| **Warning to Melodic users** |
| --- |
| OpenCV 3.2, which is the version in Ubuntu 18.04, has a buggy ArUco board pose detector. Do not expect adequate results if you are using an ArUco board with OpenCV 3.2. Jump directly to eye-hand calibration with `aruco_ros` package. **We got your back!**|

Moveit provides a convenient eye-hand calibration package. It's a good idea to follow this [tutorial](https://ros-planning.github.io/moveit_tutorials/doc/hand_eye_calibration/hand_eye_calibration_tutorial.html) step by step.

**put tutorial image image here**

The whole proces includes: start your camera, set tag parameters, print Aruco tag, set context configuration, task at least 5 samples, solve camera pose, save cameras poses & joint states.It uses opencv built-in aruco detector, which is buggy in opencv 3.2.0. We actually tested on opencv 3.2.0 and it gave very bad results.   

**Take care of which sensor frame you are using.** `Moveit calibration` uses the right-down-forward standard as specified [here](https://www.ros.org/reps/rep-0103.html). You might get totally different frame coordinates before and after loading camera into `HandEyeCalibration` For Intel Realsesne D435i, we found the frame coordinates of `camea_color_frame` has x axis pointing outwards while it is forced to z axis pointing outwards after being loaded. For this reason, we suggest using `camera_color_optical_frame`. Both `camea_color_frame` and `camera_color_optical_frame` do not locate at camera center since RGB camera is on rightmost side of D534i.

**put image to context confiruation, RGB**

## Eye-in-hand calibration with `aruco_ros` package (for ubuntu 18.04 with default opencv 3.2.0)
Since the built-in aruco detecter is buggy in opencv 3.2.0, we instead use `aruco_ros` package. This section details the eye-in-hand calibration with `aruco_ros` package. First, in addition to the standard pre-requisites you may have downloaded, git clone `aruco_ros` with the link below:  

* [aruco_ros](https://github.com/pal-robotics/aruco_ros/tree/melodic-devel): melodic-devel

**put link to aruco tag generation**

This package supports single or double aruco tags detection. After compiling the package, generate a aruco tag with appropriate size via this [link](https://tn1ck.github.io/aruco-print/) and print it out. Then edit marker configuration:

```
cd PATH/TO/YOUR/WORKSPACE/src/aruco_ros/aruco_ros/launch
sudo gedit single.launch
```

Here we use single aruco tag with markID as 9, marksize as 0.160m, camera_frame as `camera_color_optical_frame`. The launch file is configured as below:

**put image to launch file configuration**

Run the command below to test if printed marker is detected successfully:

```
roslaunch realsense2_camera rs_camera.lanuch  # launch your camera
roslaunch aruco_ros sinlge.launch  # launch your marker detecter
rosrun image_view image_view image:=/aruco_single/result  # visualize detected marker
```
You should be able to see something like:

**put image to detected marker**

Then, you may continues to launch you robot. In our case, we run:

```
roslaunch kortex_driver kortex_driver
```

For some one who is using the same robot or camera, drivers can be downloaded and installed as below:

* [ros_kortex](https://github.com/Kinovarobotics/ros_kortex/tree/melodic-devel): melodic-devel
* [realsense-ros](https://github.com/IntelRealSense/realsense-ros): development

After launching the camera, aruco and robot successfully, you can load `HandEyeCalibration` in RViz as below:

**add handeye panel**

In `HandEyeCalibration`, go to `Context`. Set the parameters as below:

* Sensor configuration: Eye-in-hand
* Sensor frame: camera_color_optical_frame
* End-effector frame: tool_frame
* Robot base frame: base_link

Go to `Calibrate`. Now, you may start to move the camera around the take some samples and solve for the transform. **Be sure to include some rotation between each pair of poses, and don’t always rotate around the same axis–at least two rotation axes are needed to uniquely solve for the calibration**.  Calibration will be performed automatically, and updated each time a new sample is added. It needs at least 5 sample, but the more the better. You might need to install extra packages for solver if error pops out. Press the `save joint states` and `save camera pose` upon getting the result.

For the record, our tf tree is as below:

**tf tree**

## Eye-to-hand calibration with `aruco_ros` package (for ubuntu 18.04 with default opencv 3.2.0)
TBA
## Open-loop planar grasp execution
TBA
## Open-loop 6-DOF grasp execution
TBA
## Close-loop planar grasp execution
TBA
## Close-loop 6-DoF grasp execution
TBA









