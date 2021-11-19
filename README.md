# HandEyeCalib-ROS

## Introduction
This repo provides a detailed tutorial to eye-in-han calibration, eye-to-hand calibration, and downstream grasp execution. The ultimate goal is to build a handy toolbox for either desktop or mobile grasping/manipulation. We will update this repo from time to time to make sure the latest progress is included. Feel free to leave any questions or comments, we are willing to discuss.

## Progress
- [ ] Eye-hand calibration with built-in aruco detecter (for opencv version after 3.2.0)  
- [ ] Eye-in-hand calibration with `aruco_ros` package (for ubuntu 18.04 with default opencv 3.2.0)
- [ ] Eye-to-hand calibration with `aruco_ros` package (for ubuntu 18.04 with default opencv 3.2.0)
- [ ] open-loop planar grasp execution
- [ ] open-loop 6-DOF grasp execution
- [ ] close-loop planar grasp execution
- [ ] close-loop 6-DoF grasp execution

## Eye-hand calibration with built-in aruco detecter (for opencv version after 3.2.0)  

| **Warning to Melodic users** |
| --- |
| OpenCV 3.2, which is the version in Ubuntu 18.04, has a buggy ArUco board pose detector. Do not expect adequate results if you are using an ArUco board with OpenCV 3.2. Jump directly to eye-hand calibration with `aruco_ros` package. **We got your back!**|

Moveit provides a convenient eye-hand package. It's a good idea to follow its [tutorial](https://ros-planning.github.io/moveit_tutorials/doc/hand_eye_calibration/hand_eye_calibration_tutorial.html) step by step. Before that, some packages are needed.

### Setup
* system: Ubuntu 18.04
* ROS version: melodic
* Robot: KINOVA Gen3 & Gen3 Lite
* Camera: Intel Realsense D435i

### Pre-requisites
* [moveit_calibration](https://github.com/ros-planning/moveit_calibration): master branch  
* [rviz_visual_tools](https://github.com/PickNikRobotics/rviz_visual_tools/tree/master): master branch  
* [geometric_shapes](https://github.com/ros-planning/geometric_shapes/tree/melodic-devel): melodic-devel  
* [moveit_visual_tools](https://github.com/ros-planning/moveit_visual_tools): melodic-devel  

Make sure you git clone the right branch and put all the packages under the same ROS worksapce.

### Calibration
Follow the [tutorial](https://ros-planning.github.io/moveit_tutorials/doc/hand_eye_calibration/hand_eye_calibration_tutorial.html) step by step. The whole proces includes: start your camera, set tag parameters, print Aruco tag, set context configuration, task at least 5 samples, solve camera pose, save cameras poses & joint states.It uses opencv built-in aruco detector, which is buggy in opencv 3.2.0. We actually tested on opencv 3.2.0 and it gave very bad results.   

**Take care of which sensor frame you are using.** `Moveit calibration` uses the right-down-forward standard as specified [here](https://www.ros.org/reps/rep-0103.html). You might get totally different frame coordinates before and after loading camera into `HandEyeCalibration` For Intel Realsesne D435i, we found the frame coordinates of `camea_color_frame` has x axis pointing outwards while it is forced to z axis pointing outwards after being loaded. For this reason, we suggest using `camera_color_optical_frame`. Both `camea_color_frame` and `camera_color_optical_frame` do not locate at camera center since RGB camera is on rightmost side of D534i.

## Eye-in-hand calibration with `aruco_ros` package (for ubuntu 18.04 with default opencv 3.2.0)
Since the built-in aruco detecter is buggy in opencv 3.2.0, we instead use `aruco_ros` package. This section details the eye-in-hand calibration with `aruco_ros` package.

### Setup
* system: Ubuntu 18.04
* ROS version: melodic
* OpenCV version: 3.2.0
* Robot: KINOVA Gen3 & Gen3 Lite
* Camera: Intel Realsense D435i

### Pre-requisites





