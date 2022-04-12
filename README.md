# HandEyeCalib-ROS

## Introduction
This repo provides a detailed tutorial to eye-in-han calibration, eye-to-hand calibration, and downstream grasp execution. The ultimate goal is to build a handy toolbox for either desktop or mobile grasping/manipulation. We will update this repo from time to time to make sure the latest progress is included. Feel free to leave any questions or comments, we are willing to discuss.

## Progress
- [X] Eye-hand calibration with built-in aruco detecter (for opencv version after 3.2.0)  
- [X] Eye-in-hand calibration with `aruco_ros` package (for ubuntu 18.04 with default opencv 3.2.0)
- [X] Eye-to-hand calibration with `aruco_ros` package (for ubuntu 18.04 with default opencv 3.2.0)
- [X] open-loop planar grasp execution
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


<p align="center">
  <img src="https://github.com/mkt1412/HandEyeCalib-ROS/blob/main/images/Screenshot%20from%202021-11-19%2010-03-58.png" width="500">
</p>

The whole process includes: start your camera, set tag parameters, print Aruco tag, set context configuration, take at least 5 samples, solve camera pose, save cameras poses & joint states. It uses opencv built-in aruco detector, which is buggy in opencv 3.2.0. We actually tested on opencv 3.2.0 and it gave very bad results.   

**Take care of which sensor frame you are using.** `Moveit calibration` uses the right-down-forward standard as specified [here](https://www.ros.org/reps/rep-0103.html). You might get totally different frame coordinates before and after loading camera into `HandEyeCalibration` For Intel Realsesne D435i, we found the frame coordinates of `camea_color_frame` has x axis pointing outwards while it is forced to z axis pointing outwards after being loaded. For this reason, we suggest using `camera_color_optical_frame`. Both `camea_color_frame` and `camera_color_optical_frame` are not located at camera center since RGB camera is on rightmost side of D534i.

<!-- ![D535i](https://www.intelrealsense.com/wp-content/uploads/2020/05/depth-camera-d435_details.jpg | width=100) -->
<p align="center">
  <img src="https://www.intelrealsense.com/wp-content/uploads/2020/05/depth-camera-d435_details.jpg" width="500">
</p>

## Eye-in-hand calibration with `aruco_ros` package (for ubuntu 18.04 with default opencv 3.2.0)
Since the built-in aruco detecter is buggy in opencv 3.2.0, we instead use `aruco_ros` package. This section details the eye-in-hand calibration with `aruco_ros` package. First, in addition to the standard pre-requisites you may have downloaded, git clone `aruco_ros` with the link below:  

* [aruco_ros](https://github.com/pal-robotics/aruco_ros/tree/melodic-devel): melodic-devel

This package supports single or double aruco tags detection. After compiling the package, generate a aruco tag with appropriate size via this [link](https://tn1ck.github.io/aruco-print/) and print it out. Then edit marker configuration:

```
cd PATH/TO/YOUR/WORKSPACE/src/aruco_ros/aruco_ros/launch
sudo gedit single.launch
```

Here we use single aruco tag with markID as 9, marksize as 0.160m, camera_frame as `camera_color_optical_frame`. The launch file is configured as below:

<p align="center">
  <img src="https://github.com/mkt1412/HandEyeCalib-ROS/blob/main/images/Screenshot%20from%202021-11-19%2016-07-03.png" width="800">
</p>

Run the command below to test if printed marker is detected successfully:

```
roslaunch realsense2_camera rs_camera.lanuch  # launch your camera
roslaunch aruco_ros sinlge.launch  # launch your marker detecter
rosrun image_view image_view image:=/aruco_single/result  # visualize detected marker
```
You should be able to see something like:

<p align="center">
  <img src="https://github.com/mkt1412/HandEyeCalib-ROS/blob/main/images/Screenshot%20from%202021-11-19%2015-11-12.png" width="500">
</p>

Then, you may continue to launch you robot. In our case, we run:

```
roslaunch kortex_driver kortex_driver
```

For some one who is using the same robot or camera, drivers can be downloaded and installed as below:

* [ros_kortex](https://github.com/Kinovarobotics/ros_kortex/tree/melodic-devel): melodic-devel
* [realsense-ros](https://github.com/IntelRealSense/realsense-ros): development

After launching the camera, aruco and robot successfully, you can load `HandEyeCalibration` in RViz as below:

<p align="center">
  <img src="https://github.com/mkt1412/HandEyeCalib-ROS/blob/main/images/Screenshot%20from%202021-11-19%2015-13-55.png" width="800">
</p>

In `HandEyeCalibration`, go to `Context`. Set the parameters as below:

* Sensor configuration: Eye-in-hand
* Sensor frame: camera_color_optical_frame
* End-effector frame: tool_frame
* Robot base frame: base_link

Go to `Calibrate`. Now, you may start to move the camera around the take some samples and solve for the transform. **Be sure to include some rotation between each pair of poses, and don’t always rotate around the same axis–at least two rotation axes are needed to uniquely solve for the calibration**.  Calibration will be performed automatically, and updated each time a new sample is added. It needs at least 5 sample, but the more the better. You might need to install extra packages for solver if error pops out. Press the `save joint states` and `save camera pose` upon getting the result.

For the record, our tf tree is as below:

<p align="center">
  <img src="https://github.com/mkt1412/HandEyeCalib-ROS/blob/main/images/Screenshot%20from%202021-11-19%2015-14-46.png" width="800">
</p>

## Eye-to-hand calibration with `aruco_ros` package (for ubuntu 18.04 with default opencv 3.2.0)
Eye-to-hand calibration is almost identical to eye-in-hand calibration. The only difference is where to attach the marker and sensor configuration in Moveit HandEye Calibration.

First, attach the marker to the end-effector (shown below), and move the end-effector into the camera's view.

<p align="center">
  <img src="https://github.com/mkt1412/HandEyeCalib-ROS/blob/main/images/653839573.jpg" width="400">
</p>

Then, in `HandEyeCalibration`, go to `Context`. Set the parameters as below:

* Sensor configuration: Eye-to-hand
* Sensor frame: camera_color_optical_frame
* End-effector frame: tool_frame
* Robot base frame: base_link

Your RViz should look like this:

<p align="center">
  <img src="https://github.com/mkt1412/HandEyeCalib-ROS/blob/main/images/Screenshot%20from%202022-04-12%2019-11-45.png" width="800">
</p>

The rest is the same as eye-in-hand calibration.

## Open-loop planar grasp execution
To grasp with 4-DoF (2.5d planar grasp), we modify [GGCNN](https://arxiv.org/abs/1804.05172) to fit our robot and setup (original implementation can be found [here](https://github.com/dougsm/ggcnn)). We choose GGCNN becuase it's light-weight and easy to deploy on any machine. 

To detect 2.5d planar grasp, run the following commands:

```
cd PATH/TO/HandEyeCalib-ROS/planar_grasp_ggcnn
roslaunch kortex_driver kortex_driver.launch # start robot arm driver
roslaunch YOUR_HAND_EYE_CALIBRATION.launch # hand-eye calibration from last step, not included here
```
The commands above first launches KINOVA Gen3/Gen3 Lite dirver and then adds the calibrated hand-eye transform to arm tf tree.  

Run modified ggcnn:
```
conda activate YOUR_GGCNN_ENV
python vis_ggcnn.py
```
After running *viz_ggcnn.py*, RGB stream with detected planar grasp will pop out. By default, only the best grasp is shown. Feel free to change `--n-grasps` to any number you like. Aside from detecting grasps, the code will also publish a rostopic `/ggcnn_grasp_pose`. This topic includes grasp center(point), quality, angle, length, and depth value. To execute published grasp on your robot, open another terminal, run:

```
python ggcnn_pickNplace_gen3lite_full_arm_movement.py
```
It subscribes to `/ggcnn_grasp_pose`, converts the received grasp from camera frame to robot frame, and executes the grasp.










## Open-loop 6-DOF grasp execution
TBA
## Close-loop planar grasp execution
TBA
## Close-loop 6-DoF grasp execution
TBA









