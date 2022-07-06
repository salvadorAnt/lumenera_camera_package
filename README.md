# Lumenera Camera ROS Package
ROS package for Lumenera Cameras.

# How to install it
- First, download `lucam-sdk` from (Lumenera website)[www.lumenera.com]. Then install it according to the README file that is with it. Make sure to make all examples, especially OpenCV example that is explained it the readme file of the SDK.

- Second, clone this github repository in your `src` folder of your `catkin` workstation.

- Third, copy two folders `lib` and `include` from where you installed the SDK, they are in `api` folder. Then past those two folders into the package folder: `lumenera_camera_package`.

- Fourth, go to catking workstation and then use `catkin_make`  to build the package.

# How to use it
- First, in a terminal run `roscore`.
- Second, in other terminal, use this while you are in your catkin workstations `./devel/lib/lumenera_camera_package/lumenera_camera_package_node `.

# Outputs
For each camera, it produces one topic with the name `/lumenera_camera_package/i`, `i` is camera number. For example, for the first camera the topic will be `/lumenera_camera_package/1`

You can subscribe to topics and use the published images.

For example, in the following image, I connected two lumenera cameras to my PC and I executed the ROS node. It published two topics, one for each camera. The frame rate for each one of the two is near 37.
![Lumenera Cameras ROS package](/imgs/demo-0.png?raw=true)


Also, in the following image, I connected one lumenera camera to my PC and I executed the ROS node. It published one topic. The frame rate for the camera is near 90 frames per second.
![Lumenera Cameras ROS package](/imgs/demo-1.png?raw=true)

# NOTICE
It is not an official ROS package by Lumenera camera or SDK provider.
I tested it on Ubuntu 18.04 with ROS Melodic. I have not tested it on other environments. Maybe you will be required to do some adjustments.
