# Lumenera Camera ROS Package
ROS package for Lumenera Cameras

# How to install it
- First, download `lucam-sdk` from (Lumenera website)[www.lumenera.com]. Then install it according to the README file that is with it.

- Second, clone this repository in your `src` folder of your `catkin` workstation.

- Third, copy two folders `lib` and `include` from where you installed the SDK. Then past those two folders into the package folder: `lumenera_camera_package`.

- Fourth, go to catking workstation and then use `catkin_make`  to build the package.

# How to use it
- First, run `roscore`.
- Second, use thin in your terminal while you are in your catkin workstations `./devel/lib/lumenera_camera_package/lumenera_camera_package_node `.

# Outputs
For each camera, it produced one topic with the name `/lumenera_camera_package/i`, `i` is camera number. For example, for the first camera the topic will be `/lumenera_camera_package/1`

You can subscribe to topics and use the published images.

For example, it the following image, I connected to lumenera cameras to my PC and I executed the ROS node. It published two topic, one for each camera. The frame rate for each one of them is near 26.
![Lumenera Cameras ROS package](/imgs/demo.png?raw=true)

# NOTICE
I tested it on Ubuntu 18.04 with ROS Melodic. I have not tested it on other environments. Maybe you will be required to do some adjustments.
