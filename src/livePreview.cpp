#define LUMENERA_LINUX_API
#undef LUMENERA_MAC_API
#undef LUMENERA_WINDOWS_API

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>

#include <lucamapi.h>
#include <lucamerr.h>
#include "Camera.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using std::cout;
using std::endl;

int main( int argc, char** argv )
{
  // ROS node name
  ros::init(argc, argv, "lumenera_camera_node");
	
  // ros node handle
  ros::NodeHandle nh;

  // ros rate Hz
  ros::Rate rate(90);

  // print start of node
  ROS_INFO("Node: [lumenera_camera_node] has been started.");


  // frame id
  int frame_id = 0;

  BYTE *imagePtr;
  int frameSize;
  bool displaySobelImage = false;
  if (argc > 1) {
    displaySobelImage = true;
  }


  int numCameras = LucamNumCameras();
  
  ROS_INFO_STREAM("Found " << numCameras << " camera" << std::string((1 == numCameras) ? "" : "s") << endl);

  if (0 == numCameras) {
    return 0;
  }

  // For now, limit ourselves to 1 camera
  numCameras = std::max(numCameras, 1);
  std::vector<Camera> cameras(numCameras);

  // Initialize each camera
  for(size_t i=0; i < cameras.size(); i++) {
    ROS_INFO_STREAM("Initializing Camera #" << i+1 << endl);
    cameras[i].init(i+1, displaySobelImage ? "Sobel" : "");
  }

  // Start streaming on each camera
  for(size_t i=0; i < cameras.size(); i++) {
    ROS_INFO_STREAM("Starting streaming on Camera #" << i+1 << endl);
    cameras[i].startStreaming();
  }

  //
  const int scale  = 1;
  const int delta  = 0;
  const int ddepth = CV_16S;

  // image publisher 
  // for each camera create an publisher
  std::vector<ros::Publisher> publishers;
  for (size_t i = 0; i < cameras.size(); i++) {
    char topic_name[200];
    sprintf(topic_name, "/lumenera_camera_package/%d", i + 1);
    publishers.push_back(nh.advertise<sensor_msgs::Image>(topic_name, 10));
  }

  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;

  while (ros::ok()) {
    // Grab and display a single image from each camera
    for (size_t i = 0; i < cameras.size(); i++) {

      imagePtr = cameras[i].getRawImage();

      frameSize = cameras[i].getFrameSize();

      cameras[i].createRGBImage(imagePtr,frameSize);

      unsigned char* pImage = cameras[i].getImage();
      if (NULL != pImage) {

        Mat image(cameras[i].getMatSize(), CV_8UC3, pImage, Mat::AUTO_STEP);

        cameras[i].releaseImage(); // release asap

        //cvtColor(image, image, CV_BGR2RGB,3);

        // publish on ROS topic
        std_msgs::Header header; // empty header
        header.seq = frame_id; // user defined counter
        header.stamp = ros::Time::now(); // time
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);
        img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
        publishers[i].publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);

        // Show the actual image
        //resize(image, image, cameras[i].getDisplaySize());
        //imshow(cameras[i].unprocessedWindowName(), image );

      }
      
    }

    // increase frame Id
    frame_id = frame_id + 1;

    // wait for a key for 1 ms. If 'q' key pressed, quit program
    //int keyPressed = waitKey(1);
    //if (keyPressed == 'q') {
    //  ROS_INFO_STREAM("Quitting..." << endl);
    //  break; 
    //}
    //rate.sleep();
    
  }

  
  for(size_t i=0; i < cameras.size(); i++) {
    cameras[i].stopStreaming();
  }

  ROS_INFO("Node: [lumenera_camera_node] has been Ended.");

  return 0;
}
