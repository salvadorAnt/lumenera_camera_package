#define LUMENERA_LINUX_API
#undef LUMENERA_MAC_API
#undef LUMENERA_WINDOWS_API

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <thread>

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

void publish_camera_on_topic(std::vector<Camera> cameras, const std::vector<ros::Publisher> publishers, const int camera_index)
{ 
  ROS_INFO_STREAM( "Start thread for camera " << camera_index << "." << std::endl);

  int frameSize;
  BYTE *imagePtr;

  // frame id
  int frame_id = 0;

  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;

  while (ros::ok() && ros::master::check()) {
    // Grab and display a single image from each camera
    
    imagePtr = cameras[camera_index].getRawImage();

    frameSize = cameras[camera_index].getFrameSize();

    cameras[camera_index].createRGBImage(imagePtr,frameSize);

    unsigned char* pImage = cameras[camera_index].getImage();
    if (NULL != pImage) {

      Mat image(cameras[camera_index].getMatSize(), CV_8UC3, pImage, Mat::AUTO_STEP);

      // release asap
      cameras[camera_index].releaseImage(); 

      //cvtColor(image, image, CV_BGR2RGB,3);

      // publish on ROS topic
      std_msgs::Header header; // empty header
      header.seq = frame_id; // counter for frame rate
      header.stamp = ros::Time::now(); // time
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);
      img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
      publishers[camera_index].publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);

    }

    // increase frame Id
    frame_id = frame_id + 1;
  }

  ROS_INFO_STREAM( "ROS closing for thread of camera " << camera_index << " recieved." << std::endl);
}

int main( int argc, char** argv )
{
  std::thread * th;

  // ROS node name
  ros::init(argc, argv, "lumenera_camera_node");
	
  // ros node handle
  ros::NodeHandle nh;

  // print start of node
  ROS_INFO("Node: [lumenera_camera_node] has been started.");

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
    cameras[i].init(i+1, "");
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
  for (int i = 0; i < cameras.size(); i++) {
    char topic_name[200];
    sprintf(topic_name, "/lumenera_camera_package/%d", i + 1);
    publishers.push_back(nh.advertise<sensor_msgs::Image>(topic_name, 10));
  }
  
  // work with each camera on a seprate thread
  std::vector<std::thread *> thread_vector;
  for(size_t camera_index=0; camera_index < cameras.size(); camera_index++) {
    th = new std::thread;
    *th = std::thread(publish_camera_on_topic, cameras, publishers, camera_index);
    thread_vector.push_back(th);
  }

  ros::spin();

  for(size_t camera_index=0; camera_index < cameras.size(); camera_index++) {
    thread_vector[camera_index]->join();
    delete thread_vector[camera_index];
  }


  for(size_t camera_index=0; camera_index < cameras.size(); camera_index++) {
    cameras[camera_index].stopStreaming();
  }

  ROS_INFO("Node: [lumenera_camera_node] has been Ended.");

  return 0;
}
