//******************************************************************************
//
// Copyright (c) 2019-2020, TELEDYNE LUMENERA, a business unit of TELEDYNE
// DIGITAL IMAGING, INCORPORATED. All rights reserved.
//
// This program is subject to the terms and conditions defined in file 'LICENSE'
// , which is part of this Linux LuCam SDK package.
//
//******************************************************************************

//
// Provides a live preview of a camera, using OpenCV
// If you pass a command-line parameter (anything), then a
// live preview window with a Sobel filter applied will be created
//
// Note that this requires that you have g++, and OpenCV installed on your system
// Brute force way to ensure these are installed is to use:
// sudo apt-get install g++
// sudo apt-get install libopencv-*
//


#define LUMENERA_LINUX_API
#undef LUMENERA_MAC_API
#undef LUMENERA_WINDOWS_API

#include <iostream>
#include <vector>
#include <string>

#include <lucamapi.h>
#include <lucamerr.h>
#include "Camera.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace cv;
using std::cout;
using std::endl;

int main( int argc, char** argv )
{

  BYTE *imagePtr;
  int frameSize;
  bool displaySobelImage = false;
  if (argc > 1) {
    displaySobelImage = true;
  }


  int numCameras = LucamNumCameras();
  
  cout << "Found " << numCameras << " camera" << std::string((1 == numCameras) ? "" : "s") << endl;

  if (0 == numCameras) {
    return 0;
  }

  // For now, limit ourselves to 1 camera
  numCameras = std::max(numCameras, 1);
  std::vector<Camera> cameras(numCameras);

  // Initialize each camera
  for(size_t i=0; i < cameras.size(); i++) {
    cout << "Initializing Camera #" << i+1 << endl;
    cameras[i].init(i+1, displaySobelImage ? "Sobel" : "");
  }

  // Start streaming on each camera
  for(size_t i=0; i < cameras.size(); i++) {
    cout << "Starting streaming on Camera #" << i+1 << endl;
    cameras[i].startStreaming();
  }

  //
  // Loop infinitely, or until a 'q' is seen
  //
  const int scale  = 1;
  const int delta  = 0;
  const int ddepth = CV_16S;

  cout << "Starting previews.\nPress 'q' or Control-c to quit." << endl;

  while (true) {
    // Grab and display a single image from each camera
    for (size_t i = 0; i < cameras.size(); i++) {

      imagePtr = cameras[i].getRawImage();

      frameSize = cameras[i].getFrameSize();

      cameras[i].createRGBImage(imagePtr,frameSize);

      unsigned char* pImage = cameras[i].getImage();
      if (NULL != pImage) {

        Mat image(cameras[i].getMatSize(), CV_8UC3, pImage, Mat::AUTO_STEP);

        cameras[i].releaseImage(); // release asap

        cvtColor(image, image, CV_BGR2RGB,3);

        if (displaySobelImage) {
          GaussianBlur(image, image, Size(3,3), 0, 0, BORDER_DEFAULT);
        
          cv::Mat grey;
          cvtColor(image, grey, CV_BGR2GRAY);

          // Gradient X
          Mat grad_x;
          Mat abs_grad_x;
          Sobel( grey, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
          convertScaleAbs( grad_x, abs_grad_x );
          
          // Gradient Y
          Mat grad_y;
          Mat abs_grad_y;
          Sobel( grey, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
          convertScaleAbs( grad_y, abs_grad_y );

          // Total Gradient (approximate)
          Mat grad;
          addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad ); 
        
          resize( grad,  grad, cameras[i].getDisplaySize());
          imshow(cameras[i].  processedWindowName(), grad  );
        }
        
        // Show the actual image
        resize(image, image, cameras[i].getDisplaySize());
        imshow(cameras[i].unprocessedWindowName(), image );

      }
      
    }

    // wait for a key for 1 ms. If 'q' key pressed, quit program
    int keyPressed = waitKey(1);
    if (keyPressed == 'q') {
      cout << "Quitting..." << endl;
      break; 
    }
    
  }

  
  for(size_t i=0; i < cameras.size(); i++) {
    cameras[i].stopStreaming();
  }

  return 0;
}
