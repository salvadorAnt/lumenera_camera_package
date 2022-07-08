#define LUMENERA_LINUX_API
#undef LUMENERA_MAC_API
#undef LUMENERA_WINDOWS_API

#include "Camera.h"
#include <stdio.h>
#include <fstream>
#include <iterator>
#include <vector>
#include <string>
#include <iostream>
#include <cassert>
#include <unistd.h>

using std::string;
using std::vector;

Camera::Camera() :
  cameraIndex(-1),
  hCamera(NULL),
  targetIntensity(0),
  streaming(false),
  rgbImageOwnerIsProducer(true)
{
}


Camera::~Camera(void)
{
  if(streaming) {
    stopStreaming();
  }

  if(NULL != hCamera) {
    LucamCameraClose(hCamera);
    hCamera = NULL;
  }
}

//
// processing is set to "" if you don't want a window with a processed image
//
void Camera::init(int index, const std::string& processing)
{
  char buffer[512];
  LONG flags;
  HANDLE temp_hCamera = LucamCameraOpen(index);
  if (NULL == temp_hCamera) {
    return;
  }
  hCamera = temp_hCamera;
  cameraIndex = index;
  LucamSetProperty(hCamera, LUCAM_PROP_GAIN, 2, 0);
  targetIntensity = 90;
  LucamSetProperty(hCamera, LUCAM_PROP_AUTO_EXP_TARGET, (float)targetIntensity,0);
  LucamSetProperty(hCamera, LUCAM_PROP_EXPOSURE, 5, 0);

  LucamStreamVideoControl(hCamera,START_STREAMING ,NULL);

  imageFormat.Size = sizeof(LUCAM_IMAGE_FORMAT);
  LucamGetVideoImageFormat(hCamera, &imageFormat);
  
  LucamOneShotAutoWhiteBalance(hCamera, 0, 0, imageFormat.Width, imageFormat.Height);
  LucamDigitalWhiteBalance(hCamera, 0, 0, imageFormat.Width, imageFormat.Height);

  LucamStreamVideoControl(hCamera, STOP_STREAMING, NULL);
  LucamSetProperty(hCamera, LUCAM_PROP_EXPOSURE, 5, LUCAM_PROP_FLAG_AUTO|LUCAM_PROP_FLAG_USE);
  
  if(hCamera !=NULL)    {
    conversionParams.CorrectionMatrix = LUCAM_CM_FLUORESCENT;
    conversionParams.DemosaicMethod = LUCAM_DM_FAST;
    conversionParams.UseColorGainsOverWb = TRUE;
    conversionParams.Size = sizeof(LUCAM_CONVERSION_PARAMS);
    LucamGetProperty(hCamera, LUCAM_PROP_DIGITAL_GAIN_BLUE,  &conversionParams.DigitalGainBlue, &flags);
    LucamGetProperty(hCamera, LUCAM_PROP_DIGITAL_GAIN_GREEN, &conversionParams.DigitalGainGreen, &flags);
    LucamGetProperty(hCamera, LUCAM_PROP_DIGITAL_GAIN_RED,   &conversionParams.DigitalGainRed, &flags);
    conversionParams.FlipX = FALSE;
    conversionParams.FlipY = FALSE;
    conversionParams.Hue=0;
    conversionParams.Saturation=1;
  }
  
  
  
}

void Camera::startStreaming()
{
  if (!streaming) {
      if (LucamStreamVideoControl(hCamera,START_STREAMING ,NULL)) {
        streaming = true;
      }
  }
}

void Camera::stopStreaming()
{
  if (streaming) {
    if (LucamStreamVideoControl(hCamera, STOP_STREAMING, NULL)) {
      streaming = false;
    }
  }
}


void Camera::releaseImage()
{
  rgbImageOwnerIsProducer = true;
}

unsigned char* Camera::getRawImage()
{
  rawImage.resize(imageFormat.ImageSize);
  LucamTakeVideo(hCamera,1,reinterpret_cast<BYTE*>(&rawImage[0]));
  return &rawImage[0];
}

unsigned char* Camera::getImage()
{
  // Can't touch the image if the producer is still owning it
  // Only the owner can transfer ownership
  if (rgbImageOwnerIsProducer) {
    return NULL;
  }
  
  return &rgbImage[0];

}

int Camera::getFrameSize()
{
  return imageFormat.ImageSize;
}

std::string Camera::name() const
{
  if (cameraIndex >= 1) {
    char buffer[128];
    sprintf(&buffer[0], "Camera %d", cameraIndex);
    return std::string(buffer);
  }
  assert(false);
  return std::string("Uninitialized camera");
}

cv::Size Camera::getMatSize()
{
  cv::Size s(imageFormat.Width, imageFormat.Height);
  return s;
}

void Camera::conversionDump(const std::string& filename) const 
{
  std::ofstream f(filename.c_str());
  for(vector<unsigned char>::const_iterator i = rgbImage.begin(); i != rgbImage.end(); ++i) {
    f << *i << '\n';
  }
}


void Camera::createRGBImage(const BYTE* raw, const int length)
{
  // Consumer of this object still needs the rgb image?
  // Drop this frame then
  if (!rgbImageOwnerIsProducer) {
    return;
  }
  assert(rgbImageOwnerIsProducer);


  rgbImage.resize(length * 3);

  LucamConvertFrameToRgb24Ex(hCamera, &rgbImage[0], raw, &imageFormat, &conversionParams);

  // Transfer ownership to consumer
  rgbImageOwnerIsProducer = false;

}
