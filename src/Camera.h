//******************************************************************************
//
// Copyright (c) 2019-2020, TELEDYNE LUMENERA, a business unit of TELEDYNE
// DIGITAL IMAGING, INCORPORATED. All rights reserved.
//
// This program is subject to the terms and conditions defined in file 'LICENSE'
// , which is part of this Linux LuCam SDK package.
//
//******************************************************************************

#ifndef OPENCV_SOBEL_CAMERA_H
#define OPENCV_SOBEL_CAMERA_H

#define LUMENERA_LINUX_API
#undef LUMENERA_MAC_API
#undef LUMENERA_WINDOWS_API


#include <lucamapi.h>
#include <string>
#include <vector>

#include <opencv2/highgui/highgui.hpp>

class Camera
{
public:
        Camera();
        virtual ~Camera(void);
        void init(int cameraIndex, const std::string& processingDesc = "Processed");

        void startStreaming();
        void stopStreaming();

        unsigned char* getImage();
        unsigned char* getRawImage();
        void           releaseImage();

        cv::Size    getDisplaySize();
        cv::Size    getMatSize();
        int getFrameSize();

        std::string name() const;

        std::string unprocessedWindowName() const { return m_unprocessedWindowName; }
        std::string   processedWindowName() const { return   m_processedWindowName; }

        void conversionDump(const std::string& filename) const;
        void createRGBImage(const BYTE* raw, const int length);

protected:
        int    cameraIndex;
        HANDLE hCamera;

        LUCAM_CONVERSION_PARAMS conversionParams;
        LUCAM_IMAGE_FORMAT imageFormat;

        bool streaming;
        int  width;
        int  height;
        int targetIntensity;

        std::string m_unprocessedWindowName;
        std::string   m_processedWindowName;

        //
        // We'll use a variation on Peterson's algorithm to control access to the rgbImage.
        //
        // The producer (the callback) can only write into the buffer if it's the owner
        // The consumer (the thread reading and displaying images) can only access the buffer if it's the owner
        // i.e. the producer is now the owner
        //
        // Producer
        //   if (!ownerIsProducer) return;
        //   // write image data to rgbImage
        //   ownerIsProducer = false;
        //
        // Consumer - Get Image
        //   if (ownerIsProducer) return
        //   // Allowed to access data
        // 
        // Consumer - Release Image
        //   assert(!ownerIsProducer);
        //   ownerIsProducer = true
        //
        // This works iff there are two threads involved.
        // One producer thread, one consumer thread.
        //
        std::vector<unsigned char> rgbImage;
        std::vector<unsigned char> rawImage;
        volatile bool rgbImageOwnerIsProducer;
};

#endif
