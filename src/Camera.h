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
        int targetIntensity;

        std::string m_unprocessedWindowName;
        std::string   m_processedWindowName;

        std::vector<unsigned char> rgbImage;
        std::vector<unsigned char> rawImage;
        volatile bool rgbImageOwnerIsProducer;
};

#endif
