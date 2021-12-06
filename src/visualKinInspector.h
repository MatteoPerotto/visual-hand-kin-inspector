#include<iostream>
#include<yarp/os/Network.h>
#include<yarp/os/BufferedPort.h>
#include<yarp/sig/Image.h>
#include<opencv4/opencv2/core.hpp>

class EncoderReader
{
    public:
    EncoderReader();
    ~EncoderReader();
    
};

class ImageStreamer
{
    public:
    ImageStreamer(const std::string& readPortPrefix);
    ~ImageStreamer();
    cv::Mat readFrame();
    void sendFrame(const cv::Mat& imgToBeSent);

    yarp::os::Network yarp_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> rgbPortIn_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> rgbPortOut_;

};

