#include<iostream>
#include<visualKinInspector.h>
#include<yarp/os/Bottle.h>
#include<yarp/os/BufferedPort.h>
#include<yarp/os/LogStream.h>
#include<yarp/os/Network.h>
#include<opencv4/opencv2/core.hpp>
#include <yarp/cv/Cv.h>


EncoderReader::EncoderReader()
{
}

EncoderReader::~EncoderReader()
{
}

// The constructor creates two ports, one reading images from the camera and the other sending images to the yarpviewer port 
ImageStreamer::ImageStreamer(const std::string& rPortPrefix){

    // Check YARP network
    if (!yarp_.checkNetwork())
    {
        throw(std::runtime_error("[ERROR] YARP network is not available."));
    }

    // Open rgb input port
    if (!(rgbPortIn_.open("/imageStreamer/" + rPortPrefix + ":i")))
    {
        throw(std::runtime_error("[ERROR] Cannot open an imageStreamer input port."));
    }

    // Open rgb output port
    if (!(rgbPortOut_.open("/imageStreamer/" + rPortPrefix + ":o")))
    {
        throw(std::runtime_error("[ERROR] Cannot open an imageStreamer output port."));
    }

}

ImageStreamer::~ImageStreamer()
{
}

cv::Mat ImageStreamer::readFrame()
{

    yarp::sig::ImageOf<yarp::sig::PixelRgb>* imgIn;
    imgIn = rgbPortIn_.read();

    cv::Mat imgCv = yarp::cv::toCvMat(*imgIn);
    return imgCv;
}

void ImageStreamer::sendFrame(const cv::Mat& imgToBeSent)
{

    yarp::sig::ImageOf<yarp::sig::PixelRgb>& imgYarp = rgbPortOut_.prepare();
    cv::Mat imgCv = imgToBeSent.clone();
    imgYarp = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(imgCv);

    rgbPortOut_.write();
}


