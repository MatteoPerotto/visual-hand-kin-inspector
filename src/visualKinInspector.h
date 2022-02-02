#include<Eigen/Core>
#include<iostream>
#include<opencv4/opencv2/core.hpp>
#include<yarp/dev/IEncoders.h>
#include<yarp/dev/PolyDriver.h>
#include<yarp/os/Network.h>
#include<yarp/os/BufferedPort.h>
#include<yarp/sig/Image.h>
#include <iCub/iKin/iKinFwd.h>
#include <unordered_map>

class EncoderReader
{
    public:
    EncoderReader(std::string robotName, std::string partName);
    ~EncoderReader();
    Eigen::VectorXd readEncoders();
    std::unordered_map<std::string, iCub::iKin::iCubFinger> fingers_;

    private:
    yarp::dev::PolyDriver robotDriver_;
    yarp::dev::PolyDriver analogDriver_;
    int jnts_;
    yarp::dev::IEncoders *enc_;
    //Eigen::Map<Eigen::VectorXd> encoderReads_;
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

