#include<iostream>
#include<visualKinInspector.h>
#include<yarp/os/Bottle.h>
#include<yarp/os/BufferedPort.h>
#include<yarp/os/LogStream.h>
#include<yarp/os/Network.h>
#include<yarp/os/Property.h>
#include<yarp/eigen/Eigen.h>
#include<yarp/sig/Vector.h>
#include<opencv4/opencv2/core.hpp>
#include<yarp/cv/Cv.h>
#include <iCub/iKin/iKinFwd.h>
#include <unordered_map>


EncoderReader::EncoderReader(std::string robotName, std::string partName)
{   
    // Instantiate fingers 
    fingers_["thumb"] = iCub::iKin::iCubFinger("left_thumb");
    fingers_["index"] = iCub::iKin::iCubFinger("left_index");
    fingers_["middle"] = iCub::iKin::iCubFinger("left_middle");
    fingers_["ring"] = iCub::iKin::iCubFinger("left_ring");
    fingers_["little"] = iCub::iKin::iCubFinger("left_little");

    // Read encoders 
    yarp::os::Property robotDriverOpt;
    robotDriverOpt.put("device", "remote_controlboard");
    robotDriverOpt.put("local", "/test/client");
    robotDriverOpt.put("remote", "/" + robotName + "/" + partName); 
    
    if(robotDriver_.open(robotDriverOpt))
    {
        robotDriver_.view(enc_);
        enc_->getAxes(&jnts_);
        std::cout << "Found ENCODERS: " << jnts_ << std::endl;

    }
}

EncoderReader::~EncoderReader()
{
    robotDriver_.close();
}

std::unordered_map<std::string,Eigen::VectorXd>  EncoderReader::readEncoders()
{   
    yarp::sig::Vector encoders(9);
    yarp::sig::Vector armEncoders(16);
    yarp::sig::Vector allchainJoints;
         
    // Obtain encoder values for the whole arm 
    enc_->getEncoders(armEncoders.data());

    // Select only hand encoders 
    yarp::eigen::toEigen(encoders) = yarp::eigen::toEigen(armEncoders).segment<9>(7);//

    std::unordered_map<std::string,Eigen::VectorXd> outputEncoders;

    for (auto& finger : fingers_)
    {
        yarp::sig::Vector chainJoints;
        finger.second.getChainJoints(encoders, chainJoints);
        Eigen::VectorXd chainJointsEigen = yarp::eigen::toEigen(chainJoints) * M_PI / 180.0;
        outputEncoders[finger.first] = chainJointsEigen;    
    }
    return outputEncoders;
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


