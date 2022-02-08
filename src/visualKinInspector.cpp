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


EncoderReader::EncoderReader(std::string robotName, std::string partName, std::vector<std::string> dofList)
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

    dofList_ = dofList;
    encoderReads_ = Eigen::VectorXd::Zero(20);
}

EncoderReader::~EncoderReader()
{
    robotDriver_.close();
}

Eigen::VectorXd EncoderReader::readEncoders()
{   
    yarp::sig::Vector encoders(9);
    yarp::sig::Vector armEncoders(16);
    yarp::sig::Vector allchainJoints;
         
    // Obtain encoder values for the whole arm 
    enc_->getEncoders(armEncoders.data());

    // Select only hand encoders 
    yarp::eigen::toEigen(encoders) = yarp::eigen::toEigen(armEncoders).segment<9>(7);//

    std::unordered_map<std::string,Eigen::VectorXd> fingerEncoders;

    for (auto& finger : fingers_)
    {
        yarp::sig::Vector chainJoints;
        finger.second.getChainJoints(encoders, chainJoints);
        Eigen::VectorXd chainJointsEigen = yarp::eigen::toEigen(chainJoints) * M_PI / 180.0;
        fingerEncoders[finger.first] = chainJointsEigen;    
    }

    encoderReads_[0] = fingerEncoders["little"][0];
    encoderReads_[1] = fingerEncoders["thumb"][0];
    encoderReads_[2] = 0;
    encoderReads_[3] = fingerEncoders["ring"][0];
    encoderReads_[4] = -fingerEncoders["index"][0];
    encoderReads_[5] = fingerEncoders["index"][1];
    encoderReads_[6] = fingerEncoders["index"][2];
    encoderReads_[7] = fingerEncoders["index"][3];
    encoderReads_[8] = fingerEncoders["ring"][1];
    encoderReads_[9] = fingerEncoders["ring"][2];
    encoderReads_[10] = fingerEncoders["ring"][3];
    encoderReads_[11] = fingerEncoders["middle"][0];
    encoderReads_[12] = fingerEncoders["middle"][1];
    encoderReads_[13] = fingerEncoders["middle"][2];
    encoderReads_[14] = fingerEncoders["thumb"][1];
    encoderReads_[15] = fingerEncoders["thumb"][2];
    encoderReads_[16] = fingerEncoders["thumb"][3];
    encoderReads_[17] = fingerEncoders["little"][1];
    encoderReads_[18] = fingerEncoders["little"][2];
    encoderReads_[19] = fingerEncoders["little"][3];

    return encoderReads_;
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


