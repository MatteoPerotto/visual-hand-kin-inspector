#include<iostream>
#include<mk2EncoderReader.h>
#include<yarp/os/Bottle.h>
#include<yarp/os/LogStream.h>
#include<yarp/os/Property.h>
#include<yarp/eigen/Eigen.h>
#include<yarp/sig/Vector.h>
#include<opencv4/opencv2/core.hpp>
#include <iCub/iKin/iKinFwd.h>
#include <unordered_map>


EncoderReader::EncoderReader(std::string portName)
{   
    // Read encoders 
    yarp::os::Property robotDriverOpt;
    robotDriverOpt.put("device", "remote_controlboard");
    robotDriverOpt.put("local", "/test/client");
    robotDriverOpt.put("remote", portName);  
    
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

void EncoderReader::instantiateFingers()
{
    // Instantiate fingers 
    fingers_["thumb"] = iCub::iKin::iCubFinger("left_thumb");
    fingers_["index"] = iCub::iKin::iCubFinger("left_index");
    fingers_["middle"] = iCub::iKin::iCubFinger("left_middle");
    fingers_["ring"] = iCub::iKin::iCubFinger("left_ring");
    fingers_["little"] = iCub::iKin::iCubFinger("left_little");
}

std::unordered_map<std::string, double> EncoderReader::readEncoders()
{   

    yarp::sig::Vector encoders(9);
    yarp::sig::Vector armEncoders(16);
    yarp::sig::Vector allchainJoints;
         
    // Obtain encoder values for the whole arm 
    enc_->getEncoders(armEncoders.data());

    // Select only hand encoders 
    yarp::eigen::toEigen(encoders) = yarp::eigen::toEigen(armEncoders).segment<9>(7);

    std::unordered_map<std::string,Eigen::VectorXd> fingerEncoders;

    for (auto& finger : fingers_)
    {
        yarp::sig::Vector chainJoints;
        finger.second.getChainJoints(encoders, chainJoints);
        Eigen::VectorXd chainJointsEigen = yarp::eigen::toEigen(chainJoints) * M_PI / 180.0;
        fingerEncoders[finger.first] = chainJointsEigen;    
    }

    encoderReads_["l_hand_little_0_joint"] = fingerEncoders["little"][0];
    encoderReads_["l_hand_thumb_0_joint"] = fingerEncoders["thumb"][0];
    encoderReads_["l_hand_middle_0_joint"] = 0;
    encoderReads_["l_hand_ring_0_joint"] = fingerEncoders["ring"][0];
    encoderReads_["l_hand_index_0_joint"] = -fingerEncoders["index"][0];
    encoderReads_["l_hand_index_1_joint"] = fingerEncoders["index"][1];
    encoderReads_["l_hand_index_2_joint"] = fingerEncoders["index"][2];
    encoderReads_["l_hand_index_3_joint"] = fingerEncoders["index"][3];
    encoderReads_["l_hand_ring_1_joint"] = fingerEncoders["ring"][1];
    encoderReads_["l_hand_ring_2_joint"] = fingerEncoders["ring"][2];
    encoderReads_["l_hand_ring_3_joint"] = fingerEncoders["ring"][3];
    encoderReads_["l_hand_middle_1_joint"] = fingerEncoders["middle"][0];
    encoderReads_["l_hand_middle_2_joint"] = fingerEncoders["middle"][1];
    encoderReads_["l_hand_middle_3_joint"] = fingerEncoders["middle"][2];
    encoderReads_["l_hand_thumb_1_joint"] = fingerEncoders["thumb"][1];
    encoderReads_["l_hand_thumb_2_joint"] = fingerEncoders["thumb"][2];
    encoderReads_["l_hand_thumb_3_joint"] = fingerEncoders["thumb"][3];
    encoderReads_["l_hand_little_1_joint"] = fingerEncoders["little"][1];
    encoderReads_["l_hand_little_2_joint"] = fingerEncoders["little"][2];
    encoderReads_["l_hand_little_3_joint"] = fingerEncoders["little"][3];

    return encoderReads_;
}   


