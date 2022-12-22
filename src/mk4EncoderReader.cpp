#include<iostream>
#include<mk4EncoderReader.h>
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
    /*
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
    */
}

EncoderReader::~EncoderReader()
{
    //robotDriver_.close();
}

void EncoderReader::instantiateFingers()
{
    std::cout << "[INFO] No need to instantiate fingers for mk4, read only from port" << std::endl;
}

std::unordered_map<std::string, double> EncoderReader::readEncoders()
{   
    /*yarp::sig::Vector encoders(7);
    // Obtain encoder values for the whole arm 
    enc_->getEncoders(encoders.data()); 
    encoders *= M_PI / 180.0;

    std::cout << encoders.toString() << std::endl;

    encoderReads_["l_thumb_prox_joint"] = encoders[0];  
    encoderReads_["l_index_prox_joint"] = encoders[1]; 
    encoderReads_["l_palm_middle_joint"] = encoders[2];
    encoderReads_["l_palm_ring_joint"] = encoders[3];
    encoderReads_["l_palm_pinkie_joint"] = encoders[3];  
    encoderReads_["l_thumb_add_joint"] = encoders[4];
    encoderReads_["l_thumb_rot_joint"] = encoders[5];
    encoderReads_["l_index_add_joint"] = encoders[6];
    encoderReads_["l_thumb_dist_joint"] = 0.0;
    encoderReads_["l_middle_dist_joint"] = 0.0;
    encoderReads_["l_index_dist_joint"] = 0.0;
    encoderReads_["l_ring_dist_joint"] = 0.0;
    encoderReads_["l_pinkie_dist_joint"] = 0.0;
    */

    encoderReads_["l_thumb_prox_joint"] = 0.0;  
    encoderReads_["l_index_prox_joint"] = 0.0; 
    encoderReads_["l_palm_middle_joint"] = 0.0;
    encoderReads_["l_palm_ring_joint"] = 0.0;
    encoderReads_["l_palm_pinkie_joint"] = 0.0;
    encoderReads_["l_thumb_add_joint"] = 0.0;
    encoderReads_["l_thumb_rot_joint"] = 0.0;
    encoderReads_["l_index_add_joint"] = 0.0;
    encoderReads_["l_thumb_dist_joint"] = 0.0;
    encoderReads_["l_middle_dist_joint"] = 0.0;
    encoderReads_["l_index_dist_joint"] = 0.0;
    encoderReads_["l_ring_dist_joint"] = 0.0;
    encoderReads_["l_pinkie_dist_joint"] = 0.0;
    

    return encoderReads_;
}   


