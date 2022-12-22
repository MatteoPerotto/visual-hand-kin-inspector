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
    EncoderReader(std::string portName);
    ~EncoderReader();
    std::unordered_map<std::string, double> readEncoders();
    void instantiateFingers();

    private:
    yarp::dev::PolyDriver robotDriver_;
    int jnts_;
    yarp::dev::IEncoders *enc_;
    std::unordered_map<std::string, double> encoderReads_; 
};



