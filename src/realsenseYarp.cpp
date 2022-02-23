#include<realsenseYarp.h>
#include<yarp/dev/IRGBDSensor.h>
#include<yarp/dev/PolyDriver.h>
#include<yarp/os/Property.h>
#include<vector>


RealsenseYarp::RealsenseYarp(std::string portPrefix)
{
    yarp::dev::PolyDriver driver;
    yarp::dev::IRGBDSensor* interface;

    yarp::os::Property driverProperties;
    driverProperties.put("device", "RGBDSensorClient");
    driverProperties.put("localImagePort",  "/" + portPrefix + "/RGBDSensorClient/image:i");
    driverProperties.put("localDepthPort",  "/" + portPrefix + "/RGBDSensorClient/depth:i");
    driverProperties.put("localRpcPort",    "/" + portPrefix + "/RGBDSensorClient/rpc:i");
    driverProperties.put("remoteImagePort", "/depthCamera/rgbImage:o");
    driverProperties.put("remoteDepthPort", "/depthCamera/depthImage:o");
    driverProperties.put("remoteRpcPort",   "/depthCamera/rpc:i");

    if (driver.open(driverProperties) && driver.view(interface) && (interface != nullptr))
    {
        yarp::os::Property cameraIntrinsics;
        interface->getRgbIntrinsicParam(cameraIntrinsics);

        std::size_t cameraWidth = interface->getRgbWidth();
        std::size_t cameraHeight = interface->getRgbHeight();

        imgSize_.first = cameraWidth;
        imgSize_.second  = cameraHeight;

        ppx_ = cameraIntrinsics.find("principalPointX").asFloat64();
        ppy_ = cameraIntrinsics.find("principalPointY").asFloat64();
        fx_ = cameraIntrinsics.find("focalLengthX").asFloat64();
        fy_ = cameraIntrinsics.find("focalLengthY").asFloat64();
        
        intP_(0, 0) = fx_;
        intP_(0, 1) = 0;
        intP_(0, 2) = ppx_;
        intP_(1, 0) = 0;
        intP_(1, 1) = fy_;
        intP_(1, 2) = ppy_;
        intP_(2, 0) = 0;
        intP_(2, 1) = 0;
        intP_(2, 2) = 1;

        driver.close();
    }
}

RealsenseYarp::~RealsenseYarp()
{
    
}

Eigen::Matrix3d  RealsenseYarp::getIntrinsicMatrix()
{
    return intP_;
}

std::vector<double> RealsenseYarp::getIntrinsicVector()
{
    std::vector<double> outV = {ppx_,ppy_,fx_,fy_};
    return outV;
}

std::pair<int,int> RealsenseYarp::getImgSize()
{
    return imgSize_;
}