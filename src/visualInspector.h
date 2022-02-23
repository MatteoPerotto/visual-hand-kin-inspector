#ifndef VISKIN  
#define VISKIN

#include<Eigen/Geometry>
#include<meshKinematics.h>
#include<meshSuperimposer.h>
#include<mk2EncoderReader.h>
#include<opencv4/opencv2/core.hpp>
#include<poseDetector.h>
#include<yarp/os/RFModule.h> 
#include<yarp/os/ResourceFinder.h>
#include<yarp/os/BufferedPort.h>
#include<yarp/sig/Image.h>

#endif 

class VisualInspector : public yarp::os::RFModule
{

public:
 
    bool close() override;
    bool configure(yarp::os::ResourceFinder& rf) override;
    double getPeriod() override;
    bool updateModule() override;

    std::vector<int> retrieveIntList(yarp::os::ResourceFinder rf, std::string key, std::size_t size);
    std::vector<double> retrieveDoubleList(yarp::os::ResourceFinder rf, std::string key, std::size_t size);

private:

    int scaleFactor_;
    std::vector<std::pair<std::string,std::string>> mPath_;
    std::unique_ptr<PoseDetector> ptrPoseDet_;
    std::unique_ptr<MeshKinematics> ptrMkObject_;
    std::unique_ptr<MeshSuperimposer> ptrMeshSup_;
    std::unique_ptr<EncoderReader> ptrEncRead_;
    std::vector<double> intVector_;
    std::pair<int,int> imgSize_;
    std::vector<int> markerIds_;
    Eigen::Matrix3d intMatrix_;
    Eigen::Transform<double, 3, Eigen::Affine> fixedT_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> rgbPortIn_;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> rgbPortOut_;

};


