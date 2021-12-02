#ifndef MESH_SUPERIMPOSER
#define MESH_SUPERIMPOSER 

#include <Eigen/Geometry>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <SuperimposeMesh/SICAD.h>

class MeshSuperimposer 
{
public:
    MeshSuperimposer(std::string& path, Eigen::Transform<double,3,Eigen::Affine>& cameraExt, cv::Mat& cameraInt);
    ~MeshSuperimposer();

    cv::Mat meshSuperimpose(std::vector<double> currentPose, cv::Mat currentFrame);
    std::vector<double> eigTransformToPose(Eigen::Transform<double,3,Eigen::Affine>& eigTransform);

    SICAD::ModelPathContainer meshesPath_;
    cv::Mat cameraIntrinsic_;
    Eigen::Transform<double,3,Eigen::Affine> cameraEstrinsic_;

    std::unique_ptr<SICAD> sicadPtr_;
    Superimpose::ModelPoseContainer objposeMap_;

private:
};


#endif // MESH_SUPERIMPOSER 