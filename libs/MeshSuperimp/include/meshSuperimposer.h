#ifndef MESH_SUPERIMPOSER
#define MESH_SUPERIMPOSER

#include <Eigen/Geometry>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <SuperimposeMesh/SICAD.h>

class MeshSuperimposer
{
public:
    MeshSuperimposer(std::vector<std::pair<std::string,std::string>>& paths, Eigen::Matrix3d cameraInt, int imgW, int imgH, Eigen::Transform<double,3,Eigen::Affine> cameraExt = Eigen::Transform<double,3,Eigen::Affine>::Identity());
    ~MeshSuperimposer();

    cv::Mat meshSuperimpose(std::vector<Eigen::Transform<double,3,Eigen::Affine>>& eigTransforms, cv::Mat currentFrame);
    std::vector<double> eigTransformToPose(Eigen::Transform<double,3,Eigen::Affine>& eigTransform);

private:
    size_t meshN_;
    SICAD::ModelPathContainer meshesContainer_;
    std::vector<std::string> idContainer_;
    Eigen::MatrixXd cameraIntrinsic_;
    Eigen::Transform<double,3,Eigen::Affine> cameraEstrinsic_;
    std::unique_ptr<SICAD> sicadPtr_;
};


#endif // MESH_SUPERIMPOSER
