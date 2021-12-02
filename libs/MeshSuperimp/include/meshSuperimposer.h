#ifndef MESH_SUPERIMPOSER
#define MESH_SUPERIMPOSER 

#include <Eigen/Geometry>
#include <opencv4/opencv2/core.hpp>
#include <SuperimposeMesh/SICAD.h>

class MeshSuperimposer 
{
public:
    MeshSuperimposer(Eigen::Transform<double,3,Eigen::Affine> cameraExt, Eigen::MatrixXd cameraInt);
    ~MeshSuperimposer();
    void meshSuperimpose(const Eigen::Transform<double,3,Eigen::Affine> & tr);

    Eigen::MatrixXd cameraIntrinsic_;
    Eigen::Transform<double,3,Eigen::Affine> cameraExtrinsic_;
    cv::Mat markerImage_;

private:
};


#endif // MESH_SUPERIMPOSER 