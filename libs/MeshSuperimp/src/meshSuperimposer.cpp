#include <Eigen/Geometry>
#include <iostream>
#include <meshSuperimposer.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <stdio.h>

#include <SuperimposeMesh/SICAD.h>
#include <unistd.h>


// Constructor 
MeshSuperimposer::MeshSuperimposer(std::string& path, Eigen::Transform<double,3,Eigen::Affine>& cameraExt, cv::Mat& cameraInt)
{
    // Save the mesh path 
    const char * cpath = path.std::string::c_str();
    if(access(cpath, F_OK ) != 0){
        std::cout << "ERROR: Cannot find the mesh file" << std::endl;
    }else{
        meshesPath_.emplace("wrist",cpath);
    }

    // Initialie SICAD object
    int width = 640;
    int height = 480;

    sicadPtr_ = std::unique_ptr<SICAD> 
    (
        new SICAD(meshesPath_, 640, 480, cameraInt.at<double>(0, 0), cameraInt.at<double>(1, 1), 
            cameraInt.at<double>(0, 2), cameraInt.at<double>(1, 2))
    );

    sicadPtr_->setBackgroundOpt(true);
    
    // Initialize intrinsic and extrinsic parameters 
    cameraIntrinsic_ = cameraInt;
    cameraEstrinsic_ = cameraExt;

}

// Destructor 
MeshSuperimposer::~MeshSuperimposer(){

}

std::vector<double> MeshSuperimposer::eigTransformToPose(Eigen::Transform<double,3,Eigen::Affine>& eigTransform)
{
    std::vector<double> pose(7);

    pose[0] = eigTransform.translation()(0);
    pose[1] = eigTransform.translation()(1);
    pose[2] = eigTransform.translation()(2);

    Eigen::AngleAxisd angleAxis(eigTransform.rotation());
    pose[3] = angleAxis.axis()(0);
    pose[4] = angleAxis.axis()(1);
    pose[5] = angleAxis.axis()(2);
    pose[6] = angleAxis.angle();

    return pose;
}

cv::Mat MeshSuperimposer::meshSuperimpose(std::vector<double> currentPose, cv::Mat currentFrame)
{   
    objposeMap_.emplace("wrist", currentPose);
    double camX [3] = {0.0, 0.0, 0.0};
    double camO [4] = {1.0, 0.0, 0.0, 0.0};
    sicadPtr_->superimpose(objposeMap_, camX, camO, currentFrame);

    cv::cvtColor(currentFrame, currentFrame, cv::COLOR_BGR2RGB, 0);

    return currentFrame;
}


