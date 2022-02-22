#include<Eigen/Geometry>
#include<iostream>
#include<meshSuperimposer.h>
#include<opencv4/opencv2/core.hpp>
#include<opencv4/opencv2/aruco.hpp>
#include<opencv4/opencv2/imgcodecs.hpp>
#include<opencv4/opencv2/imgproc/imgproc.hpp>
#include<opencv4/opencv2/videoio.hpp>
#include<opencv4/opencv2/highgui.hpp>
#include<stdio.h>
#include<cmath>
#include<SuperimposeMesh/SICAD.h>
#include<unistd.h>


// Constructor 
MeshSuperimposer::MeshSuperimposer(std::vector<std::pair<std::string,std::string>>& paths, Eigen::Matrix3d cameraInt, int imgW, int imgH, Eigen::Transform<double,3,Eigen::Affine> cameraExt)
{
    // Save the mesh path
    meshN_ = paths.size();
    for(int meshIndex=0; meshIndex<meshN_; meshIndex++){

        const char * cpath = paths[meshIndex].second.std::string::c_str();
        if(access(cpath, F_OK ) != 0){
            std::cout << "ERROR: Cannot find the mesh file" << std::endl;
        }else{
            meshesContainer_.emplace(paths[meshIndex].first,cpath);
            idContainer_.push_back(paths[meshIndex].first);
            std::cout << idContainer_[meshIndex] << std::endl;
        } 
    }
    
    // Initialie SICAD object
    sicadPtr_ = std::unique_ptr<SICAD> 
    (
        new SICAD(meshesContainer_, imgW, imgH, cameraInt(0, 0), cameraInt(1, 1), 
            cameraInt(0, 2), cameraInt(1, 2))
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

cv::Mat MeshSuperimposer::meshSuperimpose(std::vector<Eigen::Transform<double,3,Eigen::Affine>>& eigTransforms, cv::Mat currentFrame)
{   
   
    std::vector<double> currentPose(7);
    Superimpose::ModelPoseContainer objposeMap;
    cv::Mat bkgrndImage(640, 480, CV_8UC3, cv::Scalar(0, 0, 0));

    for(int meshIndex=0; meshIndex<meshN_; meshIndex++){

        currentPose[0] = eigTransforms[meshIndex].translation()(0);
        currentPose[1] = eigTransforms[meshIndex].translation()(1);
        currentPose[2] = eigTransforms[meshIndex].translation()(2);
        
        Eigen::AngleAxisd angleAxis(eigTransforms[meshIndex].rotation());
        currentPose[3] = angleAxis.axis()(0); 
        currentPose[4] = angleAxis.axis()(1);
        currentPose[5] = angleAxis.axis()(2);
        currentPose[6] = angleAxis.angle();
        objposeMap.emplace(idContainer_[meshIndex], currentPose);
    }
    
    double camX [3] = {0.0, 0.0, 0.0};
    double camO [4] = {1.0, 0.0, 0.0, static_cast<float>(M_PI)};
    sicadPtr_->superimpose(objposeMap, camX, camO, bkgrndImage);

    cv::cvtColor( bkgrndImage , bkgrndImage , cv::COLOR_BGR2GRAY);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(bkgrndImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

    cv::Scalar color = cv::Scalar(255, 0, 0);
    cv::drawContours( currentFrame, contours, 0, color, 2, cv::LINE_8, hierarchy, 0 );

    return currentFrame;
}


