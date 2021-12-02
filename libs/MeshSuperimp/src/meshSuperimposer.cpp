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
MeshSuperimposer::MeshSuperimposer(Eigen::Transform<double,3,Eigen::Affine> cameraExt, Eigen::MatrixXd cameraInt){

    // Initialize intrinsic and extrinsic parameters 
    cameraIntrinsic_ = cameraInt;
    cameraExtrinsic_ = cameraExt;

}

// Destructor 
MeshSuperimposer::~MeshSuperimposer(){

}


void MeshSuperimposer::meshSuperimpose(const Eigen::Transform<double,3,Eigen::Affine> & tr){
    
    //std::cout << tr.matrix() << std::endl;

}

