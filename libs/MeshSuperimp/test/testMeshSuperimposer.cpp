#include<SuperimposeMesh/SICAD.h>
#include<Eigen/Geometry>
#include<iostream>
#include<unistd.h>
#include<librealsense2/rs.hpp> 
#include<opencv4/opencv2/core.hpp>
#include<opencv4/opencv2/aruco.hpp>
#include<opencv4/opencv2/imgcodecs.hpp>
#include<opencv4/opencv2/imgproc/imgproc.hpp>
#include<opencv4/opencv2/videoio.hpp>
#include<opencv4/opencv2/highgui.hpp>
#include<meshSuperimposer.h>

int main(int argc, char** argv){

    Eigen::Transform<double,3,Eigen::Affine> extP = Eigen::Transform<double,3,Eigen::Affine>::Identity();
    Eigen::Matrix3d intP;

    int width = 640;
    int height = 480;
    const float fx = 615.3594360351562;
    const float fy = 615.5988159179688;
    const float ppx = 323.4178161621094;
    const float ppy = 248.9889831542969;
    const float coeff[5] = {0,0,0,0,0};

    intP(0, 0) = fx;
    intP(0, 1) = 0;
    intP(0, 2) = ppx;
    intP(1, 0) = 0;
    intP(1, 1) = fy;
    intP(1, 2) = ppy;
    intP(2, 0) = 0;
    intP(2, 1) = 0;
    intP(2, 2) = 1;
    
    std::vector<std::pair<std::string,std::string>> paths;
    paths.push_back(std::make_pair("wrist","/home/matteoperotto/robotology-superbuild/src/icub-models/iCub/meshes/simmechanics/sim_l_wrist_hand_prt.stl"));
    paths.push_back(std::make_pair("palm","/home/matteoperotto/robotology-superbuild/src/icub-models/iCub/meshes/simmechanics/sim_l_hand_palm_naked_prt.stl"));

    MeshSuperimposer mSup(paths, extP, intP, 640, 480);

    Eigen::Transform<double,3,Eigen::Affine> mytransform1;
    mytransform1 = Eigen::Transform<double,3,Eigen::Affine>::Identity();
    mytransform1 = mytransform1*Eigen::Translation<double,3>(0,0,-500);

    Eigen::Transform<double,3,Eigen::Affine> mytransform2;
    mytransform2 = Eigen::Transform<double,3,Eigen::Affine>::Identity();
    mytransform2 = mytransform2*Eigen::Translation<double,3>(50,75,-500);

    std::vector<Eigen::Transform<double,3,Eigen::Affine>> vectTransform;
    vectTransform.push_back(mytransform1);
    vectTransform.push_back(mytransform2);


    std::shared_ptr<rs2::pipeline> p (new rs2::pipeline);
    p->start();
    int j=0;

    for (;;)
    {
        rs2::frameset frames = p->wait_for_frames();
        rs2::video_frame color = frames.get_color_frame();
        cv::Mat imageIn(cv::Size(width, height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        cv::cvtColor(imageIn, imageIn, cv::COLOR_BGR2RGB, 0); 

        // Superimpose 
        cv::Mat outImg = mSup.meshSuperimpose(vectTransform,imageIn);

        // Output the superimposed 
        cv::imshow("Webcam source", outImg);
        if (cv::waitKey(5) >= 0)
            break;
        
        j++;
    }  

    return EXIT_SUCCESS ;
    
}

