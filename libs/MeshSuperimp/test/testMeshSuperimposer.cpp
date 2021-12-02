#include <SuperimposeMesh/SICAD.h>
#include <Eigen/Geometry>
#include <iostream>
#include <unistd.h>
#include<librealsense2/rs.hpp> 
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <opencv4/opencv2/highgui.hpp>

int main(int argc, char** argv){

    Eigen::Affine3d T_rt(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ()));
    Eigen::Transform<double,3,Eigen::Affine> extP;
    extP.matrix() = T_rt.matrix();

    int width = 640;
    int height = 480;
    float cx = 323.418;
    float cy = 248.989;
    float fx = 615.359;
    float fy = 615.599;

    Eigen::MatrixXd intP(3,3);
    intP(0,0) = fx;
    intP(0,1) = 0;
    intP(0,2) = cx;
    intP(1,0) = 0;
    intP(1,1) = fy;
    intP(1,2) = cy; 
    intP(2,2) = 0;
    intP(2,2) = 0;
    intP(2,2) = 1;

    SICAD::ModelPathContainer meshes;
    if(access( "/home/matteoperotto/robotology-superbuild/src/icub-models/iCub/meshes/simmechanics/sim_l_wrist_hand_prt.stl", F_OK ) != 0){
        std::cout << "ERROR: Cannot find the mesh file" << std::endl;
        return EXIT_FAILURE;
    }else{
        meshes.emplace("wrist","/home/matteoperotto/robotology-superbuild/src/icub-models/iCub/meshes/simmechanics/sim_l_wrist_hand_prt.stl");
    }

    SICAD si_cad(meshes, width, height, fx, fy, cx, cy);

    Superimpose::ModelPose obj_pose(7);
    obj_pose[0] = 0;
    obj_pose[1] = 0;
    obj_pose[2] = 0.2; // mt
    obj_pose[3] = 1;
    obj_pose[4] = 0;
    obj_pose[5] = 0;
    obj_pose[6] = 0;
    
    Superimpose::ModelPoseContainer objpose_map;
    objpose_map.emplace("wrist", obj_pose);

    double cam_x[] = {  0, 0, 0};
    double cam_o[] = {1.0, 0, 0, 0};

    si_cad.setBackgroundOpt(true);

    std::shared_ptr<rs2::pipeline> p (new rs2::pipeline);
    p->start();

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Mat markerImage;
    cv::aruco::drawMarker(dictionary, 23, 500, markerImage, 1);


    for (;;)
    {
        rs2::frameset frames = p->wait_for_frames();
        rs2::video_frame color = frames.get_color_frame();
        cv::Mat imageIn(cv::Size(width, height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        
        // Superimpose 
        si_cad.superimpose(objpose_map, cam_x, cam_o, imageIn);

        // Output the superimposed 
        cv::imshow("Webcam source", imageIn);
        if (cv::waitKey(5) >= 0)
            break;
    }
 
    return EXIT_SUCCESS ;

}

