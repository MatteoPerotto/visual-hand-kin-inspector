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

    int width = 640;
    int height = 480;
    float ppx = 323.418;
    float ppy = 248.989;
    float fx = 615.359;
    float fy = 615.599;

    cv::Mat intP = cv::Mat::eye(3, 3, CV_64F);
    intP.at<double>(0, 0) = fx;
    intP.at<double>(0, 1) = 0;
    intP.at<double>(0, 2) = ppx;
    intP.at<double>(1, 0) = 0;
    intP.at<double>(1, 1) = fy;
    intP.at<double>(1, 2) = ppy; 
    intP.at<double>(2, 2) = 0;
    intP.at<double>(2, 2) = 0;
    intP.at<double>(2, 2) = 1;

    std::string path = "/home/matteoperotto/robotology-superbuild/src/icub-models/iCub/meshes/simmechanics/sim_l_wrist_hand_prt.stl";
   
    MeshSuperimposer mSup(path, extP, intP);

    Superimpose::ModelPose obj_pose(7);
    obj_pose[0] = 0;
    obj_pose[1] = 0;
    obj_pose[2] = -500; // mt
    obj_pose[3] = 1;
    obj_pose[4] = 0;
    obj_pose[5] = 0;
    obj_pose[6] = 0;
       
    std::shared_ptr<rs2::pipeline> p (new rs2::pipeline);
    p->start();
    int j=0;

    for (;;)
    {
        rs2::frameset frames = p->wait_for_frames();
        rs2::video_frame color = frames.get_color_frame();
        cv::Mat imageIn(cv::Size(width, height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
             
        // Superimpose 
        cv::Mat outImg = mSup.meshSuperimpose(obj_pose,imageIn);

        // Output the superimposed 
        cv::imshow("Webcam source", outImg);
        if (cv::waitKey(5) >= 0)
            break;
        
        j++;
    }  

    return EXIT_SUCCESS ;
    
}

