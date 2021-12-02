#include<iostream>
#include<Eigen/Geometry>
#include<librealsense2/rs.hpp>
#include<poseDetector.h>
#include<opencv4/opencv2/aruco.hpp>
#include<opencv4/opencv2/core.hpp>
#include<opencv4/opencv2/highgui.hpp>
#include<opencv4/opencv2/imgcodecs.hpp>
#include<opencv4/opencv2/imgproc/imgproc.hpp>
#include<opencv4/opencv2/videoio.hpp>

int main(int argc, char** argv)
{
    // Initialize extrinsic 
    Eigen::Affine3d T_rt(Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ()));
    Eigen::Transform<double,3,Eigen::Affine> extP;
    extP.matrix() = T_rt.matrix();

    // Initialize pose detector object 
    PoseDetector posedet(extP);

    //Initialize a shared pointer to the pipeline [CASE THE PIPELINE IS PASSED TO THE CONSTRUCTOR TO OBTAIN INTRINSIC ]
    std::shared_ptr<rs2::pipeline> p (new rs2::pipeline);
    posedet.getIntrinsic(p);
    
    for (;;)
    {    
        rs2::frameset frames = p->wait_for_frames();
        rs2::video_frame color = frames.get_color_frame();
        
        cv::Mat imageIn(cv::Size(posedet.imgW_, posedet.imgH_), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat markedImage = posedet.poseUpdate(imageIn);

        cv::imshow("Realsense", markedImage);
        if (cv::waitKey(5) >= 0)
            break;
    }
    
    return EXIT_SUCCESS ;

}