#include<Eigen/Geometry>
#include<iostream>
#include<librealsense2/rs.hpp>
#include<poseDetector.h>
#include<opencv4/opencv2/aruco.hpp>
#include<opencv4/opencv2/core.hpp>
#include<opencv4/opencv2/highgui.hpp>
#include<opencv4/opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv)
{
    // Initialize pose detector object 
    PoseDetector posedet(0); 
    std::vector<int> markerIds1 = {0,1,2,3};
    std::vector<int> markerIds2 = {4,5,6,7};
    posedet.addBoard(0,2,2,0.02,0.005,markerIds1);
    posedet.addBoard(1,2,2,0.02,0.005,markerIds2);

    for(auto& b: posedet.boardPtr_)
    {
        std::cout << "Board id: " << b.first << std::endl;
    }
 
    // Test the tuples 
    for(auto& singleMarker: posedet.outPoses_)
    {   
        std::cout << "ID: " << singleMarker.first << "\nBOOL: " << singleMarker.second.first << "\nPOSE:\n" << singleMarker.second.second.matrix() << std::endl;
    } 

    //Initialize a shared pointer to the pipeline [CASE THE PIPELINE IS PASSED TO THE CONSTRUCTOR TO OBTAIN INTRINSIC ]
    std::shared_ptr<rs2::pipeline> p (new rs2::pipeline);
    posedet.getIntrinsic(p);
    std::cout << "Intrinsic retrieved\n" << posedet.cameraIntrinsic_ <<  std::endl;
    std::cout << "Distortion retrieved\n" << posedet.distCoeff_ <<  std::endl;
    
    const float fx = 615.3594360351562;
    const float fy = 615.5988159179688;
    const float ppx = 323.4178161621094;
    const float ppy = 248.9889831542969;
    const float coeff[5] = {0,0,0,0,0};
    posedet.fillIntrinsic(ppx, ppy, fx, fy, coeff);
    std::cout << "Intrinsic filled\n" << posedet.cameraIntrinsic_ <<  std::endl;
    std::cout << "Distortion filled\n" << posedet.distCoeff_ <<  std::endl;
    
    for (;;)
    {    
        rs2::frameset frames = p->wait_for_frames();
        rs2::video_frame color = frames.get_color_frame();
        
        cv::Mat imageIn(cv::Size(640, 480), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

        cv::cvtColor(imageIn,imageIn, cv::COLOR_BGR2RGB);
        auto newPose = posedet.poseUpdate(imageIn);

        for(auto& singleMarker: posedet.outPoses_)
        {   
            std:: cout << "ID: " << singleMarker.first << "\nBOOL: " << singleMarker.second.first << "\nPOSE:\n" << singleMarker.second.second.matrix() << std::endl;
        }  


        cv::imshow("Realsense", imageIn);
        if (cv::waitKey(5) >= 0)
            break;
        
    }

    return EXIT_SUCCESS ;
    
}