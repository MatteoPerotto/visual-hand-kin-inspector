#include<Eigen/Geometry>
#include<iostream>
#include<librealsense2/rs.hpp> 
#include<opencv4/opencv2/aruco.hpp>
#include<opencv4/opencv2/core.hpp>
#include<opencv4/opencv2/highgui.hpp>
#include<opencv4/opencv2/imgcodecs.hpp>
#include<opencv4/opencv2/imgproc/imgproc.hpp>
#include<opencv4/opencv2/videoio.hpp>
#include<poseDetector.h>
#include<stdio.h>

// Constructor 
PoseDetector::PoseDetector(Eigen::Transform<double,3,Eigen::Affine> cameraExt)
{
    // Initialize extrinsic parameters 
    cameraEstrinsic_ = cameraExt;

    // Initialize intrinsic parameters 
    distCoeff_ = cv::Mat::zeros(5, 1, CV_64FC1);
    cameraIntrinsic_ = cv::Mat::eye(3, 3, CV_64FC1);

    // Create the marker 
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::drawMarker(dictionary_, 23, 500, markerImage_, 1);
}

// Destructor 
PoseDetector::~PoseDetector()
{

}

void PoseDetector::printMarker()
{
    cv::imwrite("marker23.png", markerImage_);
}

void PoseDetector::fillIntrinsic(const float& ppx, const float& ppy, const float& fx, const float& fy, const float (&coeff)[5])
{
    // [ADD w AND h]
    cameraIntrinsic_.at<double>(0, 0) = fx ;
    cameraIntrinsic_.at<double>(0, 1) = 0;
    cameraIntrinsic_.at<double>(0, 2) = ppx;
    cameraIntrinsic_.at<double>(1, 0) = 0;
    cameraIntrinsic_.at<double>(1, 1) = fy;
    cameraIntrinsic_.at<double>(1, 2) = ppy;
    cameraIntrinsic_.at<double>(2, 0) = 0;
    cameraIntrinsic_.at<double>(2, 1) = 0;
    cameraIntrinsic_.at<double>(2, 2) = 1;

    distCoeff_.at<double>(1, 0) = coeff[0];
    distCoeff_.at<double>(2, 0) = coeff[1];
    distCoeff_.at<double>(3, 0) = coeff[2];
    distCoeff_.at<double>(4, 0) = coeff[3];
    distCoeff_.at<double>(5, 0) = coeff[4];

    std::cout << "Intrinsic filled\n" << cameraIntrinsic_ <<  std::endl;
    std::cout << "Distortion filled\n" << cameraIntrinsic_ <<  std::endl;
}

void PoseDetector::getIntrinsic(std::shared_ptr<rs2::pipeline> p)
{
    p->start();
    auto const i = p->get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    
    imgW_ = i.width;
    imgH_ = i.height;

    cameraIntrinsic_.at<double>(0, 0) = i.fx ;
    cameraIntrinsic_.at<double>(0, 1) = 0;
    cameraIntrinsic_.at<double>(0, 2) = i.ppx;
    cameraIntrinsic_.at<double>(1, 0) = 0;
    cameraIntrinsic_.at<double>(1, 1) = i.fy;
    cameraIntrinsic_.at<double>(1, 2) = i.ppy;
    cameraIntrinsic_.at<double>(2, 0) = 0;
    cameraIntrinsic_.at<double>(2, 1) = 0;
    cameraIntrinsic_.at<double>(2, 2) = 1;

    distCoeff_.at<double>(1, 0) = i.coeffs[0];
    distCoeff_.at<double>(2, 0) = i.coeffs[1];
    distCoeff_.at<double>(3, 0) = i.coeffs[2];
    distCoeff_.at<double>(4, 0) = i.coeffs[3];
    distCoeff_.at<double>(5, 0) = i.coeffs[4];

    std::cout << "Intrinsic retrieved\n" << cameraIntrinsic_ <<  std::endl;
    std::cout << "Distortion retrieved\n" << distCoeff_ <<  std::endl;

}

cv::Mat PoseDetector::poseUpdate(cv::Mat& currentFrame)
{
    // [CADD MARKER DIM INTEAD OF 0.02]
    cv::Mat outFrame;
    currentFrame.copyTo(outFrame);

    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers(currentFrame, dictionary_, markerCorners, foundMarkerIds_, parameters, rejectedCandidates);
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.02, cameraIntrinsic_, distCoeff_, rvecs_, tvecs_);
    cv::aruco::drawDetectedMarkers(outFrame, markerCorners, foundMarkerIds_);

    for (int i = 0; i < rvecs_.size(); ++i) 
    {
        auto rvec = rvecs_[i];
        auto tvec = tvecs_[i];
        cv::aruco::drawAxis(outFrame, cameraIntrinsic_, distCoeff_, rvec, tvec, 0.1);
    }

    std::cout << "\nCurrent translation:" << tvecs_[0] << std::endl;

    return outFrame;
}

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