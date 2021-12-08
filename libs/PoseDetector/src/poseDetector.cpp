#include<Eigen/Geometry>
#include<iostream>
#include<librealsense2/rs.hpp> 
#include<opencv4/opencv2/aruco.hpp>
#include<opencv4/opencv2/core.hpp>
#include<opencv4/opencv2/core/eigen.hpp>
#include<opencv4/opencv2/highgui.hpp>
#include<opencv4/opencv2/imgcodecs.hpp>
#include<opencv4/opencv2/imgproc/imgproc.hpp>
#include<opencv4/opencv2/videoio.hpp>
#include<opencv4/opencv2/calib3d.hpp>
#include<poseDetector.h>


// Constructor 
PoseDetector::PoseDetector( Eigen::Transform<double,3,Eigen::Affine> cameraExt, Eigen::MatrixXd cameraInt, Eigen::MatrixXd distCoeff )
{
    // Initialize extrinsic parameters 
    cameraEstrinsic_ = cameraExt;

    // Initialize intrinsic parameters 
    if(cameraInt.size()!=0)
    {
        cameraIntrinsic_ = cameraInt;
        areIntrisicInit_ = true;
    }
    
    if(distCoeff.size()!=0)
    {
        distCoeff_ = distCoeff;
        areCoeffInit_ = true;
    }

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
    cameraIntrinsic_(0, 0) = fx;
    cameraIntrinsic_(0, 1) = 0;
    cameraIntrinsic_(0, 2) = ppx;
    cameraIntrinsic_(1, 0) = 0;
    cameraIntrinsic_(1, 1) = fy;
    cameraIntrinsic_(1, 2) = ppy;
    cameraIntrinsic_(2, 0) = 0;
    cameraIntrinsic_(2, 1) = 0;
    cameraIntrinsic_(2, 2) = 1;

    distCoeff_(0, 0) = coeff[0];
    distCoeff_(1, 0) = coeff[1];
    distCoeff_(2, 0) = coeff[2];
    distCoeff_(3, 0) = coeff[3];
    distCoeff_(4, 0) = coeff[4];

    std::cout << "Intrinsic filled\n" << cameraIntrinsic_ <<  std::endl;
    std::cout << "Distortion filled\n" << distCoeff_ <<  std::endl;
}

void PoseDetector::getIntrinsic(std::shared_ptr<rs2::pipeline> p)
{
    p->start();
    auto const i = p->get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();

    cameraIntrinsic_(0, 0) = i.fx ;
    cameraIntrinsic_(0, 1) = 0;
    cameraIntrinsic_(0, 2) = i.ppx;
    cameraIntrinsic_(1, 0) = 0;
    cameraIntrinsic_(1, 1) = i.fy;
    cameraIntrinsic_(1, 2) = i.ppy;
    cameraIntrinsic_(2, 0) = 0;
    cameraIntrinsic_(2, 1) = 0;
    cameraIntrinsic_(2, 2) = 1;

    distCoeff_(0, 0) = i.coeffs[0];
    distCoeff_(1, 0) = i.coeffs[1];
    distCoeff_(2, 0) = i.coeffs[2];
    distCoeff_(3, 0) = i.coeffs[3];
    distCoeff_(4, 0) = i.coeffs[4];

    std::cout << "Intrinsic retrieved\n" << cameraIntrinsic_ <<  std::endl;
    std::cout << "Distortion retrieved\n" << distCoeff_ <<  std::endl;

}

cv::Mat PoseDetector::poseUpdate(cv::Mat& currentFrame)
{
    // [CADD MARKER DIM INSTEAD OF 0.02]

    if(areIntrisicInit_==false && areCoeffInit_==false){
        throw(std::runtime_error("[ERROR] Intrinsic parameters and distortion coefficients must be initialized. Can use the .fillIntrinsic(const float& ppx, const float& ppy, const float& fx, const float& fy, const float (&coeff)[5]) method"));
    }
    cv::Mat outFrame;
    currentFrame.copyTo(outFrame);

    cv::Mat cvIntrinsic;
    cv::Mat cvDistCoeff;
    
    cv::eigen2cv(cameraIntrinsic_,cvIntrinsic);
    cv::eigen2cv(distCoeff_,cvDistCoeff);
    std::vector<cv::Vec3d> rvecs, tvecs;

    Eigen::Matrix3d rotEigen = Eigen::Matrix3d::Zero(3,3);
    Eigen::Vector3d traslEigen = Eigen::Vector3d::Zero(3,1);
    cv::Mat R = cv::Mat::zeros(cv::Size(3, 3), CV_64FC1);
    Eigen::Transform<double,3,Eigen::Affine> homT;
    
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers(currentFrame, dictionary_, markerCorners, foundMarkerIds_, parameters, rejectedCandidates);
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.02, cvIntrinsic, cvDistCoeff, rvecs, tvecs);

    // Easy for one marker, change for the board
    for(int i = 0; i < rvecs.size(); ++i)
    {
        cv::Rodrigues(rvecs[i],R);
        cv::cv2eigen(R,rotEigen);
        cv::cv2eigen(tvecs[i],traslEigen); 
        
        homT = Eigen::Translation<double,3>(traslEigen);
        homT.rotate(rotEigen);

        arucoTransform_.push_back(homT);
            
    }
    arucoTransform_.clear();
     	
    // Draw and output 
    cv::aruco::drawDetectedMarkers(outFrame, markerCorners, foundMarkerIds_);
    for (int i = 0; i < rvecs.size(); ++i) 
    {
        auto rvec = rvecs[i];
        auto tvec = tvecs[i];
        cv::aruco::drawAxis(outFrame, cvIntrinsic, cvDistCoeff, rvec, tvec, 0.1);
    }
    cv::cvtColor(outFrame, outFrame, cv::COLOR_BGR2RGB, 0);

    return outFrame;
}




