#ifndef POSE_DETECTOR
#define POSE_DETECTOR

#include<Eigen/Geometry>
#include<opencv4/opencv2/core.hpp>
#include<iostream>
#include <librealsense2/rs.hpp> 

class PoseDetector
{
    public:
    PoseDetector( Eigen::Transform<double,3,Eigen::Affine> cameraExt);
    ~PoseDetector();
    void printMarker();
    cv::Mat poseUpdate(cv::Mat & currentFrame);
    void fillIntrinsic(const float& ppx, const float& ppy, const float& fx, const float& fy, const float (&coeff)[5]);
    void getIntrinsic(std::shared_ptr<rs2::pipeline> p);

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Mat markerImage_;

    cv::Mat cameraIntrinsic_; 
    Eigen::Transform<double,3,Eigen::Affine> cameraEstrinsic_;
    cv::Mat distCoeff_;
    size_t imgW_;
    size_t imgH_;

    std::vector<int> foundMarkerIds_;
    std::vector<cv::Vec3d> rvecs_, tvecs_;

    private:
};

#endif //POSE_DETECTOR