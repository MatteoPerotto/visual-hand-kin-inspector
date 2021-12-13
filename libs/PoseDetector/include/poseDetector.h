#ifndef POSE_DETECTOR
#define POSE_DETECTOR

#include<Eigen/Geometry>
#include<opencv4/opencv2/core.hpp>
#include<iostream>
#include<librealsense2/rs.hpp> 
#include<opencv4/opencv2/aruco.hpp>

class PoseDetector
{
    public:
    PoseDetector( Eigen::Transform<double,3,Eigen::Affine> cameraExt, 
                    Eigen::MatrixXd cameraInt = Eigen::MatrixXd::Zero(3,3), 
                    Eigen::MatrixXd distCoeff = Eigen::MatrixXd::Zero(5,1)
                );
    ~PoseDetector();
    void printMarker();
    std::pair<bool,Eigen::Transform<double,3,Eigen::Affine>> poseUpdate(cv::Mat & currentFrame);
    void fillIntrinsic(const float& ppx, const float& ppy, const float& fx, const float& fy, const float (&coeff)[5]);
    void getIntrinsic(std::shared_ptr<rs2::pipeline> p);

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Mat markerImage_;

    Eigen::MatrixXd cameraIntrinsic_; 
    Eigen::Transform<double,3,Eigen::Affine> cameraEstrinsic_;
    Eigen::MatrixXd distCoeff_;
    std::vector<int> foundMarkerIds_;

    private:
    bool areIntrisicInit_  = false;
    bool areCoeffInit_ = false;
};

#endif //POSE_DETECTOR