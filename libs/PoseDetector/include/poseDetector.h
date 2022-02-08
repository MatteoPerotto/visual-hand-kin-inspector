#ifndef POSE_DETECTOR
#define POSE_DETECTOR

#include<Eigen/Geometry>
#include<opencv4/opencv2/core.hpp>
#include<iostream>
#include<librealsense2/rs.hpp> 
#include<opencv4/opencv2/aruco.hpp>
#include<unordered_map>

class PoseDetector
{
    public:
    PoseDetector( std::vector<int>& markerIds, const int& dict, const float& markerMetersSize, 
                    Eigen::Transform<double,3,Eigen::Affine> cameraExt = Eigen::Transform<double,3,Eigen::Affine>::Identity(), 
                    Eigen::MatrixXd cameraInt = Eigen::MatrixXd::Zero(3,3), 
                    Eigen::MatrixXd distCoeff = Eigen::MatrixXd::Zero(5,1)
                );
    ~PoseDetector();
    void printMarker();
    std::unordered_map<int, std::pair<bool,Eigen::Transform<double,3,Eigen::Affine>>> markerPoseUpdate(cv::Mat & currentFrame);
    std::unordered_map<int, std::pair<bool,Eigen::Transform<double,3,Eigen::Affine>>> markerBoardUpdate(cv::Mat& currentFrame);
    void fillIntrinsic(const float& ppx, const float& ppy, const float& fx, const float& fy, const float (&coeff)[5]);
    void getIntrinsic(std::shared_ptr<rs2::pipeline> p);
    void defineFixedT(std::unordered_map<int,Eigen::Transform<double,3,Eigen::Affine>> markerFixedTransform);

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Mat markerImage_;

    Eigen::MatrixXd cameraIntrinsic_; 
    Eigen::Transform<double,3,Eigen::Affine> cameraEstrinsic_;
    Eigen::MatrixXd distCoeff_;
    std::vector<int> foundMarkerIds_;

    std::unordered_map<int, std::pair<bool,Eigen::Transform<double,3,Eigen::Affine>>> outPoses_;
    
    std::unordered_map<int,Eigen::Transform<double,3,Eigen::Affine>> markerFixedTransform_;
    private:
    bool areIntrisicInit_  = false;
    float markerSize_;
    

};

#endif //POSE_DETECTOR