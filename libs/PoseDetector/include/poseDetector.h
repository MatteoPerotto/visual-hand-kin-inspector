#ifndef POSE_DETECTOR
#define POSE_DETECTOR

#include<Eigen/Geometry>
#include<opencv4/opencv2/aruco.hpp>
#include<opencv4/opencv2/core.hpp>
#include<unordered_map>

class PoseDetector
{
    public:
    PoseDetector(   int dict,
                    Eigen::MatrixXd cameraInt = Eigen::MatrixXd::Zero(3,3),
                    Eigen::Transform<double,3,Eigen::Affine> cameraExt = Eigen::Transform<double,3,Eigen::Affine>::Identity(),
                    Eigen::MatrixXd distCoeff = Eigen::MatrixXd::Zero(5,1)
                );
    ~PoseDetector();

    void addBoard(const int id, const int X, const int Y, const double markerSize, const double markerSpacing, std::vector<int> markerIds, Eigen::Transform<double,3,Eigen::Affine> boardFixedTransform = Eigen::Transform<double,3,Eigen::Affine>::Identity());
    std::unordered_map<int, std::pair<bool,Eigen::Transform<double,3,Eigen::Affine>>> poseUpdate(cv::Mat& currentFrame);
    void fillIntrinsic(const float& ppx, const float& ppy, const float& fx, const float& fy, const float (&coeff)[5]);
    // void getIntrinsic(std::shared_ptr<rs2::pipeline> p);
    void printBoards();
    void printBoardInfo();

    Eigen::Transform<double,3,Eigen::Affine> cameraEstrinsic_;
    Eigen::MatrixXd cameraIntrinsic_;
    Eigen::MatrixXd distCoeff_;

    std::unordered_map<int,Eigen::Transform<double,3,Eigen::Affine>> boardFixedTransform_;
    std::unordered_map<int,cv::Ptr<cv::aruco::GridBoard>> boardPtr_;
    std::unordered_map<int,double> markersSize_;

    std::unordered_map<int, std::pair<bool,Eigen::Transform<double,3,Eigen::Affine>>> outPoses_;

    private:
    cv::Ptr<cv::aruco::Dictionary> dict_;
    bool areIntrisicInit_  = false;
    int dictionary_;
    bool areBoardInit_ = false;

};

#endif //POSE_DETECTOR
