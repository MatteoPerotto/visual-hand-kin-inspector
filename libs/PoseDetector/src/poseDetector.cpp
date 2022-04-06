#include<Eigen/Geometry>
#include<librealsense2/rs.hpp> 
#include<opencv4/opencv2/aruco.hpp>
#include<opencv4/opencv2/core.hpp>
#include<opencv4/opencv2/core/eigen.hpp>
#include<opencv4/opencv2/calib3d.hpp>
#include<poseDetector.h>
#include<string>
#include<opencv2/imgcodecs.hpp>


#include<iostream>


// Constructor 
PoseDetector::PoseDetector(int dict, Eigen::MatrixXd cameraInt, Eigen::Transform<double,3,Eigen::Affine> cameraExt,  Eigen::MatrixXd distCoeff )
{
    // Initialize extrinsic parameters 
    cameraEstrinsic_ = cameraExt;

    // Initialize parameters 
    if(cameraInt.size()!=0)
    {
        cameraIntrinsic_ = cameraInt;
        areIntrisicInit_ = true;
    }

    if(cameraExt.matrix().size()!=0)
    {
        cameraEstrinsic_ = cameraExt;
    }
    
    if(distCoeff.size()!=0)
    {
        distCoeff_= distCoeff;
    }     
    
    dictionary_ = dict;
    dict_ = cv::aruco::getPredefinedDictionary(dict);
}

// Destructor 
PoseDetector::~PoseDetector()
{

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
}


void PoseDetector::addBoard(const int id , const int X, const int Y, const double markerSize, const double markerSpacing, std::vector<int> markerIds, Eigen::Transform<double,3,Eigen::Affine> boardFixedTransform )
{   
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(X, Y, markerSize, markerSpacing, dict_);
    board->ids = markerIds;
    boardPtr_[id] = board;
    boardFixedTransform_[id] = boardFixedTransform;
    outPoses_[id] = std::make_pair(false,Eigen::Transform<double,3,Eigen::Affine>::Identity());
    markersSize_[id] = markerSize;
    areBoardInit_ = true;
}

void PoseDetector::printBoardInfo()
{
    for(auto& b: boardPtr_)
    {   
        std::cout << "*** BOARD ***" << std::endl;
        std::cout << "Board: " << b.first << std::endl;  
        std::cout << "Dictionary int: " << dictionary_ << std::endl;
        std::cout << "Dictionary ptr: " << dict_ << std::endl;
        std::cout << "Size: " << b.second->getGridSize() << std::endl;
        std::cout << "MSize: " << b.second->getMarkerLength()  << std::endl;
        std::cout << "Ptr: " << b.second << std::endl;
        std::cout << "FixedT: " << std::endl << boardFixedTransform_[b.first].matrix() << std::endl;
    
        for(auto& i: b.second->ids)
        {
            std::cout << "Id " << i << std::endl;
        } 
    }
}

void PoseDetector::printBoards()
{  
    for(auto& b: boardPtr_)
    {
        cv::Mat boardImage; 
        b.second->draw( cv::Size(236,1087), boardImage);
        std::string name = "board"+std::to_string(b.first)+".png";
        cv::imwrite(name,boardImage);
    }  
    
}

std::unordered_map<int, std::pair<bool,Eigen::Transform<double,3,Eigen::Affine>>> PoseDetector::poseUpdate(cv::Mat& currentFrame)
{   
    
    for(auto& b: outPoses_)
    {   
        b.second.first = false;
    } 

    if(areIntrisicInit_== false )
    {
        throw(std::runtime_error("[ERROR] Intrinsic parameters must be initialized. Can use the .fillIntrinsic(const float& ppx, const float& ppy, const float& fx, const float& fy, const float (&coeff)[5]) method"));
    }

    if(areBoardInit_== false )
    {
        throw(std::runtime_error("[ERROR] No board initialized. Please call: addBoard(const int id , const int X, const int Y, const double markerSize, const double markerSpacing, std::vector<int> markerIds, Eigen::Transform<double,3,Eigen::Affine> boardFixedTransform )"));
    }

    cv::Mat cvCameraIntrinsic; 
    cv::Mat cvDistCoeff;
    cv::eigen2cv(cameraIntrinsic_,cvCameraIntrinsic);
    cv::eigen2cv(distCoeff_,cvDistCoeff);

    std::cout << "Inrinsic" << std::endl << cvCameraIntrinsic << std::endl;

    std::vector<int> foundMarkerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    cv::aruco::detectMarkers(currentFrame, dict_, markerCorners, foundMarkerIds, parameters, rejectedCandidates);
    cv::aruco::drawDetectedMarkers(currentFrame, markerCorners, foundMarkerIds);

    for(auto&  b: boardPtr_)
    {   
        std::cout << b.second << std::endl;
        Eigen::Transform<double,3,Eigen::Affine> homT;
        Eigen::Matrix3d rotEigen = Eigen::Matrix3d::Zero(3,3);
        Eigen::Vector3d traslEigen = Eigen::Vector3d::Zero(3,1);
        cv::Mat R = cv::Mat::zeros(cv::Size(3, 3), CV_64FC1);

        cv::Vec3d rvec, tvec;

        int isDetected = cv::aruco::estimatePoseBoard(markerCorners, foundMarkerIds, b.second, cvCameraIntrinsic, cvDistCoeff, rvec, tvec);

        if(isDetected!=0) 
        {    
            cv::Rodrigues(rvec,R);
            cv::cv2eigen(R,rotEigen);
            cv::cv2eigen(tvec,traslEigen); 

            homT = Eigen::Translation<double,3>(traslEigen);
            homT.rotate(rotEigen);   

            outPoses_[b.first] = std::make_pair(true,homT*boardFixedTransform_[b.first]);            
            std::cout << "Pose board"<< b.first << std::endl <<  outPoses_[b.first].second.matrix() << std::endl;
            cv::aruco::drawAxis(currentFrame, cvCameraIntrinsic, cvDistCoeff, rvec, tvec, 2*markersSize_[b.first]);
        }
    }

    return outPoses_;
}


