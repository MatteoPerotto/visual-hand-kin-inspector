#ifndef REALSENSE_YARP
#define REALSENSE_YARP

#include<Eigen/Geometry>
#include<map>
#include<vector>

class RealsenseYarp
{

public:
    RealsenseYarp(std::string portPrefix);
    ~RealsenseYarp();
    std::pair<int,int> getImgSize();
    Eigen::Matrix3d getIntrinsicMatrix();
    std::vector<double> getIntrinsicVector();

    std::pair<int,int>  imgSize_;
    Eigen::Matrix3d intP_;

private:
    double ppx_,ppy_,fx_,fy_;

};


#endif 