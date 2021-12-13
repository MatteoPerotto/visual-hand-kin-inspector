#include<iostream>
#include<visualKinInspector.h>
#include<opencv4/opencv2/core.hpp>
#include<opencv4/opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<meshKinematics.h>
#include<Eigen/Dense>
#include<poseDetector.h>
#include<numeric>
#include<meshSuperimposer.h>
#include<thread>
#include<chrono>

// yarp connect /depthCamera/rgbImage:o /imageStreamer/P:i
// yarp connect /imageStreamer/P:o /yarpview/img:i
// ~/robotology-superbuild/src/icub-models/iCub/robots/left_wrist_mk2/model.urdf

int main(int arc, char** argv){

    // Open communication
    //std::string portsPrefix = "P";
    //ImageStreamer imgStr(portsPrefix); 

    // Initialize the mesh kinematic calculator 
    MeshKinematics meshKinObject(argv[1]); 
    auto meshPaths = meshKinObject.meshPath_;

    // Print urdf mesh paths
    std::cout << "\nFound the following meshes:\n" << std::endl;
    int s = meshPaths.size();
    for(int i =0;i<s;i++){
        std::cout << meshPaths[i].first << " : " << meshPaths[i].second << std::endl;
    }
    
    // Here read from encoders in simulation
    // Eigen::VectorXd q(meshKinObject.dofs_);

    const size_t samples = 300;

    Eigen::MatrixXd m(samples,meshKinObject.dofs_);
    m = Eigen::MatrixXd::Zero(samples, meshKinObject.dofs_);
    m.col(2) = Eigen::VectorXd::LinSpaced(samples, 0, 3.14/2);

    // Fill extrinsic (identity transformation)
    Eigen::Transform<double,3,Eigen::Affine> extP;
    extP = Eigen::Transform<double,3,Eigen::Affine>::Identity();

    // Define intrinsic
    const float fx = 615.3594360351562;
    const float fy = 615.5988159179688;
    const float ppx = 323.4178161621094;
    const float ppy = 248.9889831542969;
    const float coeff[5] = {0,0,0,0,0};

    // Initialize pose detector object 
    PoseDetector poseDet(extP);
    // Fill intrinsic
    poseDet.fillIntrinsic(ppx, ppy, fx, fy, coeff);

    // Initilaize superimposer 
    Eigen::Matrix3d intP;
    intP(0, 0) = fx;
    intP(0, 1) = 0;
    intP(0, 2) = ppx;
    intP(1, 0) = 0;
    intP(1, 1) = fy;
    intP(1, 2) = ppy;
    intP(2, 0) = 0;
    intP(2, 1) = 0;
    intP(2, 2) = 1;

    MeshSuperimposer mSup(meshPaths, extP, intP, 640, 480);

    //cv::VideoWriter video("d1.avi", cv::VideoWriter::fourcc('M','J','P','G'), 30, cv::Size(640,480));
    
    cv::VideoCapture cap("/home/matteoperotto/Documents/outcpp1.avi");

    for(int i=0;i<samples;i++)
    {   
        // Read the image 
        cv::Mat myImg;
        cap >> myImg;
        //myImg = imgStr.readFrame();
        //cv::cvtColor(myImg, myImg, cv::COLOR_BGR2RGB);

   
        // Obtain the trasform of the aruco marker 
        auto newMarkerPose = poseDet.poseUpdate(myImg);
        
        // Update the position of the mesh in world RF
        std::vector<Eigen::Transform<double, 3, Eigen::Affine>> meshTransform;
        meshTransform = meshKinObject.updateConfiguration(m.row(i));
         
        if(newMarkerPose.first)
        {

            for(int j=0; j<meshTransform.size(); j++){
                meshTransform[j] = newMarkerPose.second*meshTransform[j];
            }

            cv::Mat outImg = mSup.meshSuperimpose(meshTransform,myImg);
            //video.write(outImg);
            cv::imshow("Webcam source", outImg);
            std::this_thread::sleep_for (std::chrono::milliseconds(250));
            if (cv::waitKey(5) >= 0)
                break;
        }    
    }
    //video.release();
    cap.release();
}