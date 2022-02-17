#include<iostream>
#include<mk2EncoderReader.h>
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

#include<yarp/os/Network.h>
#include<yarp/cv/Cv.h>
#include<yarp/os/BufferedPort.h>
#include<yarp/sig/Image.h>

// yarp connect /depthCamera/rgbImage:o /imageStreamer/P:i
// yarp connect /imageStreamer/P:o /yarpview/img:i

// ~/robotology-superbuild/src/icub-models/iCub/robots/left_wrist_mk2/model.urdf
// /home/matteoperotto/visual-hand-kin-inspector/res/model.urdf
// yarpmotorgui --from /home/matteoperotto/robotology-superbuild/src/icub-models/iCub_manual/conf_manual/iCubGazeboV2_5_visuomanip/gazebo_icub_left_hand.ini

// yarpmotorgui --robot icubSim


int main(int arc, char** argv)
{
    // Open communication
    std::string portsPrefix = "P";
    yarp::os::Network yarpNetwork;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> rgbPortIn;

    // Chech network
    if (!yarpNetwork.checkNetwork())
    {
        throw(std::runtime_error("[ERROR] YARP network is not available."));
    }

    // Open rgb input port
    if (!(rgbPortIn.open("/imageStreamer/" + portsPrefix + ":i")))
    {
        throw(std::runtime_error("[ERROR] Cannot open an imageStreamer input port."));
    }

    // Create images
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* imgIn;
    cv::Mat imgInCv;
    cv::Mat outImg;

    // Initialize the mesh kinematic calculator 
    MeshKinematics meshKinObject(argv[1]); 
    auto meshPaths = meshKinObject.meshPath_;
    auto dofList = meshKinObject.dofList_;
    int scaleFactor = 1000; 
      

    // Print urdf mesh paths
    std::cout << "\nFound the following meshes:\n" << std::endl;
    int s = meshPaths.size();
    for(int i =0;i<s;i++)
    {
        std::cout << meshPaths[i].first << " : " << meshPaths[i].second << std::endl;
    }
    
    // Here read from encoders in simulation
    // EncoderReader encRead("wristMk2Sim","left_wrist");
    // EncoderReader encRead("icub","left_arm",dofList);  
    EncoderReader encRead("icubSim","left_arm",dofList);   

    // Fill extrinsic (identity transformation)
    Eigen::Transform<double,3,Eigen::Affine> extP;
    extP = Eigen::Transform<double,3,Eigen::Affine>::Identity();

    // Define intrinsic
    const float fx = 618.0714111328125;
    const float fy = 617.783447265625;
    const float ppx = 305.902252197265625;
    const float ppy = 246.352935791015625;
    const float coeff[5] = {0,0,0,0,0};

    // Initialize pose detector object 
    PoseDetector poseDet(0);
    std::vector<int> markerIds1 = {0,1,2,3};
    std::vector<int> markerIds2 = {4,5,6,7};
    poseDet.addBoard(0,2,2,0.02,0.005,markerIds1);
    poseDet.addBoard(1,2,2,0.02,0.005,markerIds2);

    // Fill intrinsic 
    poseDet.fillIntrinsic(ppx, ppy, fx, fy, coeff);
    std::cout << "Intrinsic filled\n" << poseDet.cameraIntrinsic_ <<  std::endl;
    std::cout << "Distortion filled\n" << poseDet.distCoeff_ <<  std::endl;

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

    //Define the constant transform for each maker wrt the hand RF
    Eigen::Transform<double, 3, Eigen::Affine> arucoTransform;
    Eigen::Transform<double, 3, Eigen::Affine> fixedTransform2;

    Eigen::Matrix3d R;
    R(0,0) = 0;
    R(0,1) = 0.99995722;
    R(0,2) = 0.00924999;
    R(1,0) = -1;
    R(1,1) = 0;
    R(1,2) = 0;
    R(2,0) = 0;
    R(2,1) = -0.00924999;
    R(2,2) = 0.99995722;

    Eigen::Vector3d T;
    T(0) = 0.0196458;
    T(1) = 0.0243051;
    T(2) = 0.0384987;

    arucoTransform = Eigen::Translation<double,3>(T);
    arucoTransform.rotate(R);
    arucoTransform = arucoTransform.inverse();

    Eigen::Matrix3d R2 = Eigen::AngleAxis<double>(Eigen::AngleAxis<double>(-0.261799319827, Eigen::Vector3d::UnitY())*Eigen::AngleAxis<double>(-1.57079632679, Eigen::Vector3d::UnitX())).toRotationMatrix();  
    Eigen::Vector3d T2;
    T2(0) = 0.0576542978428;
    T2(1) = -0.0055568;
    T2(2) = 0.0136938323083;

    fixedTransform2 = Eigen::Translation<double,3>(T2);
    fixedTransform2.rotate(R2);
    fixedTransform2 = fixedTransform2.inverse();

    Eigen::Vector3d BT;
    BT(0) = -0.004;
    BT(1) = 0.048;
    BT(2) = 0;

    Eigen::Transform<double, 3, Eigen::Affine> cornerTranslation;
    cornerTranslation = Eigen::Translation<double,3>(BT);

    for(;;)
    {   
        // Read the image 
        imgIn = rgbPortIn.read();
        imgInCv = yarp::cv::toCvMat(*imgIn);
        
        // Obtain the trasform of the aruco marker
        auto newMarkerPose = poseDet.poseUpdate(imgInCv);

        // Extract encoder readings 
        auto encoderSignal = encRead.readEncoders();
        
         std::cout << "Heerre" << std::endl;
        // Update the position of the mesh in world RF
        std::vector<Eigen::Transform<double, 3, Eigen::Affine>> meshTransform;
        meshTransform = meshKinObject.updateConfiguration(encoderSignal);
        
        for(int j=0; j<meshTransform.size(); j++)
        {
                meshTransform[j] = newMarkerPose[0].second*cornerTranslation*arucoTransform*fixedTransform2*meshTransform[j]; 
                meshTransform[j].translation()*= scaleFactor;
        }

        cv::Mat outImg = mSup.meshSuperimpose(meshTransform,imgInCv);
        cv::imshow("Webcam source", outImg);
        if (cv::waitKey(5) >= 0)
            break;
    }
    
}