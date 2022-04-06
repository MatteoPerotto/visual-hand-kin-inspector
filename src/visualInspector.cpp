#include<visualInspector.h>
#include<string>
#include<yarp/os/Bottle.h>
#include<yarp/sig/Image.h>
#include<yarp/cv/Cv.h>
#include<realsenseYarp.h>
#include<opencv4/opencv2/core.hpp>
#include<opencv4/opencv2/highgui.hpp>
#include <math.h> 

bool VisualInspector::configure(yarp::os::ResourceFinder& rf) 
{   
    int configLoaded = 0;

    // Open rgb input port and obtain intinsic parameters 
    if(rf.check("inPortName") && rf.check("outPortName") && rf.check("portPrefix")  )
    {
        std::string inPortName = rf.find("inPortName").asString();
        std::string outPortName = rf.find("outPortName").asString();
        std::string portPrefix = rf.find("portPrefix").asString();
        if (!(rgbPortIn_.open("/" + portPrefix + "/" + inPortName)))
        {
            std::cerr << "[ERROR] Cannot open an imageStreamer input port." << std::endl;
            return false;
        }

        if (!(rgbPortOut_.open("/" + portPrefix + "/" + outPortName)))
        {
            std::cerr << "[ERROR] Cannot open an imageStreamer output port." << std::endl;
            return false;
        }

        RealsenseYarp rsy(portPrefix);
        intMatrix_ = rsy.getIntrinsicMatrix(); 
        imgSize_ = rsy.getImgSize();
    }
    else
    {
        std::cerr << "[ERROR] Cannot load port parameters from configuration file." << std::endl;
        return false;
    }

    // Initialize mesh kinematic from URDF        
    if(rf.check("urdfPath")) 
    {
        std::string urdfPath = rf.find("urdfPath").asString();
        ptrMkObject_ = std::make_unique<MeshKinematics>(urdfPath);
        mPath_ = ptrMkObject_->meshPath_;
        std::cout << "\nFound the following MESHES:" << std::endl;
        for(int i =0;i<mPath_.size();i++)
        {
            std::cout << mPath_[i].first << " : " << mPath_[i].second << std::endl;
        }
    }
    else
    {
        std::cerr << "[ERROR] Cannot load urdfPath parameter from configuration file." << std::endl;
        return false;
    } 

    if(rf.check("robotName") && rf.check("robotPart"))
    {
        std::string robotName = rf.find("robotName").asString();
        std::string robotPart = rf.find("robotPart").asString();
        ptrEncRead_ = std::make_unique<EncoderReader>(robotName, robotPart, ptrMkObject_->dofList_);
    }
    else
    {
        std::cerr << "[ERROR] Cannot load robotName and robotPart parameters from configuration file." << std::endl;
        return false;
    }

    ptrMeshSup_ = std::make_unique<MeshSuperimposer>(mPath_, intMatrix_, imgSize_.first, imgSize_.second);
   
    /* Load pose detector */
    if(rf.check("boardDict") && rf.check("boardN")) //4
    {   
        int dict = rf.find("boardDict").asInt32();
        ptrPoseDet_ = std::make_unique<PoseDetector>(dict, intMatrix_);
        
        int boardNum = rf.find("boardN").asInt32();
        for(int i=1; i<boardNum+1; i++)
        {   
            yarp::os::ResourceFinder rfBoard = rf.findNestedResourceFinder("board"+std::to_string(i)); 
            int boardConfigLoaded = 0;
            int boardId, markerX, markerY;
            double markerSize, markerSep;
            std::vector<int> markerIds;
            std::vector<double> traslCoeff;
            std::vector<double> rotCoeff;
            Eigen::Vector3d T;
            Eigen::Matrix3d R;
            
            if(rfBoard.check("bId")) //board 1
            {
                boardId = rfBoard.find("bId").asInt32();
                markerIds_.push_back(boardId);
                boardConfigLoaded++;
            }
            
            if(rfBoard.check("mX")) //board 2
            {
                markerX = rfBoard.find("mX").asInt32();
                boardConfigLoaded++;
            }

            if(rfBoard.check("mY")) //board 3
            {
                markerY = rfBoard.find("mY").asInt32();
                boardConfigLoaded++;
            }

            if(rfBoard.check("mSize")) //board 4
            {
                markerSize = rfBoard.find("mSize").asFloat64();
                boardConfigLoaded++;
            }

            if(rfBoard.check("mSep")) //board 5
            {
                markerSep = rfBoard.find("mSep").asFloat64();
                boardConfigLoaded++;
            }

            if(rfBoard.check("mId")) //board 6
            {   
                size_t size = markerX*markerY;
                markerIds = retrieveIntList(rfBoard,"mId",size);
                boardConfigLoaded++;
            }

            if(rfBoard.check("fixTrasl")) //board 7
            {   
                traslCoeff = retrieveDoubleList(rfBoard,"fixTrasl",3);
                T(0) = traslCoeff[0];
                T(1) = traslCoeff[1];
                T(2) = traslCoeff[2];
                boardConfigLoaded++;
            }

            if(rfBoard.check("fixRot")) //board 8
            {   
                rotCoeff = retrieveDoubleList(rfBoard,"fixRot",9);
                R(0,0) = rotCoeff[0];
                R(0,1) = rotCoeff[1];
                R(0,2) = rotCoeff[2];
                R(1,0) = rotCoeff[3];
                R(1,1) = rotCoeff[4];
                R(1,2) = rotCoeff[5];
                R(2,0) = rotCoeff[6];
                R(2,1) = rotCoeff[7];
                R(2,2) = rotCoeff[8];
                boardConfigLoaded++;
            }

            if(boardConfigLoaded != 8)
            {   
                std::cerr << "[ERROR] Number of parameters for board group in configuration file is wrong." << std::endl;
                return false;
            }
            else
            {   
                Eigen::Transform<double, 3, Eigen::Affine> boardFixedT;
                boardFixedT = Eigen::Translation<double,3>(T);
                boardFixedT.rotate(R);
                ptrPoseDet_->addBoard(boardId,markerX,markerY,markerSize,markerSep,markerIds,boardFixedT.inverse());
            } 
        }
        ptrPoseDet_->printBoardInfo();
        ptrPoseDet_->printBoards();
    }
    else
    {   
        std::cerr << "[ERROR] Cannot load board parameters from configuration file." << std::endl;
        return false;
    }
     
    if(rf.check("scaleF")) //5
    {
        scaleFactor_ = rf.find("scaleF").asInt32();
    }
    else
    {
        std::cerr << "[ERROR] Cannot load scaleF parameter from configuration file." << std::endl;
        return false; 
    }

    return true;
}


double VisualInspector::getPeriod()
{   
    return 0.0001;
}


bool VisualInspector::updateModule()
{      
    /* Read image */
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* imgIn;
    cv::Mat imgInCv;
    cv::Mat outImg;

    imgIn = rgbPortIn_.read();
    imgInCv = yarp::cv::toCvMat(*imgIn);

    /* Obtain the trasform of each aruco marker */
    auto newBoardsPose = ptrPoseDet_->poseUpdate(imgInCv);


    /* Extract encoder readings */
    auto encoderSignal = ptrEncRead_->readEncoders();


    /* Update the position of the mesh in world RF */
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> meshTransform;
    meshTransform = ptrMkObject_->updateConfiguration(encoderSignal); 

    /* "Mean" between homogeneous transformations - till now only on xyz */ 

    Eigen::Array3d avgT;
    Eigen::Matrix3d avgR = newBoardsPose[1].second.rotation();
    int n=0;

    for(auto& T : markerIds_)
    {
       if(newBoardsPose[T].first==true)
       {    
           avgT = (avgT*n + newBoardsPose[T].second.translation().array())/(n+1);
           n++;
       }
    } 

    fixedT_ = Eigen::Translation<double,3>(avgT);
    fixedT_.rotate(avgR);
    
    for(int j=0; j<meshTransform.size(); j++)
    {
        meshTransform[j] = fixedT_*meshTransform[j]; 
        meshTransform[j].translation()*= scaleFactor_;
    }

    // Create output image 
    outImg = ptrMeshSup_->meshSuperimpose(meshTransform,imgInCv);
    auto outImgYarp = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(outImg); 
    auto& outImgRef = rgbPortOut_.prepare();
    outImgRef = outImgYarp;
    
    rgbPortOut_.write();

    return true;
}

bool VisualInspector::close()
{
    return true;
}

std::vector<int> VisualInspector::retrieveIntList(yarp::os::ResourceFinder rf, std::string key, std::size_t size)
{
    std::vector<int> markerIds(size);
    yarp::os::Bottle* idsBottle = rf.find(key).asList();

    if(idsBottle == nullptr)
    {
        std::cerr << "[ERROR] Number of given ids in configuration file is wrong." << std::endl;
    }
    else
    {   
        for(std::size_t i=0; i<size; i++)
        {
            yarp::os::Value value = idsBottle->get(i);
            markerIds[i] = value.asInt32();
        }                 
    } 
    idsBottle->clear();
    return markerIds;
}

std::vector<double> VisualInspector::retrieveDoubleList(yarp::os::ResourceFinder rf, std::string key, std::size_t size)
{
    std::vector<double> markerIds(size);
    yarp::os::Bottle* idsBottle = rf.find(key).asList();

    if(idsBottle == nullptr)
    {
        std::cerr << "[ERROR] Number of given ids in configuration file is wrong." << std::endl;
    }
    else
    {   
        for(std::size_t i=0; i<size; i++)
        {
            yarp::os::Value value = idsBottle->get(i);
            markerIds[i] = value.asFloat64();
        }                 
    } 
    idsBottle->clear();
    return markerIds;
}

