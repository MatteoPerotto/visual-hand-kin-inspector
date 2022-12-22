#include<iostream>
#include<yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include<visualInspector.h>

// yarp connect /depthCamera/rgbImage:o /imageStreamer/RGB:i
// yarp connect /imageStreamer/RGB:o /yarpview/img:i

// ~/robotology-superbuild/src/icub-models/iCub/robots/left_wrist_mk2/model.urdf
// /home/matteoperotto/visual-hand-kin-inspector/res/model.urdf
// yarpmotorgui --from /home/matteoperotto/robotology-superbuild/src/icub-models/iCub_manual/conf_manual/iCubGazeboV2_5_visuomanip/gazebo_icub_left_hand.ini

// yarpmotorgui --robot icubSim

// ./bin/VisualKinInspector --from ../config/config.ini 
// yarprobotinterface --config /home/matteoperotto/Desktop/experimentalSetups/wristmk2_handmk3/new_forearm-no_wrist.xml


int main(int argc, char** argv)
{
    // Open communication
    yarp::os::Network yarpNetwork;

    // Chech network
    if (!yarpNetwork.checkNetwork())
    {
        throw(std::runtime_error("[ERROR] YARP network is not available."));
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc, argv);

    VisualInspector vInsp;
    return vInsp.runModule(rf); 
  
}