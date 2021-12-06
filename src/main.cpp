#include<iostream>
#include<visualKinInspector.h>
#include<opencv4/opencv2/core.hpp>
#include<opencv2/highgui.hpp>


int main(int arc, char** argv){

    std::string portPrefix = "P";
    ImageStreamer imgStr(portPrefix); 

    for(;;)
    {   
        cv::Mat myImg;
        myImg = imgStr.readFrame();

        /*cv::imshow("Webcam source", myImg);
        if (cv::waitKey(5) >= 0)
            break;
        */
        imgStr.sendFrame(myImg);
    }
    
}