#include<iostream>
#include <meshKinematics.h>
#include <Eigen/Dense>

// ~/robotology-superbuild/src/icub-models/iCub/robots/left_wrist_mk2/model.urdf 

// Main just to test the program
int main(int argc, char* argv[])
{

  // The following will be the inputs to the class
  MeshKinematics mkObject(argv[1]);   // The path to the urdf model (till now is given as argv)
  Eigen::VectorXd q(mkObject.dofs_);  // The value of the generalized coordinates
  q[0] = 0;
  q[1] = 0;
  q[2] = 0;
  for(int i=0; i<20; i++){
    std::cout << "########### ITERATION ########### " << i+1 << std::endl;
    mkObject.updateConfiguration(q);
    q[0] = q[0]+0.001;
    q[1] = q[1]+0.02;
    q[2] = q[2]-0.1;
  }

}