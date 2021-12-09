#include<iostream>
#include<meshKinematics.h>
#include<Eigen/Dense>

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

  for(int i=0; i<2; i++)
  {
    std::cout << "########### ITERATION " << i+1 << " ###########" << std::endl;
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> meshTransform;
    meshTransform = mkObject.updateConfiguration(q);
    std::cout << meshTransform.size() << std::endl;
    for(int i=0; i<mkObject.nLinks_; i++){
      std::cout << meshTransform[i].matrix() << std::endl;
      std::cout << "\n" << std::endl;
    }

    //q0 is x, 
    q[0] = q[0];
    q[1] = q[1];
    q[2] = q[2]+3.14;
  }
  
}