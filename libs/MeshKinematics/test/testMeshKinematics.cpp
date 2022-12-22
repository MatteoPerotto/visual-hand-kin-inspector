#include<iostream>
#include<meshKinematics.h>
#include<Eigen/Dense>

// ~/robotology-superbuild/src/icub-models/iCub/robots/left_wrist_mk2/model.urdf 

// Main just to test the program
int main(int argc, char* argv[])
{
  // The following will be the inputs to the class
  MeshKinematics mkObject(argv[1]);   // The path to the urdf model (till now is given as argv)

  std::unordered_map<std::string,double> coord;
  for(int it=0; it<mkObject.dofs_; it++)
  {
    coord[mkObject.dofList_[it]] = 0.0 + it;
  }

  for(int i=0; i<2; i++)
  {
    std::cout << "########### ITERATION " << i+1 << " ###########" << std::endl;
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> meshTransform;
    meshTransform = mkObject.updateConfiguration(coord); 
    std::cout << meshTransform.size() << std::endl;
    for(int i=0; i<mkObject.nLinks_; i++){
      std::cout << meshTransform[i].matrix() << std::endl;
      std::cout << "\n" << std::endl;
    }

    for(int it=0; it<mkObject.dofs_; it++)
    {
      coord[mkObject.dofList_[it]] = coord[mkObject.dofList_[it]]+it;
    }
  }
  
}