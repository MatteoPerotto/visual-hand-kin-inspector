/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include<Eigen/Core>
#include<Eigen/Dense>
#include<iDynTree/Core/EigenHelpers.h>
#include<iDynTree/Model/SolidShapes.h>
#include<iDynTree/ModelIO/ModelLoader.h>
#include<iDynTree/KinDynComputations.h>
#include<iostream>
#include<meshKinematics.h>


// Constructors

MeshKinematics::MeshKinematics()
{
}

MeshKinematics::MeshKinematics(const std::string& filePath)
{

  iDynTree::FrameIndex frameIndex;
  iDynTree::ModelLoader mdlLoader;
  iDynTree::Model model;
  iDynTree::ModelSolidShapes solidshapeObj;
  std::vector<std::vector<iDynTree::SolidShape*>> solidshapeVector;
  iDynTree::Transform visualH;

  // Load the model in model loader object
  if(!mdlLoader.loadModelFromFile(filePath)) { throw( std::runtime_error("Impossible to load model from " + filePath )); }
  model = mdlLoader.model();
  nLinks_ = model.getNrOfLinks();
  std::cout << "Found the following LINKS: " <<  nLinks_ << std::endl;

  // Define the KinDynComputations object to perform kinematic computation
  if(!compModel_.loadRobotModel(model)) { throw( std::runtime_error("Impossible to create iKinDynComp object " )); }
  dofs_ = compModel_.getNrOfDegreesOfFreedom();
  std::cout << "Found the following DOFS: " << dofs_ << std::endl;

  for(int i=0;i<dofs_;i++ )
  {
    auto jname = model.getJointName(i);
    dofList_.push_back(jname);
    std::cout << "JOINT" << i << ": " << dofList_[i] << std::endl;
  }

  // Create objects for shape
  solidshapeObj = model.visualSolidShapes();
  solidshapeVector = solidshapeObj.getLinkSolidShapes();

  for(frameIndex=0; frameIndex<nLinks_; frameIndex++){

    // Push the frame name in a private vector container
    std::string currentTargetFrame = model.getFrameName(frameIndex);
    frames_.push_back(currentTargetFrame);

    if(solidshapeVector[frameIndex].size()==0)
    {
      visualTransform_.push_back(iDynTree::Transform::Identity());
      meshPath_.push_back(make_pair(currentTargetFrame, " "));
    }
    else
    {
      for(int visualIndex=0; visualIndex<solidshapeVector[frameIndex].size(); visualIndex++)
      { 
        visualTransform_.push_back(solidshapeVector[frameIndex][visualIndex]->getLink_H_geometry());
        meshPath_.push_back(make_pair(currentTargetFrame, solidshapeVector[frameIndex][visualIndex]->asExternalMesh()->getFileLocationOnLocalFileSystem()));
      }
    }

  }

}

// Destructor
MeshKinematics::~MeshKinematics()
{

}

// Update method - it returns the transformations of all visual reference frames given the change in dof
std::vector<Eigen::Transform<double, 3, Eigen::Affine>> MeshKinematics::updateConfiguration(const Eigen::VectorXd& eigenCoord)
{

  std::vector<Eigen::Transform<double, 3, Eigen::Affine>> updatedPose;
  iDynTree::FrameIndex frameIndex;
  iDynTree::VectorDynSize idynq(dofs_);
  toEigen(idynq) = eigenCoord;
  compModel_.setJointPos(idynq);

  for(frameIndex=0; frameIndex<nLinks_; frameIndex++){

    iDynTree::Transform idyntreeCurrentVisualTransform = compModel_.getWorldTransform(frames_[frameIndex])*visualTransform_[frameIndex];
    Eigen::Transform<double, 3, Eigen::Affine> eigenTransform;
    eigenTransform.matrix() = toEigen(idyntreeCurrentVisualTransform.asHomogeneousTransform());
    updatedPose.push_back(eigenTransform);

  }

  return updatedPose;

}
