/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iDynTree/Core/EigenHelpers.h>
#include <iostream>
#include <iDynTree/Model/SolidShapes.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>
#include <meshKinematics.h>

// Constructor
MeshKinematics::MeshKinematics(const std::string& filePath){

  iDynTree::FrameIndex frameIndex;
  iDynTree::ModelLoader mdlLoader;
  iDynTree::Model model;
  iDynTree::ModelSolidShapes solidshapeObj;
  std::vector<std::vector<iDynTree::SolidShape*>> solidshapeVector;
  iDynTree::Transform visualH;

  // Load the modl in model loader object
  if(!mdlLoader.loadModelFromFile(filePath)) { throw( std::runtime_error("Impossible to load model from " + filePath )); }
  model = mdlLoader.model();
  nLinks_ = model.getNrOfLinks();

  // Define the KinDynComputations object to perform kinematic computation
  if(!compModel_.loadRobotModel(model)) { throw( std::runtime_error("Impossible to create iKinDynComp object " )); }
  dofs_ = compModel_.getNrOfDegreesOfFreedom();

  // Create objects for shape
  solidshapeObj = model.visualSolidShapes();
  solidshapeVector = solidshapeObj.getLinkSolidShapes();

  for(frameIndex=0; frameIndex<nLinks_; frameIndex++){

    // Push the frame name in a private vector container
    std::string currentTargetFrame = model.getFrameName(frameIndex);
    frames_.push_back(currentTargetFrame);

    // Save the fixed transformations between link and geometry refrence frames
    visualTransform_.push_back(solidshapeVector[frameIndex][0]->getLink_H_geometry());

    // Insert the frame name and path inside the urdfPath_ class container
    meshPath_.push_back(make_pair(currentTargetFrame, solidshapeVector[frameIndex][0]->asExternalMesh()->getFileLocationOnLocalFileSystem()));

  }

}

// Destructor
MeshKinematics::~MeshKinematics(){
}

// Update method - it returns the transformations of all visual reference frames given the change in dof
std::vector<Eigen::Transform<double, 3, Eigen::Affine>> MeshKinematics::updateConfiguration(const Eigen::VectorXd& eigenCoord){

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
