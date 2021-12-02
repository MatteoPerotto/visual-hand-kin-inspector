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
MeshKinematics::MeshKinematics(const std::string& file_path){

  iDynTree::FrameIndex frameIndex;
  iDynTree::ModelLoader mdlLoader;
  iDynTree::Model model;
  iDynTree::ModelSolidShapes solidshape_obj;
  std::vector<std::vector<iDynTree::SolidShape*>> solidshape_vector;
  iDynTree::Transform visualH;

  // Load the modl in model loader object
  if(!mdlLoader.loadModelFromFile(file_path)) { throw( std::runtime_error("Impossible to load model from " + file_path )); }
  model = mdlLoader.model();
  n_links_ = model.getNrOfLinks();

  // Define the KinDynComputations object to perform kinematic computation
  if(!comp_model_.loadRobotModel(model)) { throw( std::runtime_error("Impossible to create iKinDynComp object " )); }
  dofs_ = comp_model_.getNrOfDegreesOfFreedom();

  // Create objects for shape
  solidshape_obj = model.visualSolidShapes();
  solidshape_vector = solidshape_obj.getLinkSolidShapes();

  for(frameIndex=0; frameIndex<n_links_; frameIndex++){

    // Push the frame name in a private vector container
    std::string current_target_frame = model.getFrameName(frameIndex);
    frames_.push_back(current_target_frame);

    // Save the fixed transformations between link and geometry refrence frames
    visual_transform_.push_back(solidshape_vector[frameIndex][0]->getLink_H_geometry());

    // Insert the frame name and path inside the urdf_path_ class container
    urdf_path_.push_back(make_pair(current_target_frame, solidshape_vector[frameIndex][0]->asExternalMesh()->getFileLocationOnLocalFileSystem()));

  }

}

// Destructor
MeshKinematics::~MeshKinematics(){

}

// Update method - it returns the transformations of all visual reference frames given the change in dof
void MeshKinematics::updateConfiguration(const Eigen::VectorXd& eigenq){

  iDynTree::FrameIndex frameIndex;
  iDynTree::VectorDynSize idynq(dofs_);
  toEigen(idynq) = eigenq;
  comp_model_.setJointPos(idynq);

  for(frameIndex=0; frameIndex<n_links_; frameIndex++){

    iDynTree::Transform idyntree_current_visual_transform = comp_model_.getWorldTransform(frames_[frameIndex])*visual_transform_[frameIndex];
    Eigen::Transform<double, 3, Eigen::Affine> eigen_transform;
    eigen_transform.matrix() = toEigen(idyntree_current_visual_transform.asHomogeneousTransform());
    current_visual_transform_.push_back(eigen_transform);

    std::cout << "\nFrame: " << frames_[frameIndex] << std::endl;
    std::cout << eigen_transform.matrix() << std::endl;

  }

}
