/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef KIN_FROM_DESC
#define KIN_FROM_DESC

#include <iostream>
#include <iDynTree/KinDynComputations.h>
#include <Eigen/Core>
#include <Eigen/Dense>

class MeshKinematics
{
public:
  MeshKinematics(const std::string& file_path);
  ~MeshKinematics();
  void updateConfiguration(const Eigen::VectorXd& eigenq);

  std::size_t dofs_;
  std::size_t n_links_;
  std::vector<Eigen::Transform<double, 3, Eigen::Affine>> current_visual_transform_;
  std::vector<std::pair<std::string,std::string>> urdf_path_;
private:
  std::vector<std::string> frames_;
  std::vector<iDynTree::Transform> visual_transform_;
  iDynTree::KinDynComputations comp_model_;
};


#endif
