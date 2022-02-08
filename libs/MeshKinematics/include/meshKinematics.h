/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef KIN_FROM_DESC
#define KIN_FROM_DESC

#include<Eigen/Core>
#include<Eigen/Dense>
#include<iDynTree/KinDynComputations.h>
#include<iostream>

class MeshKinematics
{
  public:
    MeshKinematics(const std::string& filePath);
    ~MeshKinematics();
    std::vector<Eigen::Transform<double, 3, Eigen::Affine>> updateConfiguration(const Eigen::VectorXd& eigenCoord);
    std::size_t dofs_;
    std::size_t nLinks_;
    std::vector<std::pair<std::string,std::string>> meshPath_;
    std::vector<std::string> dofList_;
  private:
    std::vector<std::string> frames_;
    std::vector<iDynTree::Transform> visualTransform_;
    iDynTree::KinDynComputations compModel_;

};

#endif //KIN_FROM_DESC
