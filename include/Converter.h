/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <vector>

#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "sophus/geometry.hpp"
#include "sophus/sim3.hpp"

namespace ORB_SLAM3 {

namespace Converter {

std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
g2o::SE3Quat toSE3Quat(const Sophus::SE3f &T);
g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

// TODO templatize these functions
cv::Mat toCvMat(const g2o::SE3Quat &SE3);
cv::Mat toCvMat(const g2o::Sim3 &Sim3);
cv::Mat toCvMat(const Eigen::Matrix<double, 4, 4> &m);
cv::Mat toCvMat(const Eigen::Matrix<float, 4, 4> &m);
cv::Mat toCvMat(const Eigen::Matrix<float, 3, 4> &m);
cv::Mat toCvMat(const Eigen::Matrix3d &m);
cv::Mat toCvMat(const Eigen::Matrix<double, 3, 1> &m);
cv::Mat toCvMat(const Eigen::Matrix<float, 3, 1> &m);
cv::Mat toCvMat(const Eigen::Matrix<float, 3, 3> &m);

cv::Mat toCvMat(const Eigen::MatrixXf &m);
cv::Mat toCvMat(const Eigen::MatrixXd &m);

cv::Mat toCvSE3(const Eigen::Matrix<double, 3, 3> &R,
                const Eigen::Matrix<double, 3, 1> &t);
cv::Mat tocvSkewMatrix(const cv::Mat &v);

Eigen::Matrix<double, 3, 1> toVector3d(const cv::Mat &cvVector);
Eigen::Matrix<float, 3, 1> toVector3f(const cv::Mat &cvVector);
Eigen::Matrix<double, 3, 1> toVector3d(const cv::Point3f &cvPoint);
Eigen::Matrix<double, 3, 3> toMatrix3d(const cv::Mat &cvMat3);
Eigen::Matrix<double, 4, 4> toMatrix4d(const cv::Mat &cvMat4);
Eigen::Matrix<float, 3, 3> toMatrix3f(const cv::Mat &cvMat3);
Eigen::Matrix<float, 4, 4> toMatrix4f(const cv::Mat &cvMat4);
std::vector<float> toQuaternion(const cv::Mat &M);

bool isRotationMatrix(const cv::Mat &R);
std::vector<float> toEuler(const cv::Mat &R);

// TODO: Sophus migration, to be deleted in the future
Sophus::SE3<float> toSophus(const cv::Mat &T);
Sophus::Sim3f toSophus(const g2o::Sim3 &S);
}  // namespace Converter

}  // namespace ORB_SLAM3
