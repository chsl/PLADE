/**
 *	Copyright (C) 2019 by
 *       Songlin Chen (chsl523278770@163.com)
 *       Liangliang Nan (liangliang.nan@gmail.com)
 *
 *	This file is part of PLADE, which implements the point cloud registration method described
 *	in the following paper:
 *  -------------------------------------------------------------------------------------
 *      PLADE: A Plane-based Descriptor for Point Cloud Registration with Small Overlap
 *      Songlin Chen, Liangliang Nan, Renbo Xia, Jibin Zhao, and Peter Wonka.
 *      IEEE Transactions on Geoscience and Remote Sensing. 58(4), 2530-2540, 2020
 *  -------------------------------------------------------------------------------------
 *  Please consider citing the above paper if you use the code/program (or part of it).
 *
 *	PLADE is free software; you can redistribute it and/or modify it under the terms of the
 *	GNU General Public License Version 3 as published by the Free Software Foundation.
 *
 *	PLADE is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 *	even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along with this program.
 *	If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PLADE_H
#define PLADE_H

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "plane_extraction.h"


/**
 * Implementation of the PLADE registration method.
 * This method takes the file names of two points and outputs the registration transformation.
 * @param transformation On success \c transformation returns the registration transformation.
 * @param target_cloud_file The file name of the reference point cloud.
 * @param source_cloud_file The file name of the source point cloud that will be transformed to align with the reference point cloud.
 * @return \c ture on success, otherwise \c false.
 */
bool registration(Eigen::Matrix<float, 4, 4> &transformation,
                  const std::string& target_cloud_file,
                  const std::string& source_cloud_file
);

/**
 * Implementation of the PLADE registration method.
 * This method automatically tunes the RANSAC parameters such that at least 10 and at most 40 planes will be extracted
 * and used for registration.
 * @param transformation On success \c transformation returns the registration transformation.
 * @param target_cloud The reference point cloud.
 * @param source_cloud The point cloud that will be transformed to align with \c target_cloud.
 * @return \c ture on success, otherwise \c false.
 */
bool registration(Eigen::Matrix<float, 4, 4> &transformation,
                  pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud,
                  pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud
);

/**
 * Implementation of the PLADE registration method.
 * This method requires the user to provide the extracted planes.
 * and used for registration.
 * @param transformation On success \c transformation returns the registration transformation.
 * @param target_cloud The reference point cloud.
 * @param source_cloud The point cloud that will be transformed to align with \c target_cloud.
 * @param target_planes The extracted planes from the target point cloud.
 * @param source_planes The extracted planes from the source point cloud.
 * @return \c ture on success, otherwise \c false.
 */
bool registration(Eigen::Matrix<float, 4, 4> &transformation,
                  pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud,
                  pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
                  const std::vector<PLANE>& target_planes,
                  const std::vector<PLANE>& source_planes
);

/**
 * Implementation of the PLADE registration method.
 * This method requires the RANSAC parameters, so it allows to test how the plane extraction affects registration.
 * @param transformation On success \c transformation returns the registration transformation.
 * @param target_cloud The reference point cloud.
 * @param source_cloud The point cloud that will be transformed to align with \c target_cloud.
 * @param ransac_min_support_target The minimum point number for RANSAC plane extraction.
 * @param ransac_min_support_source The minimum point number for RANSAC plane extraction.
 * @return \c ture on success, otherwise \c false.
 */
bool registration(Eigen::Matrix<float, 4, 4> &transformation,
                  pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud,
                  pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
                  int ransac_min_support_target,
                  int ransac_min_support_source
);

#endif // PLADE_H