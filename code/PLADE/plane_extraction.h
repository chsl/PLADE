/********************************************************************
 * Copyright (C) 2015 Liangliang Nan <liangliang.nan@gmail.com>
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++ library
 *      for processing and rendering 3D data.
 *      Journal of Open Source Software, 6(64), 3255, 2021.
 * ------------------------------------------------------------------
 *
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 ********************************************************************/

#ifndef EASY3D_ALGO_POINT_CLOUD_RANSAC_H
#define EASY3D_ALGO_POINT_CLOUD_RANSAC_H

#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Dense>

/// \brief Extract planes from point clouds using RANSAC.
/// Usage example:
///     \code
///     PlaneExtraction ransac;
///     int num = ransac.detect(cloud);
///     \endcode

class PLANE : public std::vector<int> {
public:
    template<class InputIt>
    PLANE(InputIt first, InputIt last) : std::vector<int>(first, last) {}
    Eigen::Vector3f normal;
    float d;
};

class PlaneExtraction {
public:
    /// \brief Extract planes from the entire point cloud. \par
    /// Returns the extracted planes. Each plane is expressed by its point indices.
    static std::vector<PLANE> detect(
            const pcl::PointCloud<pcl::PointNormal>& cloud,
            unsigned int min_support = 1000,    // the minimal number of points required for a primitive
            float dist_thresh = 0.005f,    // relative to the bounding box width. NOTE: Internally the distance threshold is taken as 3 * distance_threshold!!!
            float bitmap_reso = 0.02f,    // relative to the bounding box width. NOTE: This threshold is NOT multiplied internally!
            float normal_thresh = 0.8f,    // the cos of the maximal normal deviation
            float overlook_prob = 0.001f    // the probability with which a primitive is overlooked
    );
};


#endif  //  EASY3D_ALGO_POINT_CLOUD_RANSAC_H
