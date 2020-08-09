/*
 * Software License Agreement
 *
 *  Copyright (c) P.W.Theiler, 2015
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef SHORT_FILTER_H_
#define SHORT_FILTER_H_

#include <pcl/filters/voxel_grid.h>

namespace pcl
{
  /** \brief Downsample the point cloud according to neighboring point distance.
  	* The method uses the octree structure and carries out a voxel grid filter on each octree leaf.
  	*
  	* \note This breaks the organized structure of the cloud by setting the height to 1!
  	*
  	* \param[in] cloud_in pointer to input point cloud
  	* \param[out] cloud_out sampled point cloud
  	* \param[in] voxel_size resulting point sampling distance
  	* \return the size of the downsampled scan cloud
  	*
  	* \author P.W.Theiler
  	*/
  //template <typename PointT> int
  //sampleLeafsized (typename const pcl::PointCloud <PointT>::ConstPtr &cloud_in, pcl::PointCloud <PointT> &cloud_out, float voxel_size);
int sampleLeafsized (pcl::PointCloud <pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud <pcl::PointXYZI> &cloud_out, const float &voxel_size);

}

#include "impl/short_filter.hpp"

#endif  // SHORT_FILTER_H_
