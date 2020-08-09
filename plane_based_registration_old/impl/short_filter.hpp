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

#ifndef IMPL_SHORT_FILTER_H_
#define IMPL_SHORT_FILTER_H_

#include <pcl/octree/octree_impl.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//template <typename PointT> int
//pcl::sampleLeafsized ( const pcl::PointCloud <PointT>::ConstPtr &cloud_in, pcl::PointCloud <PointT> &cloud_out, float voxel_size)
int pcl::sampleLeafsized (pcl::PointCloud <pcl::PointXYZI>::Ptr &cloud_in, pcl::PointCloud <pcl::PointXYZI> &cloud_out, const float &voxel_size)
{
  pcl::PointCloud <pcl::PointXYZI> cloud_sub;
  cloud_out.clear ();
  float leafsize = voxel_size * (std::pow (static_cast <int64_t> (std::numeric_limits <int32_t>::max ()) - 1, 1./3.) - 1);

  pcl::octree::OctreePointCloud <pcl::PointXYZI> oct (leafsize); // new octree structure
  oct.setInputCloud (cloud_in);
  oct.defineBoundingBox ();
  oct.addPointsFromInputCloud ();

  pcl::VoxelGrid <pcl::PointXYZI> vg; // new voxel grid filter
  vg.setLeafSize (voxel_size, voxel_size, voxel_size);
  vg.setInputCloud (cloud_in);

  size_t num_leaf = oct.getLeafCount ();

  pcl::octree::OctreePointCloud <pcl::PointXYZI>::LeafNodeIterator it = oct.leaf_begin (), it_e = oct.leaf_end ();
  for (size_t i = 0; i < num_leaf; ++i, ++it)
  {
    pcl::IndicesPtr ids (new std::vector <int>); // extract octree leaf points
      pcl::octree::OctreePointCloud <pcl::PointXYZI>::LeafNode *node = (pcl::octree::OctreePointCloud <pcl::PointXYZI>::LeafNode *) *it;
      node->getContainerPtr ()->getPointIndices (*ids);

    vg.setIndices (ids); // set cloud indices
    vg.filter (cloud_sub); // filter cloud

    cloud_out += cloud_sub; // add filter result
  }
  //printf("voxel grid output.%d\n",cloud_out.width);
  return (static_cast <int> (cloud_out.size ())); // return number of points in sampled cloud
}


#endif  // IMPL_SHORT_FILTER_H_
