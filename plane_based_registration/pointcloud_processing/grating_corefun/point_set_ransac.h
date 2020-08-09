#ifndef _ALGORITHM_POINT_SET_RANSAC_H_
#define _ALGORITHM_POINT_SET_RANSAC_H_


/********************************************************************
	created:	 2009/09/16
	created:	 16:9:2009   9:16
	filename: 	 ransac_detector.h
	author:		 Liangliang Nan
	contact:     liangliang.nan@gmail.com

	purpose:	 wraper class for the "RANSAC" algorithm
	*********************************************************************/


#include "algorithm_common.h"
#include "../geometry/vertex_group.h"

#include <string>
#include <vector>
#include <list>
#include <set>



class PointSet;

class ALGO_API RansacDetector
{
public:
	static std::string title() { return "RansacDetector"; }

	// add/remove the primitive type to be extracted
	void add_primitive_type(VertexGroupType t);
	void remove_primitive_type(VertexGroupType t);

	// for entire point cloud. Returns the extracted primitives.
	std::vector<VertexGroup::Ptr> detect(
		PointSet* pset, 
		unsigned int min_support = 1000,	// the minimal number of points required for a primitive
		float dist_thresh = 0.005,	// relative to the bounding box width. NOTE: Internally the distance threshold is taken as 3 * distance_threshold!!!
		float bitmap_reso = 0.02,	// relative to the bounding box width. NOTE: This threshold is NOT multiplied internally!
		float normal_thresh = 0.8,	// the cos of the maximal normal deviation
		float overlook_prob = 0.001	// the probability with which a primitive is overlooked
		);

	// for a subset of the point cloud. Returns the extracted primitives.
	std::vector<VertexGroup::Ptr> detect(
		PointSet* pset, 
		const std::vector<int>& vertitces,
		unsigned int min_support = 1000,	// the minimal number of points required for a primitive
		float dist_thresh = 0.005,	// relative to the bounding box width. NOTE: Internally the distance threshold is taken as 3 * distance_threshold!!!
		float bitmap_reso = 0.02,	// relative to the bounding box width. NOTE: This threshold is NOT multiplied internally!
		float normal_thresh = 0.8,	// the cos of the maximal normal deviation
		float overlook_prob = 0.001	// the probability with which a primitive is overlooked
		);

private:
	std::set<VertexGroupType>	types_;
};


#endif