#include "point_set_ransac.h"
#include "../geometry/point_set.h"
#include "../geometry/point_set_geometry.h"
#include "../basic/stop_watch.h"
#include "../basic/logger.h"
#include "../math/math_types.h"

#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>
#include <CylinderPrimitiveShapeConstructor.h>
#include <SpherePrimitiveShapeConstructor.h>
#include <ConePrimitiveShapeConstructor.h>
#include <TorusPrimitiveShapeConstructor.h>
#include <PlanePrimitiveShape.h>
#include <SpherePrimitiveShape.h>
#include <CylinderPrimitiveShape.h>
#include <ConePrimitiveShape.h>
#include <TorusPrimitiveShape.h>

#include <list>


std::vector<VertexGroup::Ptr> do_detect(
	PointSet* pset, 
	PointCloud& pc, 
	const std::set<VertexGroupType>& types, 
	unsigned int min_support,
	float dist_thresh,
	float bitmap_reso,
	float normal_thresh,
	float overlook_prob
	)
{
	Box3 box = Geom::bounding_box(pset);
	pc.setBBox(
		Vec3f(static_cast<float>(box.x_min()), static_cast<float>(box.y_min()), static_cast<float>(box.z_min())), 
		Vec3f(static_cast<float>(box.x_max()), static_cast<float>(box.y_max()), static_cast<float>(box.z_max()))
		);

	//////////////////////////////////////////////////////////////////////////

	Logger::out(RansacDetector::title()) << "detecting primitives..." << std::endl;
	StopWatch w;

	RansacShapeDetector::Options ransacOptions;
	ransacOptions.m_minSupport = min_support;
	ransacOptions.m_epsilon = dist_thresh * pc.getScale();
	ransacOptions.m_bitmapEpsilon = bitmap_reso * pc.getScale();
	ransacOptions.m_normalThresh = normal_thresh;
	ransacOptions.m_probability = overlook_prob;

	RansacShapeDetector detector(ransacOptions); // the detector object

	// set which primitives are to be detected by adding the respective constructors
	std::set<VertexGroupType>::const_iterator it = types.begin();
	for (; it != types.end(); ++it) {
		switch (*it)
		{
		case VG_PLANE:		detector.Add(new PlanePrimitiveShapeConstructor());		break;
		case VG_CYLINDER:	detector.Add(new CylinderPrimitiveShapeConstructor());	break;
		case VG_SPHERE:		detector.Add(new SpherePrimitiveShapeConstructor());	break;
		case VG_CONE:		detector.Add(new ConePrimitiveShapeConstructor());		break;
		case VG_TORUS:		detector.Add(new TorusPrimitiveShapeConstructor());		break;
		default:	break;
		}
	}

	MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes; // stores the detected shapes
	// returns number of unassigned points
	// the array shapes is filled with pointers to the detected shapes
	// the second element per shapes gives the number of points assigned to that primitive (the support)
	// the points belonging to the first shape (shapes[0]) have been sorted to the end of pc,
	// i.e. into the range [ pc.size() - shapes[0].second, pc.size() )
	// the points of shape i are found in the range
	// [ pc.size() - \sum_{j=0..i} shapes[j].second, pc.size() - \sum_{j=0..i-1} shapes[j].second )
	std::size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes); // run detection
	std::vector<VertexGroup::Ptr> results;

	PointCloud::reverse_iterator start = pc.rbegin();
	PointCloud::reverse_iterator end = pc.rend();
	MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, std::size_t > >::const_iterator shape_itr = shapes.begin();
	for (unsigned int id = 0; shape_itr != shapes.end(); ++shape_itr, ++id) {
		const PrimitiveShape* primitive = shape_itr->first;
		std::size_t num = shape_itr->second;

		std::list<int> vts;
		PointCloud::reverse_iterator point_itr = start;
		for (std::size_t count = 0; count < num; ++count) {
			int v = int(point_itr->index);
			vts.push_back(v);
			++point_itr;
		}
		start = point_itr;

		// Liangliang: I have to check the Ruwen Schnabel's source code of RANSAC. Can the 
		//			   returned primitive have a support point number less than min_support? 
		//			   If so, just skip it.
		if (num < min_support)
			continue;

		// extract parameters for this primitive 
		switch (primitive->Identifier())
		{
		case VG_PLANE: {
			Plane pl = dynamic_cast<const PlanePrimitiveShape*>(primitive)->Internal();
			Vec3f p = pl.getPosition();
			Vec3f n = pl.getNormal();
			Plane3 plane(vec3(p.getValue()), vec3(n.getValue()));

			VertexGroupPlane* group = new VertexGroupPlane;
			group->set_point_set(pset);
			group->insert(group->end(), vts.begin(), vts.end());
			group->set_plane(plane);
			results.push_back(group);
			break;
		}
		case VG_CYLINDER: {
			Cylinder cylinder = dynamic_cast<const CylinderPrimitiveShape*>(primitive)->Internal();
			double radius = cylinder.Radius();
			Vec3f pos = cylinder.AxisPosition();
			Vec3f nor = cylinder.AxisDirection();
			vec3  position(pos[0], pos[1], pos[2]);
			vec3  dir(nor[0], nor[1], nor[2]); dir = normalize(dir);
			// ... 
			VertexGroupCylinder* group = new VertexGroupCylinder;
			group->set_point_set(pset);
			group->insert(group->end(), vts.begin(), vts.end());
			// CylinderMy cyl(radius, position, position + dir);
			// group->set_cylinder(cyl);
			results.push_back(group);
			break;
		}
		case VG_SPHERE: {
			Sphere sphere = dynamic_cast<const SpherePrimitiveShape*>(primitive)->Internal();
			double radius = sphere.Radius();
			Vec3f center = sphere.Center();
			
			VertexGroupSphere* group = new VertexGroupSphere;
			group->set_point_set(pset);
			group->insert(group->end(), vts.begin(), vts.end());
// 			Sphere s(...);
// 			group->set_sphere(s);
			results.push_back(group);
			break;
		}

		case VG_CONE: {
			Cone cone = dynamic_cast<const ConePrimitiveShape*>(primitive)->Internal();
			double radius = cone.RadiusAtLength(1.0f);  // NOTE:: the center is the apex of the cone
			// more parameters ... 
			VertexGroupCone* group = new VertexGroupCone;
			group->set_point_set(pset);
			group->insert(group->end(), vts.begin(), vts.end());
// 			ConeMy c(...);
// 			group->set_cone(c);
			results.push_back(group);
			break;
		}
		case VG_TORUS: {
			Torus torus = dynamic_cast<const TorusPrimitiveShape*>(primitive)->Internal();
			double min_radius = torus.MinorRadius();
			double max_radius = torus.MajorRadius();
			// more parameters ... 
			VertexGroupTorus* group = new VertexGroupTorus;
			group->set_point_set(pset);
			group->insert(group->end(), vts.begin(), vts.end());
// 			TorusMy t(...);
// 			group->set_torus(c);
			results.push_back(group);
			break;
		}
		default:
			break;
		}
	}

	Logger::out(RansacDetector::title()) << results.size() << " primitives extracted. " 
		<< remaining << " points remained. time: " << w.time() << std::endl;

	return results;
}


void RansacDetector::add_primitive_type(VertexGroupType t) {
	if (t != VG_GENERAL)
		types_.insert(t);
	else
		Logger::warn(title()) << "invalid primitive type: VG_GENERAL" << std::endl;
}


void RansacDetector::remove_primitive_type(VertexGroupType t) {
	if (t != VG_GENERAL)
		types_.erase(t);
	else
		Logger::warn(title()) << "invalid primitive type: VG_GENERAL" << std::endl;
}


std::vector<VertexGroup::Ptr> RansacDetector::detect(
	PointSet* pset, 
	unsigned int min_support /* = 1000 */, 
	float dist_thresh /* = 0.005 */, 
	float bitmap_reso /* = 0.02 */, 
	float normal_thresh /* = 0.8 */, 
	float overlook_prob /* = 0.001 */ )
{
	std::vector<VertexGroup::Ptr> results;

	if (pset == nil) {
		Logger::out(title()) << "no data exists" << std::endl;
		return results;
	}

	if (pset->num_of_points() < 3) {
		Logger::out(title()) << "point set has less than 3 points" << std::endl;
		return results;
	}

	if (!pset->has_normals()) {
		Logger::out(title()) << "RANSAC Detector requires point cloud normals" << std::endl;
		return results;
	}

	// prepare the data
	PointCloud pc;
	pc.resize(pset->num_of_points());

	const std::vector<vec3>& normals = pset->normals();
	const std::vector<vec3>& points = pset->points();
#pragma omp parallel for
	for (int i = 0; i < points.size(); ++i) {
		const vec3& p = points[i];
		const vec3& n = normals[i];
		pc[i] = Point(
			Vec3f(p.x, p.y, p.z), 
			Vec3f(n.x, n.y, n.z)
			);
		pc[i].index = i;
	}

	return do_detect(pset, pc, types_, min_support, dist_thresh, bitmap_reso, normal_thresh, overlook_prob);
}


std::vector<VertexGroup::Ptr> RansacDetector::detect(
	PointSet* pset, 
	const std::vector<int>& vertitces, 
	unsigned int min_support /* = 1000 */, 
	float dist_thresh /* = 0.005 */, 
	float bitmap_reso /* = 0.02 */, 
	float normal_thresh /* = 0.8 */, 
	float overlook_prob /* = 0.001 */ )
{
	std::vector<VertexGroup::Ptr> results;

	if (pset == nil) {
		Logger::out(title()) << "no data exists" << std::endl;
		return results;
	}

	if (vertitces.size() < 3) {
		Logger::out(title()) << "input has less than 3 points" << std::endl;
		return results;
	}

	if (!pset->has_normals()) {
		Logger::out(title()) << "RANSAC Detector requires point cloud normals" << std::endl;
		return results;
	}

	// prepare the data
	PointCloud pc;
	pc.resize(vertitces.size());

	const std::vector<vec3>& normals = pset->normals();
	const std::vector<vec3>& points = pset->points();
#pragma omp parallel for
	for (int index = 0; index < vertitces.size(); ++index) {
		std::size_t idx = vertitces[index];
		const vec3& p = points[idx];
		const vec3& n = normals[idx];
		pc[index] = Point(
			Vec3f(p.x, p.y, p.z),
			Vec3f(n.x, n.y, n.z)
			);
		pc[index].index = idx;
	}

	return do_detect(pset, pc, types_, min_support, dist_thresh, bitmap_reso, normal_thresh, overlook_prob);
}