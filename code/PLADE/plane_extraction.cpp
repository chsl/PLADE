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

#include "plane_extraction.h"

#include <set>
#include <list>

#include "RansacShapeDetector.h"
#include "PlanePrimitiveShapeConstructor.h"
#include "PlanePrimitiveShape.h"

//OMG, there is class with exactly the same name in RANSAC!!!
typedef ::PointCloud PointCloud_Ransac;


// \cond
namespace details {

    void correct_normal(PLANE& plane, const pcl::PointCloud<pcl::PointNormal>& cloud) {
        double nx(0), ny(0), nz(0);
        int pointsNum = cloud.size();
        int count = 0;
        for (int i = 0; i < pointsNum; i++) {
            nx += cloud.at(i).normal_x;
            ny += cloud.at(i).normal_y;
            nz += cloud.at(i).normal_z;
        }

        // the plane normal must be similar to the avg point normal
        Eigen::Vector3f avg_normal(nx/count, ny/count, nz/count);
        avg_normal.normalize();
        if (avg_normal.dot(plane.normal) < 0)
            plane.normal = -plane.normal;
    }


    std::vector<PLANE> do_detect(
            const pcl::PointCloud<pcl::PointNormal>& cloud,
            PointCloud_Ransac &pc,
            unsigned int min_support,
            float dist_thresh,
            float bitmap_reso,
            float normal_thresh,
            float overlook_prob
    ) {
#if 1
        float minX(std::numeric_limits<float>::max()),  minY(std::numeric_limits<float>::max()),  minZ(std::numeric_limits<float>::max());
        float maxX(-std::numeric_limits<float>::max()), maxY(-std::numeric_limits<float>::max()), maxZ(-std::numeric_limits<float>::max());
        for (int i = 0; i < cloud.size(); ++i) {
            minX = std::min(cloud.at(i).x, minX);   minY = std::min(cloud.at(i).y, minY);   minZ = std::min(cloud.at(i).z, minZ);
            maxX = std::max(cloud.at(i).x, maxX);   maxY = std::max(cloud.at(i).y, maxY);   minZ = std::max(cloud.at(i).z, maxZ);
        }
        pc.setBBox(
                Vec3f(static_cast<float>(minX), static_cast<float>(minY), static_cast<float>(minZ)),
                Vec3f(static_cast<float>(maxX), static_cast<float>(maxY), static_cast<float>(maxZ))
        );
#else
        pcl::PointNormal minPt, maxPt;
        pcl::getMinMax3D(cloud, minPt, maxPt);
        pc.setBBox(
                Vec3f(static_cast<float>(minPt.x), static_cast<float>(minPt.y), static_cast<float>(minPt.z)),
                Vec3f(static_cast<float>(maxPt.x), static_cast<float>(maxPt.y), static_cast<float>(maxPt.z))
        );
#endif
        //////////////////////////////////////////////////////////////////////////

//        std::cout << "detecting primitives..." << std::endl;

        RansacShapeDetector::Options ransacOptions;
        ransacOptions.m_minSupport = min_support;
        ransacOptions.m_epsilon = dist_thresh * pc.getScale();
        ransacOptions.m_bitmapEpsilon = bitmap_reso * pc.getScale();
        ransacOptions.m_normalThresh = normal_thresh;
        ransacOptions.m_probability = overlook_prob;

        RansacShapeDetector detector(ransacOptions); // the detector object

        // set which primitives are to be detected by adding the respective constructors
        detector.Add(new PlanePrimitiveShapeConstructor());

        MiscLib::Vector<std::pair<MiscLib::RefCountPtr<PrimitiveShape>, size_t> > shapes; // stores the detected shapes
        // returns number of unassigned points
        // the array shapes is filled with pointers to the detected shapes
        // the second element per shapes gives the number of points assigned to that primitive (the support)
        // the points belonging to the first shape (shapes[0]) have been sorted to the end of pc,
        // i.e. into the range [ pc.size() - shapes[0].second, pc.size() )
        // the points of shape i are found in the range
        // [ pc.size() - \sum_{j=0..i} shapes[j].second, pc.size() - \sum_{j=0..i-1} shapes[j].second )
        std::size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes); // run detection

        PointCloud_Ransac::reverse_iterator start = pc.rbegin();
        MiscLib::Vector<std::pair<MiscLib::RefCountPtr<PrimitiveShape>, std::size_t> >::const_iterator shape_itr = shapes.begin();

        int index = 0;
        std::vector<PLANE> planes;
        for (; shape_itr != shapes.end(); ++shape_itr) {
            const PrimitiveShape *primitive = shape_itr->first;
            std::size_t num = shape_itr->second;

            std::list<int> vts;
            PointCloud_Ransac::reverse_iterator point_itr = start;
            for (std::size_t count = 0; count < num; ++count) {
                int v = int(point_itr->index);
                vts.push_back(v);
                ++point_itr;
            }
            start = point_itr;

            // Liangliang: I have to check Ruwen Schnabel's source code of RANSAC. Can the
            //			   returned primitive have a support point number less than min_support?
            //			   If so, just skip it.
            if (num < min_support)
                continue;

            // extract parameters for this primitive
            switch (primitive->Identifier()) {
                case 0: { // 0 denotes "plane"
                    PLANE plane(vts.begin(), vts.end());
                    // parameters
                    const Plane& pl = dynamic_cast<const PlanePrimitiveShape*>(primitive)->Internal();
                    const Vec3f& p = pl.getPosition();
                    Vec3f n = pl.getNormal(); n.normalize();
                    plane.normal = Eigen::Vector3f(n[0], n[1], n[2]);
                    plane.d = -(n[0] * p[0] + n[1] * p[1] + n[2] * p[2]);
                    correct_normal(plane, cloud);
                    planes.push_back(plane);
                    break;
                }
//                case PrimitivesRansac::CYLINDER: {
//                }
//                case PrimitivesRansac::SPHERE: {
//                }
//                case PrimitivesRansac::CONE: {
//                }
//                case PrimitivesRansac::TORUS: {
//                }
                default: break;
            }
            ++index;
        }

//        std::cout << index << " planes extracted. " << remaining << " points remained";
        return planes;
    }
}
// \endcond


std::vector<PLANE> PlaneExtraction::detect(
        const pcl::PointCloud<pcl::PointNormal>& cloud,
        unsigned int min_support /* = 1000 */,
        float dist_thresh /* = 0.005 */,
        float bitmap_reso /* = 0.02 */,
        float normal_thresh /* = 0.8 */,
        float overlook_prob /* = 0.001 */ ) {

    if (cloud.size() < 3) {
        std::cerr << "point set has less than 3 points" << std::endl;
        return std::vector<PLANE>();
    }

    // prepare the data
    PointCloud_Ransac pc;
    pc.resize(cloud.size());

#pragma omp parallel for
    for (int i = 0; i < cloud.size(); ++i) {
        const auto& pn = cloud.points[i];
        pc[i] = Point(
                Vec3f(pn.x, pn.y, pn.z),
                Vec3f(pn.normal_x, pn.normal_y, pn.normal_z)
        );
        pc[i].index = i;
    }

    return details::do_detect(cloud, pc, min_support, dist_thresh, bitmap_reso, normal_thresh, overlook_prob);
}