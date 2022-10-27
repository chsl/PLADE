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

#include "plade.h"
#include "util.h"
#include <pcl/io/ply_io.h>

#ifdef HAS_EASY3D
#include <easy3d/viewer/viewer.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/util/logging.h>
#endif


int main(int argc, char **argv) {
    std::string data_dir = std::string(PLADE_CODE_DIR) + "/../sample_data/";
#if 0
    std::string targetFile = data_dir + "polyhedron_target.ply";
    std::string sourceFile = data_dir + "polyhedron_source.ply";
    int ransac_min_support_target = 1000;  // tune this so #planes in target cloud is about in [10, 40]
    int ransac_min_support_source = 1000;  // tune this so #planes in source cloud is about in [10, 40]
#else
    std::string targetFile = data_dir + "room_target.ply";
    std::string sourceFile = data_dir + "room_source.ply";
    int ransac_min_support_target = 200;  // tune this so #planes in target cloud is about in [10, 40]
    int ransac_min_support_source = 4000; // tune this so #planes in source cloud is about in [10, 40]
#endif

    std::cout << "target file: " << targetFile << std::endl;
    std::cout << "source file: " << sourceFile << std::endl;
    if (extension(targetFile) != "ply" || extension(sourceFile) != "ply") {
        std::cerr << "only PLY format is accepted" << std::endl;
        return EXIT_FAILURE;
    }

    /////////////////////////////////////////////////////////////////////////

    pcl::PLYReader reader;
    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud(new pcl::PointCloud<pcl::PointNormal>);
    if (reader.read(targetFile, *target_cloud) != 0) {
        std::cerr << "loading target point cloud failed" << std::endl;
        return EXIT_FAILURE;
    }
    pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud(new pcl::PointCloud<pcl::PointNormal>);
    if (reader.read(sourceFile, *source_cloud) != 0) {
        std::cerr << "loading source point cloud failed" << std::endl;
        return EXIT_FAILURE;
    }

    bool switched = false;
    if (source_cloud->size() >= target_cloud->size() * 1.2f) {
        std::swap(target_cloud, source_cloud);
        std::swap(ransac_min_support_target, ransac_min_support_source);
        switched = true;
        std::cout << "---->>> ATTENTION: target and source have been switched for efficiency <<<----" << std::endl;
    }

    Eigen::Matrix<float, 4, 4> transformation;
    bool status = registration(transformation,
                               target_cloud,
                               source_cloud,
                               ransac_min_support_target,
                               ransac_min_support_source);
    if (!status) {
        std::cerr << "registration failed";
        return EXIT_FAILURE;
    }

    if (switched)
        transformation = transformation.inverse();

    std::cout << "transformation matrix (align source to target): " << std::endl << transformation << std::endl;

#ifdef HAS_EASY3D
    easy3d::logging::initialize();
    easy3d::Viewer viewer("PLADE registration visualization");
    auto target_model = viewer.add_model(targetFile, true);
    auto source_model = viewer.add_model(sourceFile, true);
    target_model->renderer()->get_points_drawable("vertices")->set_uniform_coloring(easy3d::vec4(1.0, 0, 0, 1));
    source_model->renderer()->get_points_drawable("vertices")->set_uniform_coloring(easy3d::vec4(0, 0, 1.0, 1));
    easy3d::mat4 T(transformation.data());
    auto src_cloud = dynamic_cast<easy3d::PointCloud*>(source_model);
    auto points = src_cloud->get_vertex_property<easy3d::vec3>("v:point");
    auto normals = src_cloud->get_vertex_property<easy3d::vec3>("v:normal");
    const easy3d::mat4 N = transpose(inverse(T));
    for (auto v : src_cloud->vertices()) {
        points[v] = T * points[v];
        normals[v] = N * normals[v];
        normals[v].normalize();
    }
    viewer.usage_string_ = " ";
    return viewer.run();
#else
    return EXIT_SUCCESS;
#endif
}