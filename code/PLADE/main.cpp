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

#include <fstream>

#include "plade.h"

#ifdef HAS_EASY3D
#include <easy3d/viewer/viewer.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/util/logging.h>
#endif


int main(int argc, char **argv) {
#if 1
    if (argc != 3 && argc != 4) {
        std::cerr << "PLADE can register two point clouds dominated by planar structures. It can be used in two ways.\n"
                   << "-------------------------------------------------------------------------------------------------\n"
                   << "Usage 1: register a 'source' point cloud to a 'target' point cloud.\n"
                   << "    ---------------------------------------------------------------------------------------------\n"
                   << "    You can call PLADE with three arguments. The first two are the file names of a target point\n"
                   << "    cloud and a source point cloud (the target point cloud file name always comes first). The\n"
                   << "    third argument specifies the result file name. Below is an example:\n"
                   << "         ./PLADE  room_target.ply  room_source.ply  result.txt\n"
                   << "    The target point cloud file name always comes first, and both point cloud files must be in\n"
                   << "    the 'ply' format. The result file will store the registration result, which is a 4 by 4\n"
                   << "    transformation matrix that aligns the source point cloud to the target point cloud.\n"
                   << "-------------------------------------------------------------------------------------------------\n"
                   << "Usage 2: register a bunch of point cloud pairs.\n"
                   << "    ---------------------------------------------------------------------------------------------\n"
                   << "    You can call PLADE with two arguments: a file (e.g., file_pairs.txt) specifying all pairs\n"
                   << "    of target/source point cloud files and a result file. Below is an example:\n"
                   << "         ./PLADE  file_pairs.txt  result.txt\n"
                   << "    In 'file_pairs.txt', every two consecutive lines store two file names. The first line is the\n"
                   << "    file name of a target point cloud, and the second line is the file name of a source point cloud.\n"
                   << "    Both point cloud files must be in the 'ply' format. The result file will store the registration\n"
                   << "    results, a set of 4 by 4 transformation matrices. Each matrix aligns a source point cloud to\n"
                   << "    its corresponding target point cloud.\n";
        return EXIT_FAILURE;
    }

    // Usage 1: register a 'source' point cloud to a 'target' point cloud
    if (argc == 4) {
        Eigen::Matrix<float, 4, 4> transformation;
        if (registration(transformation, argv[1], argv[2])) {
            std::ofstream output(argv[3]);
            if (!output.is_open()) {
                std::cerr << "failed opening the result file: " << argv[3] << std::endl;
                return EXIT_FAILURE;
            }
            output << "target: " << argv[1] << std::endl;
            output << "source: " << argv[2] << std::endl;
            output << "transformation:\n" << transformation << std::endl;
            std::cout << "the registration result has been written into file: " << argv[3] << std::endl;
            return EXIT_SUCCESS;
        }
        else {
            std::cerr << "registration failed" << std::endl;
            return EXIT_FAILURE;
        }
    }

    // Usage 2: register a bunch of point cloud pairs
    else if (argc == 3) {
        std::ifstream input(argv[1]);
        if (!input.is_open()) {
            std::cerr << "failed opening the file containing pairs of point cloud names: " << argv[1] << std::endl;
            return EXIT_FAILURE;
        }

        std::ofstream output(argv[2]);
        if (!output.is_open()) {
            std::cerr << "failed opening the result file: " << argv[2] << std::endl;
            return EXIT_FAILURE;
        }

        int count_success = 0;
        int count_failure = 0;
        while (!input.eof()) {
            std::vector<std::string> file_pair;
            while (!input.eof() && file_pair.size() < 2) {
                std::string file_name;
                getline(input, file_name);
                std::cout << file_name << std::endl;
            }

            if (file_pair.size() == 2) {
                output << "target: " << file_pair[0] << std::endl;
                output << "source: " << file_pair[1] << std::endl;
                Eigen::Matrix<float, 4, 4> transformation;
                if (registration(transformation, file_pair[0], file_pair[1])) {
                    output << "transformation:\n" << transformation << std::endl << std::endl;
                    ++ count_success;
                }
                else {
                    output << "registration failed" << std::endl << std::endl;
                    ++ count_failure;
                }
            }
        }

        if (count_success == 0) {
            std::cerr << "registration all failed (" << count_failure << " pairs)" << std::endl;
            return EXIT_FAILURE;
        }
        if (count_failure > 0)
            std::cerr << "registration of " << count_failure << " (out of " << count_failure + count_success << ") pairs failed" << std::endl;
        std::cout << "the registration result has been written into file: " << argv[2] << std::endl;

        return EXIT_SUCCESS;
    }

#else

    std::string data_dir = std::string(PLADE_CODE_DIR) + "/../sample_data/";
#if 0
    std::string target_cloud_file = data_dir + "polyhedron_target.ply";
    std::string source_cloud_file = data_dir + "polyhedron_source.ply";
#else
    std::string target_cloud_file = data_dir + "room_target.ply";
    std::string source_cloud_file = data_dir + "room_source.ply";
#endif

    Eigen::Matrix<float, 4, 4> transformation;
    bool status = registration(transformation, target_cloud_file, source_cloud_file);
    if (!status) {
        std::cerr << "registration failed" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "transformation:\n" << transformation << std::endl << std::endl;

#ifdef HAS_EASY3D
    easy3d::logging::initialize();
    easy3d::Viewer viewer("PLADE registration visualization");
    auto target_model = viewer.add_model(target_cloud_file, true);
    auto source_model = viewer.add_model(source_cloud_file, true);
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
#endif
}