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

#include <easy3d/viewer/viewer.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/util/logging.h>

#include <Eigen/Dense>


int main(int argc, char **argv) {
    const int idx = 3;

    const std::string result_file = "/Users/lnan/Downloads/jitter_data/cloud_pairs_results.txt";
    std::ifstream input(result_file);
    if (!input.is_open()) {
        std::cerr << "failed loading result file: " << result_file << std::endl;
        return EXIT_FAILURE;
    }

    std::vector<std::string> target_files, source_files;
    std::vector<Eigen::Matrix<float, 4, 4>> transformations;

    while (!input.eof()) {
        std::string line;
        getline(input, line);
        if (!line.empty()) {
            if (line.substr(0, 7) == "target:")
                target_files.push_back(line.substr(8));
            else if (line.substr(0, 7) == "source:")
                source_files.push_back(line.substr(8));
            else if ((line.find("transformation:") != std::string::npos) ||
                    (line.find("registration failed, an identity matrix is recorded:") != std::string::npos)) {
                Eigen::Matrix<float, 4, 4> T;
                for (int i=0; i<4; ++i) {
                    for (int j = 0; j < 4; ++j)
                        input >> T(i, j);
                }
                transformations.push_back(T);
            }
        }
    }

    easy3d::logging::initialize();
    easy3d::Viewer viewer("PLADE registration visualization");
    auto target_model = viewer.add_model(target_files[idx], true);
    auto source_model = viewer.add_model(source_files[idx], true);
    target_model->renderer()->get_points_drawable("vertices")->set_uniform_coloring(easy3d::vec4(1.0, 0, 0, 1));
    source_model->renderer()->get_points_drawable("vertices")->set_uniform_coloring(easy3d::vec4(0, 0, 1.0, 1));
    easy3d::mat4 T(transformations[idx].data());

    std::cout << "target: " << target_files[idx] << std::endl;
    std::cout << "source: " << source_files[idx] << std::endl;
    std::cout << "transformation: \n" << T << std::endl;

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
}