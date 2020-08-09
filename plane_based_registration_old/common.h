#pragma once
#pragma warning( disable : 4996 ) 
//common
#include <iostream>
#include <math.h>
#include <map>
#include <list>
#include <cmath>
#include <vector>
#include <algorithm> 
#include <string>
#include <fstream>
//Eigen
#include <Eigen/StdVector>
#include <Eigen/Dense>
//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h> 
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/boundary.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/features/3dsc.h>
//opencv
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv2/core/core_c.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"
#include "opencv/cxcore.hpp"
#include "opencv2/core/core.hpp"
//ANN
#include <ANN.h>
//omp
#include <omp.h>
//
#include "pointCloud_processing.h"

using namespace std;
using namespace PCP;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;