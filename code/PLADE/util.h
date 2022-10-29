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


#ifndef PLADE_UTIL
#define PLADE_UTIL

#if defined _MSC_VER || defined WIN32 || defined _WIN32
// 4244 : conversion from 'type1' to 'type2', possible loss of data
// 4661 : no suitable definition provided for explicit template instantiation reques
// 4503 : decorated name length exceeded, name was truncated
// 4146 : unary minus operator applied to unsigned type, result still unsigned
#pragma warning (disable:  4018 4244 4267 4521 4251 4273 4661 4305 4316 4477 4503 4146 4723 4800 4996)
#endif

#include <string>
#include <vector>

//Eigen
#include <Eigen/StdVector>
#include <Eigen/Dense>
//pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>
//opencv
#include <opencv2/core/core.hpp>
//ANN
#include <ANN/ANN.h>

#include "plane_extraction.h"


struct NearstPointsTwoLine
{
	Eigen::Vector3f points1;
	Eigen::Vector3f points2;
	double angle;
	double length;
	double minAngle;
};

struct INTERSECTION_LINE {
    Eigen::Vector3f lineVec;
    Eigen::Vector3f linePoint;
    std::vector<int> supportPlanes;
    std::vector<double> confidence;
    std::vector<double> distance;
    double planeAngle;
    bool isIntersectionLine;
};

struct MatchInformation
{
	//lines
	std::vector<INTERSECTION_LINE> *pIntersectionLine;
	std::vector<std::vector<NearstPointsTwoLine>> *pNearstPointsTwoLines;

	//planes
	std::vector<Eigen::Vector4f> *pPlanes;
	const std::vector<PLANE> *pSupportPlaneIndex;

	//points
	pcl::PointCloud<pcl::PointXYZ>::Ptr points;
	pcl::PointCloud<pcl::PointNormal>::Ptr originalPoints;

	//boundingBox
	Eigen::Vector3f boundingCenter;//model
	//for each plane
	std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> *pDownSampleKdTree;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> *pDownSamplePlanePoints;
	std::vector<std::vector<Eigen::Vector3f>>*pBoundingBoxFourCornerPoints;
	std::vector<Eigen::Vector3f>*pBoundingBoxCenterForEachPlane;
	std::vector<float>*pBoundingBoxRadiusForEachPlane;
};

struct PAIRLINE {
    Eigen::Vector3f lineVec1;
    Eigen::Vector3f lineVec2;
    Eigen::Vector3f linePoints1;
    Eigen::Vector3f linePoints2;
    int originalIndex1;
    int originalIndex2;
    Eigen::VectorXf descriptor;
};

struct Parameter
{
	double lengthThreshold;
	double angleThreshold;
	double cosAngleThreshold;
	int    maxCandidateResultNum;
	std::vector<KdTreeSearchNDim<Eigen::VectorXf, 8>> *kdtrees8;
	std::vector<KdTreeSearchNDim<Eigen::VectorXf, 6>> *kdtrees6;
	std::vector<KdTreeSearchNDim<Eigen::VectorXf, 4>> *kdtrees4;
	std::vector<std::vector<PAIRLINE>> *mainLinesInformation;
	int   maxNeighbor;
	float maxRadius;
};

struct MatchedResult
{
	std::vector<std::pair<int, int>>matchedPlanes;
	Eigen::Matrix3f R;
	Eigen::Vector3f T;
	float planeScore;
};

struct QueryResult
{
	PAIRLINE queryLine;
	std::vector<int> neighbor;
	std::vector<float> neighborDistance;
	QueryResult(PAIRLINE queryLine, std::vector<int> neighbor, std::vector<float> neighborDistance)
		:queryLine(queryLine), neighbor(neighbor), neighborDistance(neighborDistance) {}
};

enum METHODINDEX {
    method11,
    method12,
    method21,
    method22
};

void MatchingLines(MatchInformation current,
	MatchInformation main,
	std::vector<std::pair<int, int>> linesTobeMatched,
	std::vector<std::vector<int>>coarseMatches,
	std::vector<MatchedResult>& pMatchedResult,
	Parameter parameter);

std::string extension(const std::string& file_name) ;

template <typename PointT>
int DownSamplePointCloud(typename pcl::PointCloud<PointT>::Ptr &srcPoints,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &downSamplePoints,
                         float gridX, float gridY, float gridZ) {
    if (srcPoints->empty() || gridX <= 0 || gridY <= 0 || gridZ <= 0) {
        return -1;
    }

    typename pcl::PointCloud<PointT> tmp;

    //////////////////////////////////////////////////////////////////////////
    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize(gridX, gridY, gridZ);
    grid.setInputCloud(srcPoints);
    grid.filter(tmp);

    downSamplePoints->resize(tmp.size());
    for (int i=0; i<tmp.size(); ++i) {
        const auto& p = tmp.at(i);
        downSamplePoints->at(i) = pcl::PointXYZ(p.x, p.y, p.z);
    }

    return 0;
}

template <typename PointT>
int ComputeBoundingBox(typename pcl::PointCloud<PointT>::Ptr points,
                       Eigen::Vector3f &centerPoint, double &width, double &height, double &depth,
                       typename pcl::PointCloud<PointT> *pCornerPoints = NULL,
                       Eigen::Matrix3f *pR = NULL, Eigen::Vector3f *pT = NULL) {
    if (points->empty()) {
        return -1;
    }
    // compute principal direction
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*points, centroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*points, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
    eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

    // move the points to the that reference frame
    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    p2w.block<3, 3>(0, 0) = eigDx.transpose();
    p2w.block<3, 1>(0, 3) = -1.f * (p2w.block<3, 3>(0, 0) * centroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*points, cPoints, p2w);

    PointT min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);
    const Eigen::Vector3f mean_diag = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

    centerPoint = eigDx * mean_diag + centroid.head<3>();
    width = max_pt.x - min_pt.x;
    depth = max_pt.y - min_pt.y;
    height = max_pt.z - min_pt.z;

    //compute cornerPoints
    if (pCornerPoints) {
        pCornerPoints->clear();
        pcl::PointCloud<pcl::PointXYZ> corners;
        float x = min_pt.x;
        float y = min_pt.y;
        float z = min_pt.z;
        corners.push_back(min_pt);
        corners.push_back(pcl::PointXYZ(x, y + depth, z));
        corners.push_back(pcl::PointXYZ(x, y + depth, z + height));
        corners.push_back(pcl::PointXYZ(x, y, z + height));
        corners.push_back(pcl::PointXYZ(x + width, y, z + height));
        corners.push_back(pcl::PointXYZ(x + width, y + depth, z));
        corners.push_back(pcl::PointXYZ(x + width, y, z));
        corners.push_back(pcl::PointXYZ(x + width, y + depth, z + height));
        typename pcl::registration::TransformationEstimationSVD<PointT, PointT>::Matrix4 transformation;
        transformation = pcl::registration::TransformationEstimationSVD<PointT, PointT>::Matrix4::Identity();
        transformation.block(0, 0, 3, 3) = eigDx;
        transformation.block(0, 3, 3, 1) = centroid.head<3>();
        pcl::transformPointCloud(corners, *pCornerPoints, transformation);
    }
    if (pR) {
        *pR = eigDx;
    }
    if (pT) {
        *pT = centroid.head<3>();
    }

    return 0;
}

int ComputeDescriptorVectorForPairLines(const INTERSECTION_LINE &line1,
                                        const INTERSECTION_LINE &line2,
                                        const Eigen::Vector3f &line1sp1,
                                        const Eigen::Vector3f &line1sp2,
                                        const Eigen::Vector3f &line2sp1,
                                        const Eigen::Vector3f &line2sp2,
                                        Eigen::VectorXf &descriptor,
                                        Eigen::Vector3f &newLine1,
                                        Eigen::Vector3f &newLine2,
                                        METHODINDEX methodIndex);

int ComputeTransformationUsingTwoVecAndOnePoint(Eigen::Vector3f &sourceVec1, Eigen::Vector3f &sourceVec2,
                                                Eigen::Vector3f &destVec1, Eigen::Vector3f &destVec2,
                                                Eigen::Vector3f &sourcePoint, Eigen::Vector3f &targetPoint,
                                                Eigen::Matrix3f &R, Eigen::Vector3f &T);

template <typename PointT>
int ExchnageBetweentPCLPointXYZwithEigenVector3f(pcl::PointCloud<PointT> &pclPoint,
                                                 std::vector<Eigen::Vector3f> &eigenPoints) {
    if (pclPoint.empty()) {
        size_t pointsNum = eigenPoints.size();
        pclPoint.resize(pointsNum);
        for (size_t i = 0; i < pointsNum; i++) {
            memcpy(&pclPoint[i], &eigenPoints[i], sizeof(float) * 3);
        }
    } else {
        size_t pointsNum = pclPoint.size();
        eigenPoints.resize(pointsNum);
        for (size_t i = 0; i < pointsNum; i++) {
            memcpy(&eigenPoints[i], &pclPoint[i], sizeof(float) * 3);
        }
    }
    return 0;
}


template <typename FT>
FT generateNan() {
    FT i = 0.0, j = 0.0;
    return i / j;
}

template <typename PointT>
int ProjectPoints2Plane(const pcl::PointCloud<PointT> &pointsIn, const Eigen::Vector4f &plane,
                        pcl::PointCloud<PointT> &pointsOut,
                        int *pBegin = NULL, int *pEnd = NULL) {
    if (pointsIn.empty()) {
        return -1;
    }
    //////////////////////////////////////////////////////////////////////////
    float A = plane(0);
    float B = plane(1);
    float C = plane(2);
    float D = plane(3);
    size_t start = 0, end = pointsIn.size();
    if (pBegin && pEnd) {
        start = *pBegin;
        end = *pEnd;
        if (end > pointsIn.size()) {
            end = pointsIn.size();
        }
    }
    pointsOut.resize(end - start);
    //#pragma omp parallel for
    for (int i = start; i < end; i++) {
        const pcl::PointXYZ &tempPoints = pointsIn[i];
        if (!cvIsInf(tempPoints.x + tempPoints.y + tempPoints.z) &&
            !cvIsNaN(tempPoints.x + tempPoints.y + tempPoints.z)) {
            float k = -(A * tempPoints.x + B * tempPoints.y + C * tempPoints.z + D) / (A * A + B * B + C * C);
            pointsOut[i - start].x = tempPoints.x + k * A;
            pointsOut[i - start].y = tempPoints.y + k * B;
            pointsOut[i - start].z = tempPoints.z + k * C;
        } else {
            pointsOut[i - start].x = generateNan<float>();
            pointsOut[i - start].y = pointsOut[i].x;
            pointsOut[i - start].z = pointsOut[i].x;
        }
    }
    return 0;
}


template <typename PointT>
int ProjectPoints2Plane(const pcl::PointCloud<PointT> &pointsIn, const Eigen::Vector4f &plane,
                        std::vector<Eigen::Vector3f> &pointsOut, int *pBegin = NULL, int *pEnd = NULL) {
    pcl::PointCloud<PointT> pclPoints;
    if (0 != ProjectPoints2Plane(pointsIn, plane, pclPoints, pBegin, pEnd)) {
        return -1;
    }
    return ExchnageBetweentPCLPointXYZwithEigenVector3f(pclPoints, pointsOut);
}

int ComputeIntersectionLineOfTwoPlanes(const Eigen::Vector4f &plane1,
                                       const Eigen::Vector4f &plane2,
                                       Eigen::Vector3f &lineVec,
                                       Eigen::Vector3f &linePoint);

struct LENGTHINDEX {
    float length;
    int index;
};

inline bool myCompareLess(const LENGTHINDEX &l1, const LENGTHINDEX &l2) {
    if (l1.length < l2.length) {
        return true;
    }
    return false;
}


inline bool myCompareGreater(const LENGTHINDEX &l1, const LENGTHINDEX &l2) {
    if (l1.length > l2.length) {
        return true;
    }
    return false;
}

template <typename PointT>
int ComputeProjectionPointOf3DLine(const Eigen::Vector3f &lineVec, const Eigen::Vector3f &linePoint,
                                   const PointT &pointIn, Eigen::Vector3f &pointOut) {
    if (lineVec[0] == 0 && lineVec[1] == 0 && lineVec[2] == 0) {
        return -1;
    }
    double m = lineVec[0];
    double n = lineVec[1];
    double p = lineVec[2];
    double x0 = linePoint[0];
    double y0 = linePoint[1];
    double z0 = linePoint[2];
    double x1 = pointIn.x;
    double y1 = pointIn.y;
    double z1 = pointIn.z;
    double t = -(m * (x0 - x1) + n * (y0 - y1) + p * (z0 - z1)) / (m * m + n * n + p * p);
    pointOut[0] = float(m * t + x0);
    pointOut[1] = float(n * t + y0);
    pointOut[2] = float(p * t + z0);
    return 0;
}

template <typename PointT>
int ComputeMeanDistanceOfLine2Plane(INTERSECTION_LINE &line, const pcl::PointCloud<PointT> &cornerPoints,
                                    typename pcl::search::KdTree<PointT>::Ptr kdtree, double &meanDistance,
                                    float interval) {
    if (cornerPoints.empty()) {
        return -1;
    }
    //////////////////////////////////////////////////////////////////////////
    line.lineVec.normalize();
    std::vector<LENGTHINDEX> length(cornerPoints.size());
    std::vector<Eigen::Vector3f> projectPoints;
    projectPoints.resize(cornerPoints.size());
    Eigen::Vector3f basePoint;
    for (size_t i = 0; i < cornerPoints.size(); i++) {
        if (0 != ComputeProjectionPointOf3DLine(line.lineVec, line.linePoint, cornerPoints[i], projectPoints[i])) {
            return -1;
        }
        length[i].length = line.lineVec.dot(projectPoints[i] - projectPoints[0]);
        length[i].index = i;
    }

    sort(length.begin(), length.end(), myCompareLess);
    int count = 0;
    double lengthSum = 0;
    std::vector<int> neighbor;
    std::vector<float> neighborLength;

    for (float i = length.front().length; i < length.back().length; i += interval) {
        count++;
        Eigen::Vector3f p = projectPoints[0] + i * line.lineVec;
        pcl::PointXYZ currentPoint(p(0), p(1), p(2));
        kdtree->nearestKSearch(currentPoint, 1, neighbor, neighborLength);
        lengthSum += neighborLength[0];
    }
    meanDistance = lengthSum / count;

    return 0;
}


template<typename _T1, typename _T2>
int ComputeAngleOfTwo3DVector(_T1 &vec1, _T2 &vec2, double &angle) {
    double temp = sqrt(vec1[0] * vec1[0] + vec1[1] * vec1[1] + vec1[2] * vec1[2]) *
                  sqrt(vec2[0] * vec2[0] + vec2[1] * vec2[1] + vec2[2] * vec2[2]);
    if (!temp)//temp==0
    {
        return -1;
    }
    angle = acos((vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2]) / temp);
    return 0;
}

template <typename PointT>
inline double
computeLengthOfPoint23DLine(Eigen::Vector3f &lineVec, PointT &linePoint, PointT &testPoint) {
    if (lineVec[0] == 0 && lineVec[1] == 0 && lineVec[2] == 0) {
        return -1;
    }
    double m = lineVec[0];
    double n = lineVec[1];
    double p = lineVec[2];
    double x0 = linePoint.x;
    double y0 = linePoint.y;
    double z0 = linePoint.z;
    double x1 = testPoint.x;
    double y1 = testPoint.y;
    double z1 = testPoint.z;
    double t = (m * (x0 - x1) + n * (y0 - y1) + p * (z0 - z1)) / (m * m + n * n + p * p);
    double xc = m * t + x1;
    double yc = n * t + y1;
    double zc = p * t + z1;
    return sqrt((x0 - xc) * (x0 - xc) + (y0 - yc) * (y0 - yc) + (z0 - zc) * (z0 - zc));
}

template <typename PointT>
int RansacExtract3Dline(pcl::PointCloud<PointT> &points, std::vector<int> &boudaryPointsIndex,
                        std::vector<std::vector<int>> &extractLines,
                        int maxLinesNum, double inlierThreshold = 0.02, int minLinePointsNum = 30,
                        int eachLineTestNum = 100) {
    if (points.empty() || boudaryPointsIndex.empty() || boudaryPointsIndex.size() < minLinePointsNum) {
        return -1;
    }
    //////////////////////////////////////////////////////////////////////////
    srand(int(time(0)));
    int boundaryPointsNum = boudaryPointsIndex.size();
    extractLines.clear();
    extractLines.reserve(maxLinesNum);
    std::vector<bool> haveBeenUsed(boundaryPointsNum, false);
    int allAvaliablePointsNum = boundaryPointsNum;
    for (int i = 0; i < maxLinesNum * 2; i++) {
        std::vector<std::vector<int>> inliers;
        inliers.reserve(eachLineTestNum);
        std::vector<LENGTHINDEX> lengthIndex;
        lengthIndex.reserve(eachLineTestNum);
        int j;
        for (j = 0; j < eachLineTestNum; j++) {
            int i1 = rand() % boundaryPointsNum;
            int count;
            for (count = 0; count < boundaryPointsNum; count++) {
                if (haveBeenUsed[i1]) {
                    i1 = (++i1) % boundaryPointsNum;
                    continue;
                }
                break;
            }
            if (count == boundaryPointsNum) {
                break;//all boundary points have been used
            }
            int i2 = rand() % boundaryPointsNum;
            for (count = 0; count < boundaryPointsNum; count++) {
                if (haveBeenUsed[i2] || i2 == i1) {
                    i2 = (++i2) % boundaryPointsNum;
                    continue;
                }
                break;
            }
            if (count == boundaryPointsNum) {
                break;//all boundary points have been used
            }
            PointT &start = points[boudaryPointsIndex[i1]];
            PointT &end = points[boudaryPointsIndex[i2]];
            Eigen::Vector3f currentLineVec(start.x - end.x, start.y - end.y, start.z - end.z);
            // compute the length of each point to this line
            std::vector<int> currentInlier;
            currentInlier.push_back(i1);
            currentInlier.push_back(i2);
            for (int k = 0; k < boundaryPointsNum; k++) {
                if (haveBeenUsed[k]) {
                    continue;
                }
                if (computeLengthOfPoint23DLine(currentLineVec, start, points[boudaryPointsIndex[k]]) <
                    inlierThreshold) {
                    currentInlier.push_back(k);
                }

            }
            inliers.push_back(currentInlier);
            LENGTHINDEX tempLength;
            tempLength.index = j;
            tempLength.length = currentInlier.size();
            lengthIndex.push_back(tempLength);
        }
        if (j < eachLineTestNum) {
            //std::cout <<"no avalible points"<<std::endl;
            break;//no avalible points
        }
        partial_sort(lengthIndex.begin(), lengthIndex.begin() + 1, lengthIndex.end(), myCompareGreater);
        if (lengthIndex[0].length < minLinePointsNum) {
            continue;
        }
        int index = lengthIndex[0].index;
        std::vector<int> currentExtractLine(inliers[index].size());
        for (size_t k = 0; k < inliers[index].size(); k++) {
            haveBeenUsed[inliers[index][k]] = true;
            currentExtractLine[k] = boudaryPointsIndex[inliers[index][k]];
        }
        extractLines.push_back(currentExtractLine);
        allAvaliablePointsNum -= lengthIndex[0].length;
        if (allAvaliablePointsNum < minLinePointsNum) {
            break;
        }
    }

    return 0;
}

int Fit3DLine(std::vector<cv::Point3f> &points, cv::Vec6f &param, float *pAccuracy = NULL);


int ConstructPairLinesKdTree(std::vector<INTERSECTION_LINE> &lines,
                             std::vector<Eigen::Vector4f> &planes,
                             std::vector<KdTreeSearchNDim<Eigen::VectorXf, 8>> &kdtrees8,
                             std::vector<KdTreeSearchNDim<Eigen::VectorXf, 6>> &kdtrees6,
                             std::vector<KdTreeSearchNDim<Eigen::VectorXf, 4>> &kdtrees4,
                             std::vector<std::vector<PAIRLINE>> &linesInformation,
                             float scalar);

int ComputeNearstTwoPointsOfTwo3DLine(Eigen::Vector3f &line1Vec, Eigen::Vector3f &line1Point,
                                      Eigen::Vector3f &line2Vec, Eigen::Vector3f &line2Point,
                                      Eigen::Vector3f &point1, Eigen::Vector3f &point2,
                                      double &minLength);

int ClusterTransformation(std::vector<Eigen::Matrix3f> &Rs,
                          std::vector<Eigen::Vector3f> &Ts,
                          float distanceThreshold,
                          float angleThreshold,
                          pcl::IndicesClusters &clusters);


int AreTwoPlanesPenetrable(Eigen::Vector4f &plane1, Eigen::Vector4f &plane2,
                           std::vector<Eigen::Vector3f> &cornerPoints1,
                           std::vector<Eigen::Vector3f> &cornerPoints2,
                           pcl::search::KdTree<pcl::PointXYZ>::Ptr KdTree1,
                           pcl::search::KdTree<pcl::PointXYZ>::Ptr KdTree2,
                           bool &isPentrable, float searchRadius = 0.1, int minPointsNum = 10,
                           float minDistance = 0.03);

template<typename _T1, typename _T2>
inline double computeLengthOfPoint23DLine(_T1 &line, _T2 &point) {
    if (line[0] == 0 && line[1] == 0 && line[2] == 0) {
        return -1;//ֱ�߲�������
    }
    double m = line[0];
    double n = line[1];
    double p = line[2];
    double x0 = line[3];
    double y0 = line[4];
    double z0 = line[5];
    double x1 = point.x;
    double y1 = point.y;
    double z1 = point.z;
    double t = (m * (x0 - x1) + n * (y0 - y1) + p * (z0 - z1)) / (m * m + n * n + p * p);
    double xc = m * t + x1;
    double yc = n * t + y1;
    double zc = p * t + z1;
    return sqrt((x0 - xc) * (x0 - xc) + (y0 - yc) * (y0 - yc) + (z0 - zc) * (z0 - zc));
}

int ComputeIntersectionPointOf23DLine(Eigen::Vector3f &lineVec1, Eigen::Vector3f &linePoint1,
                                      Eigen::Vector3f &lineVec2, Eigen::Vector3f &linePoint2,
                                      Eigen::Vector3f &intersectionPoint);

template <typename PointT>
int ComputeOverlap(typename pcl::search::KdTree<PointT>::Ptr queryTree,
                   typename pcl::search::KdTree<PointT>::Ptr destTree,
                   Eigen::Vector3f &queryCenter,
                   float queryRadius,
                   float inlierDistance, float &overLapRation) {
    typename pcl::PointCloud<PointT>::ConstPtr queryPoints = queryTree->getInputCloud();
    typename pcl::PointCloud<PointT>::ConstPtr destPoints = destTree->getInputCloud();
    std::vector<int> neighbor;
    std::vector<float> neighborLength;
    //compute the coarse overlap region
    if (destTree->radiusSearch(PointT(queryCenter(0), queryCenter(1), queryCenter(2)), queryRadius,
                               neighbor, neighborLength) <= 0) {
        overLapRation = 0;
        return 0;
    }
    typename pcl::PointCloud<PointT>::Ptr neighborPoints(new pcl::PointCloud<PointT>);
    size_t neighborNum = neighbor.size();
    neighborPoints->resize(neighborNum);
    typename pcl::PointCloud<PointT> &ne = *neighborPoints;
    const ::pcl::PointCloud<PointT> &de = *destPoints;
    for (size_t i = 0; i < neighborNum; i++) {
        memcpy(&ne[i], &de[neighbor[i]], sizeof(float) * 3);
    }
    typename pcl::search::KdTree<PointT>::Ptr neighborKdTree(new pcl::search::KdTree<PointT>);
    neighborKdTree->setInputCloud(neighborPoints);
    size_t queryPointsNum = queryPoints->size();
    int count = 0;
    for (size_t i = 0; i < queryPointsNum; i++) {
        if (neighborKdTree->radiusSearch(queryPoints->at(i), inlierDistance, neighbor, neighborLength, 1) > 0) {
            count++;
        }
    }
    overLapRation = double(count) / MIN(queryPointsNum, destPoints->size());

    return 0;
}

/**
 * \brief Query the average spacing of a point cloud.
 * @param cloud The point cloud.
 * @param k The number of nearest points used.
 * @param accurate True to use every point to get an accurate calculation; false to obtain aa approximate
 *                 measure, which uses only a subset (i.e., less than samples) of the points.
 * @param samples  Use how many samples of points for the calculation.
 * @return The average spacing of the point cloud.
 */
float average_spacing(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud, int k = 6, bool accurate = false, int samples = 10000);

void save_vg(
        const pcl::PointCloud<pcl::PointNormal> &cloud,
        const std::vector<std::vector<int>> &planes,
        const std::string &file_name
);

/// A simple progress indicator for console applications. Given percentage = 0.75, the output looks like
///     75% [||||||||||||||||||||||||||||||||||||||||||               ]
void print_progress(float percentage);

#include <string>

#ifdef _WIN32
#include <cstdint>
#else

#include <sys/time.h>

#endif // _WIN32


/**
 * \brief A high resolution stop watch/timer.
 * \details This timer is able to measure the elapsed time with 1 micro-second accuracy on Windows, Linux, and Unix.
 *
 * \class StopWatch easy3d/util/stop_watch.h
 * \see ChronoWatch
 *
 * Usage example:
 *      \code
 *      StopWatch w ;
 *      // do task_1 ...
 *      LOG(INFO) << "task_1 done. Time: " << w.time_string() << " seconds";
 *	    w.start();
 *      // do task_2 ...
 *      LOG(INFO) << "task_2 done. Time: " << w.time_string() << " seconds";
 *      \endcode
 */
class StopWatch {
public:
    /// default constructor. The watch will automatically start after construction.
    StopWatch();

    /// destructor.
    ~StopWatch();

    /// starts the timer
    void start();

    /// restarts the timer. It has the same effect as start()
    void restart();

    /// returns user elapsed time (in seconds) since the construction / start.
    double elapsed_seconds(int num_digits = 1) const;

    /// the elapsed time string, e.g., 88ms, 2.3s, 1.7m, 0.1h. This function automatically determines the best unit.
    std::string time_string(int num_digits = 1) const;

private:
    double seconds() const;

#ifdef _WIN32
    int64_t  freq_;
    int64_t  start_count_;
#else
    timeval start_time_;
#endif

};

#endif