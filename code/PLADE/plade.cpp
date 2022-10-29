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

#include "util.h"
#include "plade.h"
#include "plane_extraction.h"


bool registration(Eigen::Matrix<float, 4, 4> &transformation,
                  pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud,
                  pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
                  const std::vector<PLANE>& target_planes,
                  const std::vector<PLANE>& source_planes
) {
    std::cout << "#point in target point cloud: " << target_cloud->size() << std::endl;
    std::cout << "#point in source point cloud: " << source_cloud->size() << std::endl;
    std::cout << "#planes in target point cloud: " << target_planes.size() << std::endl;
    std::cout << "#planes in source point cloud: " << source_planes.size() << std::endl;
    const float average_space = average_spacing(source_cloud, 6);
    std::cout << "average space in source point cloud: " << average_space << std::endl;

    /////////////////////////////////////////////////////////////////////////

    float downSampleDistance = average_space * 4;
    float minLineConfidence = 1.0;
    int minSupportNumOfRansacLine = 50;
    float lengthThreshold = average_space * 5;
    float angleThreshold = 5.0 / 180 * M_PI;
    float cosAngleThreshold = cos(angleThreshold);
    float faceMatchesWeight = 0.2;
    float inlierDistanceOfFittingLine = average_space * 2;
    int maxCandidateResultNum = 200;
    int maxKdtreeNeighbor = 0;
    float scale = lengthThreshold / cos(M_PI_2 - angleThreshold);

    /////////////////////////////////////////////////////////////////////////

    std::vector<Eigen::Vector4f> mainPlanes;
    for (int i = 0; i < target_planes.size(); ++i) {
        const auto &n = target_planes[i].normal;
        mainPlanes.emplace_back(Eigen::Vector4f(n.x(), n.y(), n.z(), target_planes[i].d));
    }

    std::vector<Eigen::Vector4f> sourcePlanes;
    for (int i = 0; i < source_planes.size(); ++i) {
        const auto &n = source_planes[i].normal;
        sourcePlanes.emplace_back(Eigen::Vector4f(n.x(), n.y(), n.z(), source_planes[i].d));
    }

    StopWatch w;

    //compute the bounding box
    double width, height, depth;
    Eigen::Vector3f mainBoundingCenter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr downSampleMainPoints(new pcl::PointCloud<pcl::PointXYZ>);
    DownSamplePointCloud<pcl::PointNormal>(target_cloud, downSampleMainPoints, downSampleDistance, downSampleDistance,
                                           downSampleDistance);
    //cout<<"first_downsample"<<std::endl;
    if (0 != ComputeBoundingBox<pcl::PointXYZ>(downSampleMainPoints, mainBoundingCenter, width, height, depth)) {
        return false;
    }
    double radius = MAX(MAX(width, height), depth) / 2;

    //compute the bounding box for each plane
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> downSampleMainPlanePoints(mainPlanes.size());
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> mainBoundingBoxCornerPoints(mainPlanes.size());
    std::vector<std::vector<Eigen::Vector3f>> mainBoundingBoxFourCornerPoints(mainPlanes.size());
    std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> mainDownSampleKdTree(mainPlanes.size());
    std::vector<Eigen::Vector3f> mainBoundingBoxCenterForEachPlane(mainPlanes.size());
    std::vector<float> mainBoundingBoxRadiusForEachPlane(mainPlanes.size());
    for (size_t i = 0; i < mainPlanes.size(); i++) {
        downSampleMainPlanePoints[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        mainBoundingBoxCornerPoints[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempPoints1(new pcl::PointCloud<pcl::PointXYZ>);
        tempPoints1->reserve(target_planes[i].size());
        for (size_t j = 0; j < target_planes[i].size(); j++) {
            int idx = target_planes[i][j];
            const auto &p = target_cloud->at(idx);
            tempPoints1->push_back(pcl::PointXYZ(p.x, p.y, p.z));
        }
        DownSamplePointCloud<pcl::PointXYZ>(tempPoints1, downSampleMainPlanePoints[i], downSampleDistance,
                                            downSampleDistance,
                                            downSampleDistance);
        Eigen::Vector3f centerPoint;
        double width, height, depth;
        ComputeBoundingBox<pcl::PointXYZ>(downSampleMainPlanePoints[i], centerPoint, width, height, depth,
                                          &(*mainBoundingBoxCornerPoints[i]));
        int start = 0, end = 4;
        ProjectPoints2Plane<pcl::PointXYZ>(*mainBoundingBoxCornerPoints[i], mainPlanes[i],
                                           mainBoundingBoxFourCornerPoints[i],
                                           &start, &end);
        mainBoundingBoxCenterForEachPlane[i] =
                (mainBoundingBoxFourCornerPoints[i][0] + mainBoundingBoxFourCornerPoints[i][2]) / 2;
        mainBoundingBoxRadiusForEachPlane[i] =
                (mainBoundingBoxFourCornerPoints[i][0] - mainBoundingBoxFourCornerPoints[i][2]).norm() / 2;
        //construct kdTree;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tempKdTree(new pcl::search::KdTree<pcl::PointXYZ>);
        tempKdTree->setInputCloud(downSampleMainPlanePoints[i]);
        mainDownSampleKdTree[i] = tempKdTree;
    }

    // compute intersection lines
    size_t mainPlanesNum = mainPlanes.size();
    std::vector<INTERSECTION_LINE> mainIntersectionLines;
    mainIntersectionLines.reserve(mainPlanesNum * mainPlanesNum / 2);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr downsampleKdtree(new pcl::search::KdTree<pcl::PointXYZ>);

    for (size_t i = 0; i < mainPlanesNum; i++) {
        for (size_t j = i + 1; j < mainPlanesNum; j++) {
            INTERSECTION_LINE tempIntersect;
            if (0 != ComputeIntersectionLineOfTwoPlanes(mainPlanes[i], mainPlanes[j], tempIntersect.lineVec,
                                                        tempIntersect.linePoint)) {
                continue;
            }
            //filter out lines far away from the center
            Eigen::Vector3f tempVector = tempIntersect.linePoint - mainBoundingCenter;
            double distance = sqrt(tempVector.squaredNorm() - std::pow(tempVector.dot(tempIntersect.lineVec), 2));
            if (distance > radius) {
                continue;
            }
            //compute the confidence for each line and filter out the lines with low confidence
            downsampleKdtree->setInputCloud(downSampleMainPlanePoints[i]);
            tempIntersect.distance.resize(2);
            if (0 != ComputeMeanDistanceOfLine2Plane(tempIntersect, *mainBoundingBoxCornerPoints[i],
                                                     downsampleKdtree, tempIntersect.distance[0], 0.5)) {
                continue;
            }
            downsampleKdtree->setInputCloud(downSampleMainPlanePoints[j]);
            if (0 != ComputeMeanDistanceOfLine2Plane(tempIntersect, *mainBoundingBoxCornerPoints[j],
                                                     downsampleKdtree, tempIntersect.distance[1], 0.5)) {
                continue;
            }
            tempIntersect.confidence.push_back(
                    downSampleMainPlanePoints[i]->size() * downSampleDistance * downSampleDistance /
                    tempIntersect.distance[0]);
            tempIntersect.confidence.push_back(
                    downSampleMainPlanePoints[j]->size() * downSampleDistance * downSampleDistance /
                    tempIntersect.distance[1]);
            if (MIN(tempIntersect.confidence[0], tempIntersect.confidence[1]) < minLineConfidence) {
                //continue;
            }
            tempIntersect.supportPlanes.push_back(static_cast<int>(i));
            tempIntersect.supportPlanes.push_back(static_cast<int>(j));
            if (0 != ComputeAngleOfTwo3DVector(mainPlanes[i], mainPlanes[j], tempIntersect.planeAngle)) {
                //continue;
            }
            tempIntersect.isIntersectionLine = true;
            mainIntersectionLines.push_back(tempIntersect);
        }
    }
    //cout<<"mainIntersectionLines:"<<mainIntersectionLines.size()<<std::endl;

    std::vector<int> boundaryPointsIndex, filteredBoundaryPointsIndex;
    //DetectBoundaryPoints(target_cloud,boundaryPointsIndex,0.35,0.35);
    filteredBoundaryPointsIndex = boundaryPointsIndex;
    //FilterPoints(target_cloud,boundaryPointsIndex,pcl::PointNormal(0,0,0),filteredBoundaryPointsIndex,true);
    //assign each boundary point to a plane
    std::vector<int> pointToPlaneIndex(target_cloud->size(), -1);
    for (size_t i = 0; i < target_planes.size(); i++) {
        const std::vector<int> &current = target_planes[i];
        for (size_t j = 0; j < target_planes[i].size(); j++) {
            pointToPlaneIndex[current[j]] = static_cast<int>(i);
        }
    }
    std::vector<std::vector<int>> boundaryPointsOfEachPlane(target_planes.size());
    for (size_t i = 0; i < filteredBoundaryPointsIndex.size(); i++) {
        if (pointToPlaneIndex[filteredBoundaryPointsIndex[i]] < 0) {
            continue;
        }
        boundaryPointsOfEachPlane[pointToPlaneIndex[filteredBoundaryPointsIndex[i]]].push_back(
                filteredBoundaryPointsIndex[i]);
    }


    std::vector<INTERSECTION_LINE> mainBoundaryIntersectionLines;
    for (size_t i = 0; i < boundaryPointsOfEachPlane.size(); i++) {
        std::vector<std::vector<int>> extractLines;
        RansacExtract3Dline(*target_cloud, boundaryPointsOfEachPlane[i], extractLines, 20,
                            inlierDistanceOfFittingLine, minSupportNumOfRansacLine, 100);
        for (size_t j = 0; j < extractLines.size(); j++) {
            std::vector<cv::Point3f> currentLinePoints(extractLines[j].size());
            for (size_t k = 0; k < extractLines[j].size(); k++) {
                pcl::PointNormal &p = (*target_cloud)[extractLines[j][k]];
                currentLinePoints[k].x = p.x;
                currentLinePoints[k].y = p.y;
                currentLinePoints[k].z = p.z;
            }
            cv::Vec6f line;
            if (0 != Fit3DLine(currentLinePoints, line)) {
                continue;
            }
            Eigen::Vector3f lineVec(line[0], line[1], line[2]);
            Eigen::Vector3f start(line[3], line[4], line[5]);
            Eigen::Vector3f end = start + lineVec;
            pcl::PointCloud<pcl::PointXYZ> startEndPoint;
            startEndPoint.reserve(2);
            startEndPoint.push_back(pcl::PointXYZ(start[0], start[1], start[2]));
            startEndPoint.push_back(pcl::PointXYZ(end[0], end[1], end[2]));
            std::vector<Eigen::Vector3f> projectStartEnd;
            if (0 != ProjectPoints2Plane(startEndPoint, mainPlanes[i], projectStartEnd)) {
                continue;
            }
            INTERSECTION_LINE boundaIntersectionLine;
            boundaIntersectionLine.lineVec = (projectStartEnd[1] - projectStartEnd[0]);
            boundaIntersectionLine.lineVec.normalize();
            boundaIntersectionLine.linePoint = projectStartEnd[0];
            boundaIntersectionLine.supportPlanes.push_back(static_cast<int>(i));
            boundaIntersectionLine.isIntersectionLine = false;
            mainBoundaryIntersectionLines.push_back(boundaIntersectionLine);
        }
    }

    size_t mainBoundaryIntersectionLinesNum = mainBoundaryIntersectionLines.size();
    //cout<<"mainBoundaryIntersectionLinesNum:"<<mainBoundaryIntersectionLinesNum<<std::endl;

    //copy
    std::copy(mainBoundaryIntersectionLines.begin(), mainBoundaryIntersectionLines.end(),
              std::back_inserter(mainIntersectionLines));

    size_t mainLinesNum = mainIntersectionLines.size();
    //cout<<"totalLinesNum:"<<mainLinesNum<<std::endl;

    std::vector<KdTreeSearchNDim<Eigen::VectorXf, 8>>
            kdtrees8;
    std::vector<KdTreeSearchNDim<Eigen::VectorXf, 6>>
            kdtrees6;
    std::vector<KdTreeSearchNDim<Eigen::VectorXf, 4>>
            kdtrees4;
    std::vector<std::vector<PAIRLINE>> mainLinesInformation;
    //cout<<"pairlines"<<std::endl;
    ConstructPairLinesKdTree(mainIntersectionLines, mainPlanes, kdtrees8, kdtrees6, kdtrees4, mainLinesInformation,
                             scale);

    //cout<<"pairlinesend"<<std::endl;
    //compute length of each pair of lines
    std::vector<std::vector<NearstPointsTwoLine>> mainLineLength(mainLinesNum);
    for (size_t i = 0; i < mainLinesNum; i++) {
        mainLineLength[i].resize(mainLinesNum);
        for (int j = 0; j < mainLinesNum; j++) {
            if (i > j) {
                mainLineLength[i][j].points1 = mainLineLength[j][i].points2;
                mainLineLength[i][j].points2 = mainLineLength[j][i].points1;
                mainLineLength[i][j].length = mainLineLength[j][i].length;
                mainLineLength[i][j].angle = mainLineLength[j][i].angle;
                mainLineLength[i][j].minAngle = mainLineLength[j][i].minAngle;
            } else if (i < j) {
                if (0 != ComputeNearstTwoPointsOfTwo3DLine(mainIntersectionLines[i].lineVec,
                                                           mainIntersectionLines[i].linePoint,
                                                           mainIntersectionLines[j].lineVec,
                                                           mainIntersectionLines[j].linePoint,
                                                           mainLineLength[i][j].points1,
                                                           mainLineLength[i][j].points2,
                                                           mainLineLength[i][j].length)) {
                    mainLineLength[i][j].length = -1;
                }
                ComputeAngleOfTwo3DVector(mainIntersectionLines[i].lineVec, mainIntersectionLines[j].lineVec,
                                          mainLineLength[i][j].angle);
                if (mainLineLength[i][j].angle > M_PI_2) {
                    mainLineLength[i][j].minAngle = M_PI - mainLineLength[i][j].angle;
                }
            }
        }
    }

    //fout<<sourceFileName+".bpn"<<std::endl;
    size_t currentPlanesNum = sourcePlanes.size();

    //compute the bounding box
    double currentWidth, currentHeight, currentDepth;
    Eigen::Vector3f currentBoundingCenter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr downSamplePoints(new pcl::PointCloud<pcl::PointXYZ>);
    DownSamplePointCloud<pcl::PointNormal>(source_cloud, downSamplePoints, downSampleDistance, downSampleDistance,
                                           downSampleDistance);
    if (0 != ComputeBoundingBox<pcl::PointXYZ>(downSamplePoints, currentBoundingCenter, currentWidth, currentHeight,
                                               currentDepth)) {
        return false;
    }
    double currentRadius = MAX(MAX(currentWidth, currentHeight), currentDepth) / 2;

    //compute the bounding box for each plane
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> downSampleCurrentPlanePoints(sourcePlanes.size());
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> currentBoundingBoxCornerPoints(sourcePlanes.size());
    std::vector<std::vector<Eigen::Vector3f>> currentBoundingBoxFourCornerPoints(sourcePlanes.size());
    std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> currentDownSampleKdTree(sourcePlanes.size());
    std::vector<Eigen::Vector3f> currentBoundingBoxCenterForEachPlane(sourcePlanes.size());
    std::vector<float> currentBoundingBoxRadiusForEachPlane(sourcePlanes.size());
    for (size_t i = 0; i < sourcePlanes.size(); i++) {
        downSampleCurrentPlanePoints[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        currentBoundingBoxCornerPoints[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempPoints1(new pcl::PointCloud<pcl::PointXYZ>);
        tempPoints1->reserve(source_planes[i].size());
        for (size_t j = 0; j < source_planes[i].size(); j++) {
            int idx = source_planes[i][j];
            const auto &p = source_cloud->at(idx);
            tempPoints1->push_back(pcl::PointXYZ(p.x, p.y, p.z));
        }
        DownSamplePointCloud<pcl::PointXYZ>(tempPoints1, downSampleCurrentPlanePoints[i],
                                            downSampleDistance, downSampleDistance, downSampleDistance);
        Eigen::Vector3f centerPoint;
        double width, height, depth;
        ComputeBoundingBox<pcl::PointXYZ>(downSampleCurrentPlanePoints[i], centerPoint, width, height, depth,
                                          &(*currentBoundingBoxCornerPoints[i]));
        int start = 0, end = 4;
        ProjectPoints2Plane(*currentBoundingBoxCornerPoints[i], sourcePlanes[i],
                            currentBoundingBoxFourCornerPoints[i], &start, &end);
        currentBoundingBoxCenterForEachPlane[i] =
                (currentBoundingBoxFourCornerPoints[i][0] + currentBoundingBoxFourCornerPoints[i][2]) / 2;
        currentBoundingBoxRadiusForEachPlane[i] =
                (currentBoundingBoxFourCornerPoints[i][0] - currentBoundingBoxFourCornerPoints[i][2]).norm() / 2;
        //construct kdTree;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tempKdTree(new pcl::search::KdTree<pcl::PointXYZ>);
        tempKdTree->setInputCloud(downSampleCurrentPlanePoints[i]);
        currentDownSampleKdTree[i] = tempKdTree;
    }

    std::vector<INTERSECTION_LINE> currentIntersectionLines;
    int count = 0;
    for (size_t i = 0; i < currentPlanesNum; i++) {
        for (size_t j = i + 1; j < currentPlanesNum; j++) {
            INTERSECTION_LINE tempIntersect;
            if (0 != ComputeIntersectionLineOfTwoPlanes(sourcePlanes[i], sourcePlanes[j], tempIntersect.lineVec,
                                                        tempIntersect.linePoint)) {
                continue;
            }
            //filter out lines far away from the center
            Eigen::Vector3f tempVector = tempIntersect.linePoint - currentBoundingCenter;
            double distance = sqrt(tempVector.squaredNorm() - std::pow(tempVector.dot(tempIntersect.lineVec), 2));
            if (distance > currentRadius) {
                continue;
            }
            //compute the confidence for each line and filter out the lines with low confidence
            downsampleKdtree->setInputCloud(downSampleCurrentPlanePoints[i]);
            tempIntersect.distance.resize(2);
            if (0 != ComputeMeanDistanceOfLine2Plane(tempIntersect, *currentBoundingBoxCornerPoints[i],
                                                     downsampleKdtree, tempIntersect.distance[0], 0.5)) {
                continue;
            }
            downsampleKdtree->setInputCloud(downSampleCurrentPlanePoints[j]);
            if (0 != ComputeMeanDistanceOfLine2Plane(tempIntersect, *currentBoundingBoxCornerPoints[j],
                                                     downsampleKdtree, tempIntersect.distance[1], 0.5)) {
                continue;
            }
            tempIntersect.confidence.push_back(
                    downSampleCurrentPlanePoints[i]->size() * downSampleDistance * downSampleDistance /
                    tempIntersect.distance[0]);
            tempIntersect.confidence.push_back(
                    downSampleCurrentPlanePoints[j]->size() * downSampleDistance * downSampleDistance /
                    tempIntersect.distance[1]);
            if (MIN(tempIntersect.confidence[0], tempIntersect.confidence[1]) < minLineConfidence) {
                //continue;
            }
            tempIntersect.supportPlanes.push_back(static_cast<int>(i));
            tempIntersect.supportPlanes.push_back(static_cast<int>(j));
            if (0 != ComputeAngleOfTwo3DVector(sourcePlanes[i], sourcePlanes[j], tempIntersect.planeAngle)) {
                //continue;
            }
            tempIntersect.isIntersectionLine = true;
            currentIntersectionLines.push_back(tempIntersect);
        }
    }

    std::vector<int> currentBoundaryPointsIndex;
    //DetectBoundaryPoints(points,currentBoundaryPointsIndex,0.35,0.35);
    //assign each boundary point to a plane
    std::vector<int> sourcePointToPlaneIndex(source_cloud->size(), -1);
    for (size_t i = 0; i < source_planes.size(); i++) {
        const std::vector<int> &current = source_planes[i];
        for (size_t j = 0; j < source_planes[i].size(); j++) {
            sourcePointToPlaneIndex[current[j]] = static_cast<int>(i);
        }
    }
    std::vector<std::vector<int>> currentBoundaryPointsOfEachPlane(source_planes.size());
    for (size_t i = 0; i < currentBoundaryPointsIndex.size(); i++) {
        if (sourcePointToPlaneIndex[currentBoundaryPointsIndex[i]] < 0) {
            continue;
        }
        currentBoundaryPointsOfEachPlane[sourcePointToPlaneIndex[currentBoundaryPointsIndex[i]]].push_back(
                currentBoundaryPointsIndex[i]);
    }

    std::vector<INTERSECTION_LINE> currentBoundaryIntersectionLines;
    for (size_t i = 0; i < currentBoundaryPointsOfEachPlane.size(); i++) {
        std::vector<std::vector<int>> extractLines;
        RansacExtract3Dline(*source_cloud, currentBoundaryPointsOfEachPlane[i], extractLines, 20,
                            inlierDistanceOfFittingLine, minSupportNumOfRansacLine, 100);
        for (size_t j = 0; j < extractLines.size(); j++) {
            std::vector<cv::Point3f> currentLinePoints(extractLines[j].size());
            for (size_t k = 0; k < extractLines[j].size(); k++) {
                pcl::PointNormal &p = (*source_cloud)[extractLines[j][k]];
                currentLinePoints[k].x = p.x;
                currentLinePoints[k].y = p.y;
                currentLinePoints[k].z = p.z;
            }
            cv::Vec6f line;
            if (0 != Fit3DLine(currentLinePoints, line)) {
                continue;
            }
            Eigen::Vector3f lineVec(line[0], line[1], line[2]);
            Eigen::Vector3f start(line[3], line[4], line[5]);
            Eigen::Vector3f end = start + lineVec;
            pcl::PointCloud<pcl::PointXYZ> startEndPoint;
            startEndPoint.reserve(2);
            startEndPoint.push_back(pcl::PointXYZ(start[0], start[1], start[2]));
            startEndPoint.push_back(pcl::PointXYZ(end[0], end[1], end[2]));
            std::vector<Eigen::Vector3f> projectStartEnd;
            if (0 != ProjectPoints2Plane(startEndPoint, sourcePlanes[i], projectStartEnd)) {
                continue;
            }
            INTERSECTION_LINE boundaIntersectionLine;
            boundaIntersectionLine.lineVec = (projectStartEnd[1] - projectStartEnd[0]);
            boundaIntersectionLine.lineVec.normalize();
            boundaIntersectionLine.linePoint = projectStartEnd[0];
            boundaIntersectionLine.supportPlanes.push_back(static_cast<int>(i));
            boundaIntersectionLine.isIntersectionLine = false;
            currentBoundaryIntersectionLines.push_back(boundaIntersectionLine);
        }
    }

    //cout<<"currentIntersectionLines:"<<currentIntersectionLines.size()<<std::endl;
    //copy
    std::copy(currentBoundaryIntersectionLines.begin(), currentBoundaryIntersectionLines.end(),
              std::back_inserter(currentIntersectionLines));

    size_t currentLinesNum = currentIntersectionLines.size();
    size_t currentBoundaryIntersectionLinesNum = currentBoundaryIntersectionLines.size();
    //cout<<"currentBoundaryIntersectionLinesNum:"<<currentBoundaryIntersectionLinesNum<<std::endl;
    //cout<<"totalLinesNum:"<<currentLinesNum<<std::endl;

    // find initial matches for each line
    std::vector<std::vector<int>> coarseMatches(currentLinesNum);
    std::vector<float> matchesLength;
    std::vector<std::vector<NearstPointsTwoLine>> currentLineLength(currentLinesNum);
    for (size_t i = 0; i < currentLinesNum; i++) {
        currentLineLength[i].resize(currentLinesNum);
        for (int j = 0; j < currentLinesNum; j++) {
            if (i > j) {
                currentLineLength[i][j].points1 = currentLineLength[j][i].points2;
                currentLineLength[i][j].points2 = currentLineLength[j][i].points1;
                currentLineLength[i][j].length = currentLineLength[j][i].length;
                currentLineLength[i][j].angle = currentLineLength[j][i].angle;
                currentLineLength[i][j].minAngle = currentLineLength[j][i].minAngle;
            } else if (i < j) {
                if (0 != ComputeNearstTwoPointsOfTwo3DLine(currentIntersectionLines[i].lineVec,
                                                           currentIntersectionLines[i].linePoint,
                                                           currentIntersectionLines[j].lineVec,
                                                           currentIntersectionLines[j].linePoint,
                                                           currentLineLength[i][j].points1,
                                                           currentLineLength[i][j].points2,
                                                           currentLineLength[i][j].length)) {
                    currentLineLength[i][j].length = -1;
                }
                currentLineLength[i][j].length = currentLineLength[i][j].length / scale;
                ComputeAngleOfTwo3DVector(currentIntersectionLines[i].lineVec, currentIntersectionLines[j].lineVec,
                                          currentLineLength[i][j].angle);
                currentLineLength[i][j].minAngle = (currentLineLength[i][j].angle > M_PI_2 ? M_PI -
                                                                                             currentLineLength[i][j].angle
                                                                                           : currentLineLength[i][j].angle);
            }

        }
    }

    MatchInformation current, main;
    current.boundingCenter = currentBoundingCenter;
    current.pBoundingBoxFourCornerPoints = &currentBoundingBoxFourCornerPoints;
    current.pDownSampleKdTree = &currentDownSampleKdTree;
    current.pDownSamplePlanePoints = &downSampleCurrentPlanePoints;
    current.pIntersectionLine = &currentIntersectionLines;
    current.pNearstPointsTwoLines = &currentLineLength;
    current.points = downSamplePoints;
    current.originalPoints = source_cloud;
    current.pPlanes = &sourcePlanes;
    current.pSupportPlaneIndex = &source_planes;
    current.pBoundingBoxCenterForEachPlane = &currentBoundingBoxCenterForEachPlane;
    current.pBoundingBoxRadiusForEachPlane = &currentBoundingBoxRadiusForEachPlane;

    main.boundingCenter = mainBoundingCenter;
    main.pBoundingBoxFourCornerPoints = &mainBoundingBoxFourCornerPoints;
    main.pDownSampleKdTree = &mainDownSampleKdTree;
    main.pDownSamplePlanePoints = &downSampleMainPlanePoints;
    main.pIntersectionLine = &mainIntersectionLines;
    main.pNearstPointsTwoLines = &mainLineLength;
    main.points = downSampleMainPoints;
    main.originalPoints = target_cloud;
    main.pPlanes = &mainPlanes;
    main.pSupportPlaneIndex = &target_planes;
    main.pBoundingBoxCenterForEachPlane = &mainBoundingBoxCenterForEachPlane;
    main.pBoundingBoxRadiusForEachPlane = &mainBoundingBoxRadiusForEachPlane;

    std::vector<std::pair<int, int>> linesToBeMatched;
    linesToBeMatched.reserve(currentLinesNum * currentLinesNum / 2);
    float angleThresh = cos(10.0 / 180 * M_PI);
    for (int i = 0; i < currentLinesNum; i++) {
        for (int j = i + 1; j < currentLinesNum; j++) {
            if (abs(currentIntersectionLines[i].lineVec.dot(currentIntersectionLines[j].lineVec)) > angleThresh) {
                continue;
            }
            linesToBeMatched.push_back(std::pair<int, int>(i, j));
        }
    }

    Parameter parameter;
    parameter.lengthThreshold = lengthThreshold;
    parameter.angleThreshold = angleThreshold;
    parameter.cosAngleThreshold = cosAngleThreshold;
    parameter.maxCandidateResultNum = maxCandidateResultNum;
    parameter.mainLinesInformation = &mainLinesInformation;
    parameter.kdtrees8 = &kdtrees8;
    parameter.kdtrees6 = &kdtrees6;
    parameter.kdtrees4 = &kdtrees4;
    parameter.maxNeighbor = maxKdtreeNeighbor;
    parameter.maxRadius = radius;

    std::vector<MatchedResult> matchedResults;
    MatchingLines(current, main, linesToBeMatched, coarseMatches, matchedResults, parameter);
    if (matchedResults.empty()) {
        std::cerr << "registration failed: no matched result found" << std::endl;
        return false;
    }

    w.start();
    std::cout << "registration..." << std::endl;

    downsampleKdtree->setInputCloud(downSampleMainPoints);
    std::vector<LENGTHINDEX> overlapLength(matchedResults.size());
    for (int i = 0; i < matchedResults.size(); i++) {
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation;
        transformation = pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4::Identity();
        transformation.block(0, 0, 3, 3) = matchedResults[i].R;
        transformation.block(0, 3, 3, 1) = matchedResults[i].T;

        pcl::PointCloud<pcl::PointXYZ>::Ptr transPoints(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*downSamplePoints, *transPoints, transformation);
        Eigen::Vector3f currentCenter = matchedResults[i].R * currentBoundingCenter + matchedResults[i].T;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
        kdtree->setInputCloud(transPoints);
        ComputeOverlap<pcl::PointXYZ>(kdtree, downsampleKdtree, currentCenter, currentRadius, downSampleDistance,
                                      overlapLength[i].length);
        overlapLength[i].index = i;
        overlapLength[i].length = 0.2 * (matchedResults[i].matchedPlanes.size() / double(currentPlanesNum)) +
                                  0.8 * overlapLength[i].length;
        print_progress(float(i+1)/matchedResults.size());
    }
    sort(overlapLength.begin(), overlapLength.end(), myCompareGreater);
    //cout<<"overlapLength:"<<overlapLength[0].length<<std::endl;
    int index = overlapLength[0].index;

    transformation.setIdentity();
    for (int i=0; i<3; ++i) {
        for (int j=0; j<3; ++j)
            transformation(i, j) = matchedResults[index].R(i, j);
    }
    for (int i=0; i<3; ++i)
        transformation(i, 3) = matchedResults[index].T(i, 0);

    std::cout << std::endl << "done. time: " << w.time_string() << std::endl;

    return true;
}


bool registration(Eigen::Matrix<float, 4, 4> &transformation,
                  pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud,
                  pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
                  int ransac_min_support_target,
                  int ransac_min_support_source
) {
    StopWatch w;
    std::cout << "extracting planes for target point cloud...\n";
    const std::vector<PLANE> target_planes = PlaneExtraction::detect(*target_cloud, ransac_min_support_target, 0.005f,
                                                                     0.02f, 0.8f, 0.001f);
    std::cout << "extracting planes for source point cloud...\n";
    const std::vector<PLANE> source_planes = PlaneExtraction::detect(*source_cloud, ransac_min_support_source, 0.005f,
                                                                     0.02f, 0.8f, 0.001f);
    std::cout << "run time for plane extraction: " << w.time_string() << std::endl;

    return registration(transformation, target_cloud, source_cloud, target_planes, source_planes);
}


std::vector<PLANE> extract(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, int init_min_support = 10000) {
    const int min_num = 10;
    const int max_num = 40;
    const int min_allowed_support = 50; // a plane much have >= this number points

    std::vector<PLANE> planes = PlaneExtraction::detect(*cloud, init_min_support, 0.005f, 0.02f, 0.8f, 0.001f);
    if (planes.size() >= min_num && planes.size() <=max_num)
        return planes;

    if (planes.size() > max_num) { // then we take the top 40
        struct {
            bool operator()(const PLANE& a, const PLANE& b) const { return a.size() >= b.size(); }
        } greater;
        std::sort(planes.begin(), planes.end(), greater);

        auto result = std::vector<PLANE>(planes.begin(), planes.begin() + max_num);
        std::cout << result.size() << " of the " << planes.size() << " extracted planes will be used for registration" << std::endl;
        return result;
    }

    const int max_trials = 5; // max call RANSAC 5 times
    int min_support = init_min_support / 2;
    int trials = 1;
    while (planes.size() < min_num && trials < max_trials && min_support >= min_allowed_support) {
//        std::cout << "trial: " << trials << ". min support: " << min_support << std::endl;
        planes = PlaneExtraction::detect(*cloud, min_support, 0.005f, 0.02f, 0.8f, 0.001f);
        min_support /= 2;
        ++trials;
    }
    if (trials > 1)
        std::cout << "min_support = " << min_support << " used for extracting the " << planes.size()  << " planes from point cloud" << std::endl;

    return planes;
}


bool registration(Eigen::Matrix<float, 4, 4> &transformation,
                  pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud,
                  pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud
) {
    StopWatch w;
    std::cout << "extracting planes for both point clouds...\n";

    std::vector <PLANE> target_planes = extract(target_cloud);
    if (target_planes.size() < 10) {
        std::cerr << "two few (only " << target_planes.size() << ") planes extracted from the target point cloud"
                  << std::endl;
        return false;
    }

    std::vector <PLANE> source_planes = extract(source_cloud);
    if (source_planes.size() < 10) {
        std::cerr << "two few (only " << source_planes.size() << ") planes extracted from the source point cloud"
                  << std::endl;
        return false;
    }

    std::cout << "run time for plane extraction: " << w.time_string() << std::endl;

    return registration(transformation, target_cloud, source_cloud, target_planes, source_planes);
}