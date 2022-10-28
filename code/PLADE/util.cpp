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
#include <fstream>
#include <opencv2/imgproc.hpp>


void MatchingLines(MatchInformation current,
	MatchInformation main,
	std::vector<std::pair<int, int>> linesTobeMatched,
	std::vector<std::vector<int>> coarseMatches,
	std::vector<MatchedResult>& pMatchedResult,
	Parameter parameter)
{
	std::vector<INTERSECTION_LINE> &currentIntersectionLines = *(current.pIntersectionLine);
	std::vector<std::vector<NearstPointsTwoLine>> &currentLineLength = *(current.pNearstPointsTwoLines);
	std::vector<Eigen::Vector4f> &planes = *(current.pPlanes);
	const std::vector<PLANE> &supportPlaneIndex = *(current.pSupportPlaneIndex);
	pcl::PointCloud<pcl::PointXYZ>::Ptr points = current.points;
	pcl::PointCloud<pcl::PointNormal>::Ptr currentOriginalPoints = current.originalPoints;
	Eigen::Vector3f currentBoundingCenter = current.boundingCenter;
	std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> &currentDownSampleKdTree = *(current.pDownSampleKdTree);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &downSampleCurrentPlanePoints = *(current.pDownSamplePlanePoints);
	std::vector<std::vector<Eigen::Vector3f>> &currentBoundingBoxFourCornerPoints = *(current.pBoundingBoxFourCornerPoints);
	std::vector<Eigen::Vector3f> &currentBoundingBoxCenterForEachPlane = *(current.pBoundingBoxCenterForEachPlane);
	std::vector<float> &currentBoundingBoxRadiusForEachPlane = *(current.pBoundingBoxRadiusForEachPlane);


	std::vector<INTERSECTION_LINE> &mainIntersectionLines = *(main.pIntersectionLine);
	std::vector<std::vector<NearstPointsTwoLine>> &mainLineLength = *(main.pNearstPointsTwoLines);
	std::vector<Eigen::Vector4f> &mainPlanes = *(main.pPlanes);
	const std::vector<PLANE> &supportMainPlaneIndex = *(main.pSupportPlaneIndex);
	pcl::PointCloud<pcl::PointXYZ>::Ptr mainPoints = main.points;
	pcl::PointCloud<pcl::PointNormal>::Ptr mainOriginalPoints = main.originalPoints;
	Eigen::Vector3f mianBoundingCenter = main.boundingCenter;
	std::vector<std::vector<Eigen::Vector3f>> &mainBoundingBoxFourCornerPoints = *(main.pBoundingBoxFourCornerPoints);
	std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> &mainDownSampleKdTree = *(main.pDownSampleKdTree);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &downSampleMainPlanePoints = *(main.pDownSamplePlanePoints);
	std::vector<Eigen::Vector3f> &mainBoundingBoxCenterForEachPlane = *(main.pBoundingBoxCenterForEachPlane);
	std::vector<float> &mainBoundingBoxRadiusForEachPlane = *(main.pBoundingBoxRadiusForEachPlane);

	//kdtrees8 22
	//kdtrees6 12 22-12 21 22-21
	//kdtrees4 11 12-11 21-11 22-11
	//linesInformation 22 12 22-12 21 22-21 11 12-11 21-11 22-11
	std::vector<KdTreeSearchNDim<Eigen::VectorXf, 8>> &kdtrees8 = *parameter.kdtrees8;
	std::vector<KdTreeSearchNDim<Eigen::VectorXf, 6>> &kdtrees6 = *parameter.kdtrees6;
	std::vector<KdTreeSearchNDim<Eigen::VectorXf, 4>> &kdtrees4 = *parameter.kdtrees4;
	std::vector<std::vector<PAIRLINE>> &mainLinesInformation = *parameter.mainLinesInformation;
	KdTreeSearchNDim<Eigen::VectorXf, 8> &kdtree22 = kdtrees8[0];
	KdTreeSearchNDim<Eigen::VectorXf, 6> &kdtree12 = kdtrees6[0];
	KdTreeSearchNDim<Eigen::VectorXf, 6> &kdtree2212 = kdtrees6[1];
	KdTreeSearchNDim<Eigen::VectorXf, 6> &kdtree21 = kdtrees6[2];
	KdTreeSearchNDim<Eigen::VectorXf, 6> &kdtree2221 = kdtrees6[3];
	KdTreeSearchNDim<Eigen::VectorXf, 4> &kdtree11 = kdtrees4[0];
	KdTreeSearchNDim<Eigen::VectorXf, 4> &kdtree1211 = kdtrees4[1];
	KdTreeSearchNDim<Eigen::VectorXf, 4> &kdtree2111 = kdtrees4[2];
	KdTreeSearchNDim<Eigen::VectorXf, 4> &kdtree2211 = kdtrees4[3];

	std::vector<PAIRLINE> &lf22 = mainLinesInformation[0];
	std::vector<PAIRLINE> &lf12 = mainLinesInformation[1];
	std::vector<PAIRLINE> &lf2212 = mainLinesInformation[2];
	std::vector<PAIRLINE> &lf21 = mainLinesInformation[3];
	std::vector<PAIRLINE> &lf2221 = mainLinesInformation[4];
	std::vector<PAIRLINE> &lf11 = mainLinesInformation[5];
	std::vector<PAIRLINE> &lf1211 = mainLinesInformation[6];
	std::vector<PAIRLINE> &lf2111 = mainLinesInformation[7];
	std::vector<PAIRLINE> &lf2211 = mainLinesInformation[8];

	std::vector<std::vector<QueryResult>> queryResult(9);
	std::vector<QueryResult> &query22 = queryResult[0];
	std::vector<QueryResult> &query12 = queryResult[1];
	std::vector<QueryResult> &query2212 = queryResult[2];
	std::vector<QueryResult> &query21 = queryResult[3];
	std::vector<QueryResult> &query2221 = queryResult[4];
	std::vector<QueryResult> &query11 = queryResult[5];
	std::vector<QueryResult> &query1211 = queryResult[6];
	std::vector<QueryResult> &query2111 = queryResult[7];
	std::vector<QueryResult> &query2211 = queryResult[8];

    pMatchedResult.clear();
    pMatchedResult.reserve(200);

	size_t currentPlanesNum = planes.size();
	size_t mainPlanesNum = mainPlanes.size();

	//Parameter
	float lengthThreshold = parameter.lengthThreshold;
	float angleThreshold = parameter.angleThreshold;
	float cosAngleTh = parameter.cosAngleThreshold;
	int maxNeighbor = parameter.maxNeighbor;
	double maxNeighborDistance = 0.04;
	double squaredMaxNeighbordistance = maxNeighborDistance * maxNeighborDistance;

	std::vector<Eigen::Vector3f> suppotPlanes(planes.size());
	for (size_t i = 0; i < planes.size(); i++)
	{
		suppotPlanes[i] = planes[i].block(0, 0, 3, 1);
	}
	std::vector<PAIRLINE> queryLines;
	queryLines.reserve(linesTobeMatched.size() * 4);
	std::vector<std::vector<int>> matchedLinesIndex;
	matchedLinesIndex.reserve(linesTobeMatched.size() * 4);
	std::vector<std::vector<float>> matchedLinesDistance;
	matchedLinesDistance.reserve(linesTobeMatched.size() * 4);
	int totalQueryDescriptors = 0;
	for (size_t k = 0; k < linesTobeMatched.size(); k++)
	{
		size_t i = linesTobeMatched[k].first;
		size_t j = linesTobeMatched[k].second;

		//find matches
		INTERSECTION_LINE &line1 = currentIntersectionLines[i];
		INTERSECTION_LINE &line2 = currentIntersectionLines[j];
		PAIRLINE pairLine;
		pairLine.linePoints1 = currentLineLength[i][j].points1;
		pairLine.linePoints2 = currentLineLength[i][j].points2;
		pairLine.originalIndex1 = static_cast<int>(i);
		pairLine.originalIndex2 = static_cast<int>(j);
		std::vector<int> neighbor;
		std::vector<float> neighborDistance;
		if (line1.isIntersectionLine)
		{
			if (line2.isIntersectionLine)//22
			{
				int l1sp1 = line1.supportPlanes[0];
				int l1sp2 = line1.supportPlanes[1];
				int l2sp1 = line2.supportPlanes[0];
				int l2sp2 = line2.supportPlanes[1];

				//22
				Eigen::VectorXf *p22 = new Eigen::VectorXf(8);
				ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp1], suppotPlanes[l1sp2],
					suppotPlanes[l2sp1], suppotPlanes[l2sp2], *p22,
					pairLine.lineVec1, pairLine.lineVec2, method22);
				(*p22)[0] = currentLineLength[i][j].length;
				kdtree22.find_neighbors(*p22, maxNeighbor, maxNeighborDistance, neighbor, neighborDistance);
				if (!neighbor.empty())
				{
					pairLine.descriptor = *p22;
					query22.push_back(QueryResult(pairLine, neighbor, neighborDistance));
				}
				delete p22;

				//22-21 search 21
				Eigen::VectorXf *p2221_1 = new Eigen::VectorXf(6);
				ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp1], suppotPlanes[l1sp2],
					suppotPlanes[l2sp1], line2.lineVec.cross(suppotPlanes[l2sp1]), *p2221_1,
					pairLine.lineVec1, pairLine.lineVec2, method21);
				(*p2221_1)[0] = currentLineLength[i][j].length;
				kdtree21.find_neighbors(*p2221_1, maxNeighbor, maxNeighborDistance, neighbor, neighborDistance);
				if (!neighbor.empty())
				{
					pairLine.descriptor = *p2221_1;
					query21.push_back(QueryResult(pairLine, neighbor, neighborDistance));
				}
				delete p2221_1;

				Eigen::VectorXf *p2221_2 = new Eigen::VectorXf(6);
				ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp1], suppotPlanes[l1sp2],
					suppotPlanes[l2sp2], line2.lineVec.cross(suppotPlanes[l2sp2]), *p2221_2,
					pairLine.lineVec1, pairLine.lineVec2, method21);
				(*p2221_2)[0] = currentLineLength[i][j].length;
				kdtree21.find_neighbors(*p2221_2, maxNeighbor, maxNeighborDistance, neighbor, neighborDistance);
				if (!neighbor.empty())
				{
					pairLine.descriptor = *p2221_2;
					query21.push_back(QueryResult(pairLine, neighbor, neighborDistance));
				}
				delete p2221_2;

				//22-12 search 12
				Eigen::VectorXf *p2212_1 = new Eigen::VectorXf(6);
				ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp1], line1.lineVec.cross(suppotPlanes[l1sp1]),
					suppotPlanes[l2sp1], suppotPlanes[l2sp2], *p2212_1,
					pairLine.lineVec1, pairLine.lineVec2, method12);
				(*p2212_1)[0] = currentLineLength[i][j].length;
				kdtree12.find_neighbors(*p2212_1, maxNeighbor, maxNeighborDistance, neighbor, neighborDistance);
				if (!neighbor.empty())
				{
					pairLine.descriptor = *p2212_1;
					query12.push_back(QueryResult(pairLine, neighbor, neighborDistance));
				}

				delete p2212_1;

				Eigen::VectorXf *p2212_2 = new Eigen::VectorXf(6);
				ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp2], line1.lineVec.cross(suppotPlanes[l1sp2]),
					suppotPlanes[l2sp1], suppotPlanes[l2sp2], *p2212_2,
					pairLine.lineVec1, pairLine.lineVec2, method12);
				(*p2212_2)[0] = currentLineLength[i][j].length;
				kdtree12.find_neighbors(*p2212_2, maxNeighbor, maxNeighborDistance, neighbor, neighborDistance);
				if (!neighbor.empty())
				{
					pairLine.descriptor = *p2212_2;
					query12.push_back(QueryResult(pairLine, neighbor, neighborDistance));
				}

				delete p2212_2;
				totalQueryDescriptors += 5;

			}
			else//21
			{
				int l1sp1 = line1.supportPlanes[0];
				int l1sp2 = line1.supportPlanes[1];
				int l2sp1 = line2.supportPlanes[0];
				//21 search 21
				Eigen::VectorXf *p21 = new Eigen::VectorXf(6);
				ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp1], suppotPlanes[l1sp2],
					suppotPlanes[l2sp1], line2.lineVec.cross(suppotPlanes[l2sp1]), *p21,
					pairLine.lineVec1, pairLine.lineVec2, method21);
				(*p21)[0] = currentLineLength[i][j].length;
				kdtree21.find_neighbors(*p21, maxNeighbor, maxNeighborDistance, neighbor, neighborDistance);
				if (!neighbor.empty())
				{
					pairLine.descriptor = *p21;
					query21.push_back(QueryResult(pairLine, neighbor, neighborDistance));
				}
				//search 22-21
				kdtree2221.find_neighbors(*p21, maxNeighbor, maxNeighborDistance, neighbor, neighborDistance);
				if (!neighbor.empty())
				{
					pairLine.descriptor = *p21;
					query2221.push_back(QueryResult(pairLine, neighbor, neighborDistance));
				}

				delete p21;
				totalQueryDescriptors += 2;
			}
		}
		else
		{
			if (line2.isIntersectionLine)//12
			{
				int l1sp1 = line1.supportPlanes[0];
				int l2sp1 = line2.supportPlanes[0];
				int l2sp2 = line2.supportPlanes[1];
				//12 
				Eigen::VectorXf *p12 = new Eigen::VectorXf(6);
				ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp1], line1.lineVec.cross(suppotPlanes[l1sp1]),
					suppotPlanes[l2sp1], suppotPlanes[l2sp2], *p12,
					pairLine.lineVec1, pairLine.lineVec2, method12);
				(*p12)[0] = currentLineLength[i][j].length;
				kdtree12.find_neighbors(*p12, maxNeighbor, maxNeighborDistance, neighbor, neighborDistance);
				if (!neighbor.empty())
				{
					pairLine.descriptor = *p12;
					query12.push_back(QueryResult(pairLine, neighbor, neighborDistance));
				}

				//search 22-12
				kdtree2212.find_neighbors(*p12, maxNeighbor, maxNeighborDistance, neighbor, neighborDistance);
				if (!neighbor.empty())
				{
					pairLine.descriptor = *p12;
					query2212.push_back(QueryResult(pairLine, neighbor, neighborDistance));
				}

				delete p12;
				totalQueryDescriptors += 2;
			}

		}
	}

	//cout<<"totalQueryDescriptors:"<<totalQueryDescriptors<<endl;

	Eigen::Matrix3f R;
	Eigen::Vector3f T;
	std::vector<Eigen::Matrix3f>initialRs;
	std::vector<Eigen::Vector3f>initialTs;

	for (size_t i = 0; i < queryResult.size(); i++)
	{
		//cout<<displayInfor[i]<<endl;
		std::vector<QueryResult> &currentQuery = queryResult[i];
		std::vector<PAIRLINE> &currentMatched = mainLinesInformation[i];
		size_t currentQuerySize = currentQuery.size();
		for (size_t j = 0; j < currentQuerySize; j++)
		{
			QueryResult &res = currentQuery[j];
			size_t neighborNum = res.neighbor.size();
			for (size_t k = 0; k < neighborNum; k++)
			{
				// compute the transformation
				ComputeTransformationUsingTwoVecAndOnePoint(res.queryLine.lineVec1,
					res.queryLine.lineVec2,
					currentMatched[res.neighbor[k]].lineVec1,
					currentMatched[res.neighbor[k]].lineVec2,
					res.queryLine.linePoints1,
					currentMatched[res.neighbor[k]].linePoints1,
					R, T);
				initialRs.push_back(R);
				initialTs.push_back(T);
			}
		}
	}

	//cout<<"initial matches:"<<initialRs.size()<<endl;
	pcl::IndicesClusters cluster;
	ClusterTransformation(initialRs, initialTs, parameter.lengthThreshold / 2, parameter.angleThreshold / 2, cluster);
	//cout<<"total cluster Num:"<<cluster.size();
	std::vector<std::vector<std::pair<int, int>>>matches;//for each match,save the all matched planes index
	matches.reserve(cluster.size());
	std::vector<LENGTHINDEX>sortVec(cluster.size());
	int clusterNumOfGreaterThan1 = 0;
	for (size_t i = 0; i < sortVec.size(); i++)
	{
		sortVec[i].index = static_cast<int>(i);
		sortVec[i].length = cluster[i].indices.size();
		if (sortVec[i].length > 1)
		{
			clusterNumOfGreaterThan1++;
		}
	}
	//cout<<" greater than 1 cluster num: "<<clusterNumOfGreaterThan1<<endl;
	sort(sortVec.begin(), sortVec.end(), myCompareGreater);
	std::vector<Eigen::Matrix3f>Rs;
	Rs.reserve(cluster.size());
	std::vector<Eigen::Vector3f>Ts;
	Ts.reserve(cluster.size());
	for (size_t i = 0; i < sortVec.size(); i++)
	{
		int index = sortVec[i].index;
		int k = cluster[index].indices[0];
		R = initialRs[k];
		T = initialTs[k];
		// check 
		Eigen::Vector3f tc = R * currentBoundingCenter + T;
		if ((tc - mianBoundingCenter).norm() > parameter.maxRadius)//check whether point cloud is transformed to the location which is far away from destPointCloud
		{
			continue;
		}

		// count the number of consistent planes 
		std::vector<std::pair<int, int>> currentPlaneMatches;
		for (int i1 = 0; i1 < currentPlanesNum; i1++)
		{
			Eigen::Vector3f plane1 = R * planes[i1].block(0, 0, 3, 1);
			float d = -(-planes[i1](3) + (plane1.block(0, 0, 3, 1).transpose()*T)(0));

			//check center to plane distance
			Eigen::Vector3f srcCenter2Dest = R * currentBoundingBoxCenterForEachPlane[i1] + T;
			for (size_t j1 = 0; j1 < mainPlanesNum; j1++)
			{
				Eigen::Vector3f plane_A = mainPlanes[j1].block(0, 0, 3, 1);
				if (plane1.dot(plane_A) < cosAngleTh)
				{
					continue;
				}
				Eigen::Vector3f plane_B = plane1.block(0, 0, 3, 1);
				double center2PlaneDistance = (abs(plane_A.dot(srcCenter2Dest) + mainPlanes[j1](3))
					+ abs(plane_B.dot(mainBoundingBoxCenterForEachPlane[j1]) + d)) / 2;
				if (center2PlaneDistance > lengthThreshold)
				{
					continue;
				}
				double distance = (srcCenter2Dest - mainBoundingBoxCenterForEachPlane[j1]).norm();
				if (distance / (currentBoundingBoxRadiusForEachPlane[i1] + mainBoundingBoxRadiusForEachPlane[j1]) > 1)
				{
					//cout<<"reject one "<<endl;
					continue;
				}
				currentPlaneMatches.push_back(std::pair<int, int>(i1, j1));
				break;
			}
		}
		matches.push_back(currentPlaneMatches);
		Rs.push_back(R);
		Ts.push_back(T);
	}

	// find the correct matches
	size_t maxMatchNum = 0;
	for (size_t i = 0; i < matches.size(); i++)
	{
		//cout<<matches[i].size()<<endl;
		if (maxMatchNum < matches[i].size())
		{
			maxMatchNum = matches[i].size();
		}
	}
	std::vector<std::vector<int>> matchedPlanes;
	matchedPlanes.reserve(parameter.maxCandidateResultNum * 2);
	if (maxMatchNum > 0)
	{
		//cout<<"total planes:"<<planes.size()<<endl;
		///////////////////////////////////////////////////////
		int matchedCount = 0;
		for (size_t i = maxMatchNum; i >= 2; i--)
		{
			std::vector<int> tempMatchs;
			for (size_t j = 0; j < matches.size(); j++)
			{
				if (i == matches[j].size())
				{
					tempMatchs.push_back(static_cast<int>(j));
					matchedCount++;
				}
			}
			matchedPlanes.push_back(tempMatchs);
			//cout<<i<<"matched planes "<<tempMatchs.size()<<endl;
			if (matchedCount >= parameter.maxCandidateResultNum)
			{
				break;
			}
		}
		//cout<<"matchedCount:"<<matchedCount<<endl;
		//cout<<"total "<<matchedPlanes.size()<<endl;
	}
	else
	{
		//cout<<"can not be matched"<<endl;
		return;
	}

    std::cout << "matching lines..." << std::endl;
    StopWatch w;
	int count = 0;
	for (size_t m = 0; m < matchedPlanes.size(); m++)
	{
		for (size_t i = 0; i < matchedPlanes[m].size(); i++)
		{
            print_progress(float(count)/parameter.maxCandidateResultNum);
			if (count++ > parameter.maxCandidateResultNum)
			{
                std::cout << std::endl << "done. time: " << w.time_string() << std::endl;
				return;
			}
			int index = matchedPlanes[m][i];
			pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation;
			transformation = pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4::Identity();
			transformation.block(0, 0, 3, 3) = Rs[index];
			transformation.block(0, 3, 3, 1) = Ts[index];
			//check penetration
			bool isPentrable = false;
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tempKdtree(new pcl::search::KdTree<pcl::PointXYZ>);
			for (int i1 = 0; i1 < currentPlanesNum; i1++)
			{
				isPentrable = false;
				Eigen::Vector4f plane1;
				plane1.block(0, 0, 3, 1) = Rs[index] * planes[i1].block(0, 0, 3, 1);
				plane1(3) = -(-planes[i1](3) + (plane1.block(0, 0, 3, 1).transpose()*Ts[index])(0));
				pcl::PointCloud<pcl::PointXYZ>::Ptr tempTransPoints(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::transformPointCloud(*downSampleCurrentPlanePoints[i1], *tempTransPoints, transformation);
				pcl::PointCloud<pcl::PointXYZ> tempPclFourCorner;
				ExchnageBetweentPCLPointXYZwithEigenVector3f(tempPclFourCorner, currentBoundingBoxFourCornerPoints[i1]);
				pcl::transformPointCloud(tempPclFourCorner, tempPclFourCorner, transformation);
				std::vector<Eigen::Vector3f>tempEigenFourCorner;
				ExchnageBetweentPCLPointXYZwithEigenVector3f(tempPclFourCorner, tempEigenFourCorner);
				tempKdtree->setInputCloud(tempTransPoints);
				Eigen::Vector3f currentCenter2Main = Rs[index] * currentBoundingBoxCenterForEachPlane[i1] + Ts[index];
				for (size_t j1 = 0; j1 < mainPlanesNum; j1++)
				{
					Eigen::Vector3f plane_A = mainPlanes[j1].block(0, 0, 3, 1);
					Eigen::Vector3f plane_B = plane1.block(0, 0, 3, 1);
					double center2PlaneDistance = (abs(plane_A.dot(currentCenter2Main) + mainPlanes[j1](3))
						+ abs(plane_B.dot(mainBoundingBoxCenterForEachPlane[j1]) + plane1(3))) / 2;
					if (center2PlaneDistance<lengthThreshold&&plane_B.dot(plane_A)>angleThreshold)
					{
						continue;
					}
					if (0 != AreTwoPlanesPenetrable(plane1, mainPlanes[j1], tempEigenFourCorner,
						mainBoundingBoxFourCornerPoints[j1], tempKdtree, mainDownSampleKdTree[j1], isPentrable, parameter.lengthThreshold, 10, parameter.lengthThreshold / 2))
					{
						continue;
					}
					if (isPentrable)
					{
						break;
					}
				}
				if (isPentrable)
				{
					break;
				}
			}
			if (isPentrable)
			{
				continue;
			}

			MatchedResult result;
            result.matchedPlanes = matches[index];
            result.R = Rs[index];
            result.T = Ts[index];
			pMatchedResult.push_back(result);
		}
    }
}


static const char* const PATH_SEPARATORS = "/\\";

std::string extension(const std::string& file_name) {
    std::string::size_type dot = file_name.find_last_of('.');
    std::string::size_type slash = file_name.find_last_of(PATH_SEPARATORS);
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash))
        return std::string("");
    return std::string(file_name.begin() + dot + 1, file_name.end());
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
                                        METHODINDEX methodIndex) {
    //determine the direction if line
    //line1
    float angle1 = abs(line1.lineVec.dot(line2sp1));
    float angle2 = abs(line1.lineVec.dot(line2sp2));
    Eigen::Vector3f newline2sp1, newline2sp2;
    if (angle1 <= angle2) {
        newline2sp1 = line2sp1;
        newline2sp2 = line2sp2;
    } else {
        newline2sp1 = line2sp2;
        newline2sp2 = line2sp1;
    }
    newLine2 = newline2sp1.cross(newline2sp2);
    //line2
    Eigen::Vector3f newline1sp1, newline1sp2;
    angle1 = abs(line2.lineVec.dot(line1sp1));
    angle2 = abs(line2.lineVec.dot(line1sp2));
    if (angle1 <= angle2) {
        newline1sp1 = line1sp1;
        newline1sp2 = line1sp2;
    } else {
        newline1sp1 = line1sp2;
        newline1sp2 = line1sp1;
    }
    newLine1 = newline1sp1.cross(newline1sp2);
    if (method22 == methodIndex) {
        // construct descriptor vector
        //(*p)[0]=lineLength[i][j].length;//distance of two lines
        descriptor[1] = newLine1.dot(newLine2);//angle of two lines
        descriptor[2] = newline1sp1.dot(newline1sp2);//angle of support planes of line1
        descriptor[3] = newline2sp1.dot(newline2sp2);//angle of support planes of line2
        descriptor[4] = newLine1.dot(newline2sp1);//angle of line1 with support planes of line2
        descriptor[5] = newLine1.dot(newline2sp2);//angle of line1 with support planes of line2
        descriptor[6] = newLine2.dot(newline1sp1);//angle of line2 with support planes of line1
        descriptor[7] = newLine2.dot(newline1sp2);//angle of line2 with support planes of line1
    } else if (method21 == methodIndex) {
        // construct descriptor vector
        //(*p)[0]=lineLength[i][j].length;//distance of two lines
        descriptor[1] = newLine1.dot(newLine2);//angle of two lines
        descriptor[2] = newline1sp1.dot(newline1sp2);//angle of support planes of line1
        descriptor[3] = newLine1.dot(line2sp1);//angle of line1 with support planes of line2
        descriptor[4] = newLine2.dot(newline1sp1);//angle of line2 with support planes of line1
        descriptor[5] = newLine2.dot(newline1sp2);//angle of line2 with support planes of line1
    } else if (method12 == methodIndex) {
        // construct descriptor vector
        //(*p)[0]=lineLength[i][j].length;//distance of two lines
        descriptor[1] = newLine1.dot(newLine2);//angle of two lines
        descriptor[2] = newline2sp1.dot(newline2sp2);//angle of support planes of line2
        descriptor[3] = newLine1.dot(newline2sp1);//angle of line1 with support planes of line2
        descriptor[4] = newLine1.dot(newline2sp2);//angle of line1 with support planes of line2
        descriptor[5] = newLine2.dot(line1sp1);//angle of line2 with support planes of line1
    } else if (method11 == methodIndex) {
        // construct descriptor vector
        //(*p)[0]=lineLength[i][j].length;//distance of two lines
        descriptor[1] = newLine1.dot(newLine2);//angle of two lines
        descriptor[2] = newLine1.dot(line2sp1);//angle of line1 with support planes of line2
        descriptor[3] = newLine2.dot(line1sp1);//angle of line2 with support planes of line1
    }
    return 0;
}

int ComputeTransformationUsingTwoVecAndOnePoint(Eigen::Vector3f &sourceVec1, Eigen::Vector3f &sourceVec2,
                                                Eigen::Vector3f &destVec1, Eigen::Vector3f &destVec2,
                                                Eigen::Vector3f &sourcePoint, Eigen::Vector3f &targetPoint,
                                                Eigen::Matrix3f &R, Eigen::Vector3f &T) {
    //����SVD�������任����
    pcl::PointCloud<pcl::PointXYZ> src, dst;
    Eigen::Vector3f temp1 = sourceVec1.cross(sourceVec2);
    Eigen::Vector3f temp2 = destVec1.cross(destVec2);
    src.push_back(pcl::PointXYZ(sourceVec1(0), sourceVec1(1), sourceVec1(2)));
    src.push_back(pcl::PointXYZ(sourceVec2(0), sourceVec2(1), sourceVec2(2)));
    src.push_back(pcl::PointXYZ(temp1(0), temp1(1), temp1(2)));
    dst.push_back(pcl::PointXYZ(destVec1(0), destVec1(1), destVec1(2)));
    dst.push_back(pcl::PointXYZ(destVec2(0), destVec2(1), destVec2(2)));
    dst.push_back(pcl::PointXYZ(temp2(0), temp2(1), temp2(2)));
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> TESVD;
    TESVD.estimateRigidTransformation(src, dst, transformation);
    R = transformation.block(0, 0, 3, 3);
    T = targetPoint - R * sourcePoint;

    return 0;
}

int ComputeIntersectionLineOfTwoPlanes(const Eigen::Vector4f &plane1,
                                       const Eigen::Vector4f &plane2,
                                       Eigen::Vector3f &lineVec,
                                       Eigen::Vector3f &linePoint) {
    Eigen::Vector3f p1(plane1(0), plane1(1), plane1(2));
    Eigen::Vector3f p2(plane2(0), plane2(1), plane2(2));
    p1.normalize();
    p2.normalize();
    if (abs(p1.dot(p2)) > 0.95) {
        return -1;
    }
    lineVec = p1.cross(p2);
    lineVec.normalize();
    cv::Mat A(2, 2, CV_64F, cv::Scalar(0));
    cv::Mat B(2, 1, CV_64F, cv::Scalar(0));
    B.at<double>(0) = -plane1(3);
    B.at<double>(1) = -plane2(3);
    cv::Mat result;
    if (abs(plane1(0) * plane2(1) - plane2(0) * plane1(1)) > 1e-6) {
        A.at<double>(0, 0) = plane1(0);
        A.at<double>(0, 1) = plane1(1);
        A.at<double>(1, 0) = plane2(0);
        A.at<double>(1, 1) = plane2(1);
        result = A.inv() * B;
        linePoint(0) = result.at<double>(0);
        linePoint(1) = result.at<double>(1);
        linePoint(2) = 0;
    } else if (abs(plane1(0) * plane2(2) - plane2(0) * plane1(2)) > 1e-6) {
        A.at<double>(0, 0) = plane1(0);
        A.at<double>(0, 1) = plane1(2);
        A.at<double>(1, 0) = plane2(0);
        A.at<double>(1, 1) = plane2(2);
        result = A.inv() * B;
        linePoint(0) = result.at<double>(0);
        linePoint(2) = result.at<double>(1);
        linePoint(1) = 0;

    } else if (abs(plane1(1) * plane2(2) - plane2(1) * plane1(2)) > 1e-6) {
        A.at<double>(0, 0) = plane1(1);
        A.at<double>(0, 1) = plane1(2);
        A.at<double>(1, 0) = plane2(1);
        A.at<double>(1, 1) = plane2(2);
        result = A.inv() * B;
        linePoint(1) = result.at<double>(0);
        linePoint(2) = result.at<double>(1);
        linePoint(0) = 0;
    } else {
        return -1;
    }
    return 0;
}


int Fit3DLine(std::vector<cv::Point3f> &points, cv::Vec6f &param, float *pAccuracy/*=NULL*/) {
    if (points.size() < 3) {
        return -1;
    }
    //////////////////////////////////////////////////////////////////////////
    cv::fitLine(points, param, CV_DIST_L2, 0, 0.01, 0.01);
    //�ж�ֱ����Ͻ��
    if (param[0] == 0 && param[1] == 0 && param[2] == 0) {
        return -1;
    }
    if (pAccuracy)//�����Ҫ������Ͼ���
    {
        //����㵽ֱ�ߵľ���
        size_t pointsNum = points.size();
        double sum = 0;
        for (size_t i = 0; i < pointsNum; i++) {
            sum += computeLengthOfPoint23DLine(param, points[i]);
        }
        *pAccuracy = float(sum / pointsNum);
    }
    return 0;

}


//kdtrees8 22
//kdtrees6 12 22-12 21 22-21
//kdtrees4 11 12-11 21-11 22-11
//linesInformation 22 12 22-12 21 22-21 11 12-11 21-11 22-11
int ConstructPairLinesKdTree(std::vector<INTERSECTION_LINE> &lines,
                             std::vector<Eigen::Vector4f> &planes,
                             std::vector<KdTreeSearchNDim<Eigen::VectorXf, 8>> &kdtrees8,
                             std::vector<KdTreeSearchNDim<Eigen::VectorXf, 6>> &kdtrees6,
                             std::vector<KdTreeSearchNDim<Eigen::VectorXf, 4>> &kdtrees4,
                             std::vector<std::vector<PAIRLINE>> &linesInformation,
                             float scalar) {
    if (lines.empty() || planes.empty()) {
        return -1;
    }
    size_t linesNum = lines.size();
    size_t planeNum = planes.size();
    std::vector<Eigen::Vector3f> suppotPlanes(planeNum);
    for (size_t i = 0; i < planeNum; i++) {
        suppotPlanes[i] = planes[i].block(0, 0, 3, 1);
    }
    kdtrees8.resize(1);
    kdtrees6.resize(4);
    kdtrees4.resize(4);
    KdTreeSearchNDim<Eigen::VectorXf, 8> &kdtree22 = kdtrees8[0];
    kdtree22.begin();
    KdTreeSearchNDim<Eigen::VectorXf, 6> &kdtree12 = kdtrees6[0];
    kdtree12.begin();
    KdTreeSearchNDim<Eigen::VectorXf, 6> &kdtree2212 = kdtrees6[1];
    kdtree2212.begin();
    KdTreeSearchNDim<Eigen::VectorXf, 6> &kdtree21 = kdtrees6[2];
    kdtree21.begin();
    KdTreeSearchNDim<Eigen::VectorXf, 6> &kdtree2221 = kdtrees6[3];
    kdtree2221.begin();
    KdTreeSearchNDim<Eigen::VectorXf, 4> &kdtree11 = kdtrees4[0];
    kdtree11.begin();
    KdTreeSearchNDim<Eigen::VectorXf, 4> &kdtree1211 = kdtrees4[1];
    kdtree1211.begin();
    KdTreeSearchNDim<Eigen::VectorXf, 4> &kdtree2111 = kdtrees4[2];
    kdtree2111.begin();
    KdTreeSearchNDim<Eigen::VectorXf, 4> &kdtree2211 = kdtrees4[3];
    kdtree2211.begin();
    std::vector<Eigen::VectorXf *> vec22;
    std::vector<Eigen::VectorXf *> vec12;
    std::vector<Eigen::VectorXf *> vec2212;
    std::vector<Eigen::VectorXf *> vec21;
    std::vector<Eigen::VectorXf *> vec2221;
    std::vector<Eigen::VectorXf *> vec11;
    std::vector<Eigen::VectorXf *> vec1211;
    std::vector<Eigen::VectorXf *> vec2111;
    std::vector<Eigen::VectorXf *> vec2211;
    linesInformation.resize(9);
    std::vector<PAIRLINE> &lf22 = linesInformation[0];
    std::vector<PAIRLINE> &lf12 = linesInformation[1];
    std::vector<PAIRLINE> &lf2212 = linesInformation[2];
    std::vector<PAIRLINE> &lf21 = linesInformation[3];
    std::vector<PAIRLINE> &lf2221 = linesInformation[4];
    std::vector<PAIRLINE> &lf11 = linesInformation[5];
    std::vector<PAIRLINE> &lf1211 = linesInformation[6];
    std::vector<PAIRLINE> &lf2111 = linesInformation[7];
    std::vector<PAIRLINE> &lf2211 = linesInformation[8];

    struct NEARST_POINTS_TWOLINE {
        Eigen::Vector3f points1;
        Eigen::Vector3f points2;
        double length;
    };

    StopWatch w;
    std::cout << "generating descriptors..." << std::endl;
    std::vector<std::vector<NEARST_POINTS_TWOLINE>> lineLength(linesNum);
    float minLength = 0.1 / scalar;
    float angleThresh = cos(10.0 / 180 * M_PI);
    for (size_t i = 0; i < linesNum; i++) {
        //std::cout <<i<<std::endl;
        lineLength[i].resize(linesNum);
        INTERSECTION_LINE &line1 = lines[i];
        for (size_t j = 0; j < linesNum; j++) {
            if (i == j) {
                continue;
            }
            INTERSECTION_LINE &line2 = lines[j];
            //compute length of each pair of lines
            if (i > j) {
                lineLength[i][j].points1 = lineLength[j][i].points2;
                lineLength[i][j].points2 = lineLength[j][i].points1;
                lineLength[i][j].length = lineLength[j][i].length;
            } else if (i < j) {
                if (0 != ComputeNearstTwoPointsOfTwo3DLine(line1.lineVec, line1.linePoint,
                                                                line2.lineVec, line2.linePoint,
                                                                lineLength[i][j].points1,
                                                                lineLength[i][j].points2,
                                                                lineLength[i][j].length)) {
                    lineLength[i][j].length = -1;
                }
                lineLength[i][j].length = lineLength[i][j].length / scalar;
            }
            //discard some lines
            if (abs(line1.lineVec.dot(line2.lineVec)) > angleThresh) {
                continue;
            }
            PAIRLINE pairLine;
            pairLine.linePoints1 = lineLength[i][j].points1;
            pairLine.linePoints2 = lineLength[i][j].points2;
            pairLine.originalIndex1 = static_cast<int>(i);
            pairLine.originalIndex2 = static_cast<int>(j);
            //construct kdTree
            if (line1.isIntersectionLine) {
                if (line2.isIntersectionLine)//22
                {
                    int l1sp1 = line1.supportPlanes[0];
                    int l1sp2 = line1.supportPlanes[1];
                    int l2sp1 = line2.supportPlanes[0];
                    int l2sp2 = line2.supportPlanes[1];

                    //22
                    Eigen::VectorXf *p22 = new Eigen::VectorXf(8);
                    ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp1], suppotPlanes[l1sp2],
                                                        suppotPlanes[l2sp1], suppotPlanes[l2sp2], *p22,
                                                        pairLine.lineVec1, pairLine.lineVec2, method22);

                    (*p22)[0] = lineLength[i][j].length;
                    pairLine.descriptor = *p22;
                    lf22.push_back(pairLine);
                    vec22.push_back(p22);
                    kdtree22.add_point(p22);


                    //22-21
                    Eigen::VectorXf *p2221_1 = new Eigen::VectorXf(6);
                    ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp1], suppotPlanes[l1sp2],
                                                        suppotPlanes[l2sp1],
                                                        line2.lineVec.cross(suppotPlanes[l2sp1]), *p2221_1,
                                                        pairLine.lineVec1, pairLine.lineVec2, method21);
                    (*p2221_1)[0] = lineLength[i][j].length;
                    pairLine.descriptor = *p2221_1;
                    lf2221.push_back(pairLine);

                    Eigen::VectorXf *p2221_2 = new Eigen::VectorXf(6);
                    ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp1], suppotPlanes[l1sp2],
                                                        suppotPlanes[l2sp2],
                                                        line2.lineVec.cross(suppotPlanes[l2sp2]), *p2221_2,
                                                        pairLine.lineVec1, pairLine.lineVec2, method21);
                    (*p2221_2)[0] = lineLength[i][j].length;
                    pairLine.descriptor = *p2221_2;
                    lf2221.push_back(pairLine);

                    Eigen::VectorXf *p2221_3 = new Eigen::VectorXf(6);
                    ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp1], suppotPlanes[l1sp2],
                                                        suppotPlanes[l2sp1],
                                                        line2.lineVec.cross(-suppotPlanes[l2sp1]), *p2221_3,
                                                        pairLine.lineVec1, pairLine.lineVec2, method21);
                    (*p2221_3)[0] = lineLength[i][j].length;
                    pairLine.descriptor = *p2221_3;
                    lf2221.push_back(pairLine);

                    Eigen::VectorXf *p2221_4 = new Eigen::VectorXf(6);
                    ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp1], suppotPlanes[l1sp2],
                                                        suppotPlanes[l2sp2],
                                                        line2.lineVec.cross(-suppotPlanes[l2sp2]), *p2221_4,
                                                        pairLine.lineVec1, pairLine.lineVec2, method21);
                    (*p2221_4)[0] = lineLength[i][j].length;
                    pairLine.descriptor = *p2221_4;
                    lf2221.push_back(pairLine);

                    vec2221.push_back(p2221_1);
                    vec2221.push_back(p2221_2);
                    vec2221.push_back(p2221_3);
                    vec2221.push_back(p2221_4);
                    kdtree2221.add_point(p2221_1);
                    kdtree2221.add_point(p2221_2);
                    kdtree2221.add_point(p2221_3);
                    kdtree2221.add_point(p2221_4);

                    //22-12
                    Eigen::VectorXf *p2212_1 = new Eigen::VectorXf(6);
                    ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp1],
                                                        line1.lineVec.cross(suppotPlanes[l1sp1]),
                                                        suppotPlanes[l2sp1], suppotPlanes[l2sp2], *p2212_1,
                                                        pairLine.lineVec1, pairLine.lineVec2, method12);
                    (*p2212_1)[0] = lineLength[i][j].length;
                    pairLine.descriptor = *p2212_1;
                    lf2212.push_back(pairLine);

                    Eigen::VectorXf *p2212_2 = new Eigen::VectorXf(6);
                    ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp2],
                                                        line1.lineVec.cross(suppotPlanes[l1sp2]),
                                                        suppotPlanes[l2sp1], suppotPlanes[l2sp2], *p2212_2,
                                                        pairLine.lineVec1, pairLine.lineVec2, method12);
                    (*p2212_2)[0] = lineLength[i][j].length;
                    pairLine.descriptor = *p2212_2;
                    lf2212.push_back(pairLine);

                    Eigen::VectorXf *p2212_3 = new Eigen::VectorXf(6);
                    ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp1],
                                                        line1.lineVec.cross(-suppotPlanes[l1sp1]),
                                                        suppotPlanes[l2sp1], suppotPlanes[l2sp2], *p2212_3,
                                                        pairLine.lineVec1, pairLine.lineVec2, method12);
                    (*p2212_3)[0] = lineLength[i][j].length;
                    pairLine.descriptor = *p2212_3;
                    lf2212.push_back(pairLine);

                    Eigen::VectorXf *p2212_4 = new Eigen::VectorXf(6);
                    ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp2],
                                                        line1.lineVec.cross(-suppotPlanes[l1sp2]),
                                                        suppotPlanes[l2sp1], suppotPlanes[l2sp2], *p2212_4,
                                                        pairLine.lineVec1, pairLine.lineVec2, method12);
                    (*p2212_4)[0] = lineLength[i][j].length;
                    pairLine.descriptor = *p2212_4;
                    lf2212.push_back(pairLine);

                    vec2212.push_back(p2212_1);
                    vec2212.push_back(p2212_2);
                    vec2212.push_back(p2212_3);
                    vec2212.push_back(p2212_4);
                    kdtree2212.add_point(p2212_1);
                    kdtree2212.add_point(p2212_2);
                    kdtree2212.add_point(p2212_3);
                    kdtree2212.add_point(p2212_4);

// 					//22-11
// 					Eigen::VectorXf *p2211_1=new Eigen::VectorXf(6);
// 					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],line1.lineVec.cross(suppotPlanes[l1sp1]),
// 						suppotPlanes[l2sp1],line2.lineVec.cross(suppotPlanes[l2sp1]),*p2211_1,
// 						pairLine.lineVec1,pairLine.lineVec2,method11);
// 					lf2211.push_back(pairLine);
//
// 					Eigen::VectorXf *p2211_2=new Eigen::VectorXf(6);
// 					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],line1.lineVec.cross(suppotPlanes[l1sp1]),
// 						suppotPlanes[l2sp2],line2.lineVec.cross(suppotPlanes[l2sp2]),*p2211_2,
// 						pairLine.lineVec1,pairLine.lineVec2,method11);
// 					lf2211.push_back(pairLine);
//
// 					Eigen::VectorXf *p2211_3=new Eigen::VectorXf(6);
// 					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp2],line1.lineVec.cross(suppotPlanes[l1sp2]),
// 						suppotPlanes[l2sp1],line2.lineVec.cross(suppotPlanes[l2sp1]),*p2211_3,
// 						pairLine.lineVec1,pairLine.lineVec2,method11);
// 					lf2211.push_back(pairLine);
//
// 					Eigen::VectorXf *p2211_4=new Eigen::VectorXf(6);
// 					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp2],line1.lineVec.cross(suppotPlanes[l1sp2]),
// 						suppotPlanes[l2sp2],line2.lineVec.cross(suppotPlanes[l2sp2]),*p2211_4,
// 						pairLine.lineVec1,pairLine.lineVec2,method11);
// 					lf2211.push_back(pairLine);
//
// 					Eigen::VectorXf *p2211_5=new Eigen::VectorXf(6);
// 					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],line1.lineVec.cross(-suppotPlanes[l1sp1]),
// 						suppotPlanes[l2sp1],line2.lineVec.cross(-suppotPlanes[l2sp1]),*p2211_5,
// 						pairLine.lineVec1,pairLine.lineVec2,method11);
// 					lf2211.push_back(pairLine);
//
// 					Eigen::VectorXf *p2211_6=new Eigen::VectorXf(6);
// 					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],line1.lineVec.cross(-suppotPlanes[l1sp1]),
// 						suppotPlanes[l2sp2],line2.lineVec.cross(-suppotPlanes[l2sp2]),*p2211_6,
// 						pairLine.lineVec1,pairLine.lineVec2,method11);
// 					lf2211.push_back(pairLine);
//
// 					Eigen::VectorXf *p2211_7=new Eigen::VectorXf(6);
// 					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp2],line1.lineVec.cross(-suppotPlanes[l1sp2]),
// 						suppotPlanes[l2sp1],line2.lineVec.cross(-suppotPlanes[l2sp1]),*p2211_7,
// 						pairLine.lineVec1,pairLine.lineVec2,method11);
// 					lf2211.push_back(pairLine);
//
// 					Eigen::VectorXf *p2211_8=new Eigen::VectorXf(6);
// 					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp2],line1.lineVec.cross(-suppotPlanes[l1sp2]),
// 						suppotPlanes[l2sp2],line2.lineVec.cross(-suppotPlanes[l2sp2]),*p2211_8,
// 						pairLine.lineVec1,pairLine.lineVec2,method11);
// 					lf2211.push_back(pairLine);
//
// 					(*p2211_1)[0]=lineLength[i][j].length;
// 					(*p2211_2)[0]=lineLength[i][j].length;
// 					(*p2211_3)[0]=lineLength[i][j].length;
// 					(*p2211_4)[0]=lineLength[i][j].length;
// 					(*p2211_5)[0]=lineLength[i][j].length;
// 					(*p2211_6)[0]=lineLength[i][j].length;
// 					(*p2211_7)[0]=lineLength[i][j].length;
// 					(*p2211_8)[0]=lineLength[i][j].length;
// 					vec2211.push_back(p2211_1);
// 					vec2211.push_back(p2211_2);
// 					vec2211.push_back(p2211_3);
// 					vec2211.push_back(p2211_4);
// 					vec2211.push_back(p2211_5);
// 					vec2211.push_back(p2211_6);
// 					vec2211.push_back(p2211_7);
// 					vec2211.push_back(p2211_8);
// 					kdtree2211.add_point(p2211_1);
// 					kdtree2211.add_point(p2211_2);
// 					kdtree2211.add_point(p2211_3);
// 					kdtree2211.add_point(p2211_4);
// 					kdtree2211.add_point(p2211_5);
// 					kdtree2211.add_point(p2211_6);
// 					kdtree2211.add_point(p2211_7);
// 					kdtree2211.add_point(p2211_8);
                } else//21
                {
                    int l1sp1 = line1.supportPlanes[0];
                    int l1sp2 = line1.supportPlanes[1];
                    int l2sp1 = line2.supportPlanes[0];
                    //21
                    Eigen::VectorXf *p21 = new Eigen::VectorXf(6);
                    ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp1], suppotPlanes[l1sp2],
                                                        suppotPlanes[l2sp1],
                                                        line2.lineVec.cross(suppotPlanes[l2sp1]), *p21,
                                                        pairLine.lineVec1, pairLine.lineVec2, method21);
                    (*p21)[0] = lineLength[i][j].length;
                    pairLine.descriptor = *p21;
                    lf21.push_back(pairLine);

                    Eigen::VectorXf *p21_1 = new Eigen::VectorXf(6);
                    ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp1], suppotPlanes[l1sp2],
                                                        suppotPlanes[l2sp1],
                                                        line2.lineVec.cross(-suppotPlanes[l2sp1]), *p21_1,
                                                        pairLine.lineVec1, pairLine.lineVec2, method21);
                    (*p21_1)[0] = lineLength[i][j].length;
                    pairLine.descriptor = *p21_1;
                    lf21.push_back(pairLine);

                    vec21.push_back(p21);
                    vec21.push_back(p21_1);
                    kdtree21.add_point(p21);
                    kdtree21.add_point(p21_1);


// 					//21-11
// 					Eigen::VectorXf *p2111_1=new Eigen::VectorXf(6);
// 					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],line1.lineVec.cross(suppotPlanes[l1sp1]),
// 						suppotPlanes[l2sp1],line2.lineVec.cross(suppotPlanes[l2sp1]),*p2111_1,
// 						pairLine.lineVec1,pairLine.lineVec2,method11);
// 					lf2111.push_back(pairLine);
//
// 					Eigen::VectorXf *p2111_2=new Eigen::VectorXf(6);
// 					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp2],line1.lineVec.cross(suppotPlanes[l1sp2]),
// 						suppotPlanes[l2sp1],line2.lineVec.cross(suppotPlanes[l2sp1]),*p2111_2,
// 						pairLine.lineVec1,pairLine.lineVec2,method11);
// 					lf2111.push_back(pairLine);
//
// 					(*p2111_1)[0]=lineLength[i][j].length;
// 					(*p2111_2)[0]=lineLength[i][j].length;
// 					vec2111.push_back(p2111_1);
// 					vec2111.push_back(p2111_2);
// 					kdtree2111.add_point(p2111_1);
// 					kdtree2111.add_point(p2111_2);
                }
            } else {
                if (line2.isIntersectionLine)//12
                {
                    int l1sp1 = line1.supportPlanes[0];
                    int l2sp1 = line2.supportPlanes[0];
                    int l2sp2 = line2.supportPlanes[1];
                    //12
                    Eigen::VectorXf *p12 = new Eigen::VectorXf(6);
                    ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp1],
                                                        line1.lineVec.cross(suppotPlanes[l1sp1]),
                                                        suppotPlanes[l2sp1], suppotPlanes[l2sp2], *p12,
                                                        pairLine.lineVec1, pairLine.lineVec2, method12);
                    (*p12)[0] = lineLength[i][j].length;
                    pairLine.descriptor = *p12;
                    lf12.push_back(pairLine);

                    Eigen::VectorXf *p12_1 = new Eigen::VectorXf(6);
                    ComputeDescriptorVectorForPairLines(line1, line2, suppotPlanes[l1sp1],
                                                        line1.lineVec.cross(-suppotPlanes[l1sp1]),
                                                        suppotPlanes[l2sp1], suppotPlanes[l2sp2], *p12_1,
                                                        pairLine.lineVec1, pairLine.lineVec2, method12);
                    (*p12_1)[0] = lineLength[i][j].length;
                    pairLine.descriptor = *p12_1;
                    lf12.push_back(pairLine);

                    vec12.push_back(p12);
                    vec12.push_back(p12_1);
                    kdtree12.add_point(p12);
                    kdtree12.add_point(p12_1);

// 					//12-11
// 					Eigen::VectorXf *p1211_1=new Eigen::VectorXf(6);
// 					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],line1.lineVec.cross(suppotPlanes[l1sp1]),
// 						suppotPlanes[l2sp1],line2.lineVec.cross(suppotPlanes[l2sp1]),*p1211_1,
// 						pairLine.lineVec1,pairLine.lineVec2,method11);
// 					lf1211.push_back(pairLine);
//
// 					Eigen::VectorXf *p1211_2=new Eigen::VectorXf(6);
// 					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],line1.lineVec.cross(suppotPlanes[l1sp1]),
// 						suppotPlanes[l2sp2],line2.lineVec.cross(suppotPlanes[l2sp2]),*p1211_2,
// 						pairLine.lineVec1,pairLine.lineVec2,method11);
// 					lf1211.push_back(pairLine);
//
// 					(*p1211_1)[0]=lineLength[i][j].length;
// 					(*p1211_2)[0]=lineLength[i][j].length;
// 					vec1211.push_back(p1211_1);
// 					vec1211.push_back(p1211_2);
// 					kdtree1211.add_point(p1211_1);
// 					kdtree1211.add_point(p1211_2);

                } else//11
                {
// 					int l1sp1=line1.supportPlanes[0];
// 					int l2sp1=line2.supportPlanes[0];
// 					Eigen::VectorXf *p11=new Eigen::VectorXf(6);
// 					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],line1.lineVec.cross(suppotPlanes[l1sp1]),
// 						suppotPlanes[l2sp1],line2.lineVec.cross(suppotPlanes[l2sp1]),*p11,
// 						pairLine.lineVec1,pairLine.lineVec2,method11);
// 					lf11.push_back(pairLine);
//
// 					Eigen::VectorXf *p11_1=new Eigen::VectorXf(6);
// 					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],line1.lineVec.cross(-suppotPlanes[l1sp1]),
// 						suppotPlanes[l2sp1],line2.lineVec.cross(suppotPlanes[l2sp1]),*p11_1,
// 						pairLine.lineVec1,pairLine.lineVec2,method11);
// 					lf11.push_back(pairLine);
//
// 					Eigen::VectorXf *p11_2=new Eigen::VectorXf(6);
// 					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],line1.lineVec.cross(suppotPlanes[l1sp1]),
// 						suppotPlanes[l2sp1],line2.lineVec.cross(-suppotPlanes[l2sp1]),*p11_2,
// 						pairLine.lineVec1,pairLine.lineVec2,method11);
// 					lf11.push_back(pairLine);
//
// 					Eigen::VectorXf *p11_3=new Eigen::VectorXf(6);
// 					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],line1.lineVec.cross(-suppotPlanes[l1sp1]),
// 						suppotPlanes[l2sp1],line2.lineVec.cross(-suppotPlanes[l2sp1]),*p11_3,
// 						pairLine.lineVec1,pairLine.lineVec2,method11);
// 					lf11.push_back(pairLine);
//
// 					(*p11)[0]=lineLength[i][j].length;
// 					(*p11_1)[0]=lineLength[i][j].length;
// 					(*p11_2)[0]=lineLength[i][j].length;
// 					(*p11_3)[0]=lineLength[i][j].length;
// 					kdtree11.add_point(p11);
// 					kdtree11.add_point(p11_1);
// 					kdtree11.add_point(p11_2);
// 					kdtree11.add_point(p11_3);
                }
            }

        }
        print_progress(float(i+1)/linesNum);
    }
    kdtree11.end();
    kdtree12.end();
    kdtree1211.end();
    kdtree21.end();
    kdtree2111.end();
    kdtree22.end();
    kdtree2212.end();
    kdtree2221.end();
    kdtree2211.end();

    std::vector<std::vector<Eigen::VectorXf *> *> vec;
    vec.push_back(&vec11);
    vec.push_back(&vec12);
    vec.push_back(&vec1211);
    vec.push_back(&vec21);
    vec.push_back(&vec2111);
    vec.push_back(&vec22);
    vec.push_back(&vec2211);
    vec.push_back(&vec2212);
    vec.push_back(&vec2221);
    int count = 0;
    for (size_t i = 0; i < vec.size(); i++) {
        for (size_t j = 0; j < (*vec[i]).size(); j++) {
            delete (*vec[i])[j];
            count++;
        }
    }
    std::cout << std::endl << "done. total descriptors: " << count << ". time : " << w.time_string() << std::endl;
    return 0;
}

int ComputeNearstTwoPointsOfTwo3DLine(Eigen::Vector3f &line1Vec, Eigen::Vector3f &line1Point,
                                      Eigen::Vector3f &line2Vec, Eigen::Vector3f &line2Point,
                                      Eigen::Vector3f &point1, Eigen::Vector3f &point2,
                                      double &minLength) {
    line1Vec.normalize();
    line2Vec.normalize();
    if (line1Vec == line2Vec) {
        return -1;
    }
    //////////////////////////////////////////////////////////////////////////
    Eigen::Vector3f directionVector = line1Vec.cross(line2Vec);
    directionVector.normalize();
    Eigen::Vector3f AB = line2Point - line1Point;
    //compute minLength
    minLength = abs(AB.dot(directionVector));
    //compute nearstPoint
    cv::Mat A(9, 9, CV_32F, cv::Scalar(0));
    cv::Mat B(9, 1, CV_32F, cv::Scalar(0));
    cv::Mat X;
    std::vector<float *> ptrA(9), ptrB(9);
    for (int i = 0; i < 9; i++) {
        ptrA[i] = A.ptr<float>(i);
        ptrB[i] = B.ptr<float>(i);
    }
    ptrA[0][0] = 1;
    ptrA[0][3] = -line1Vec(0);
    ptrA[1][1] = 1;
    ptrA[1][3] = -line1Vec(1);
    ptrA[2][2] = 1;
    ptrA[2][3] = -line1Vec(2);
    ptrA[3][4] = 1;
    ptrA[3][7] = -line2Vec(0);
    ptrA[4][5] = 1;
    ptrA[4][7] = -line2Vec(1);
    ptrA[5][6] = 1;
    ptrA[5][7] = -line2Vec(2);
    ptrA[6][0] = -1;
    ptrA[6][4] = 1;
    ptrA[6][8] = -directionVector(0);
    ptrA[7][1] = -1;
    ptrA[7][5] = 1;
    ptrA[7][8] = -directionVector(1);
    ptrA[8][2] = -1;
    ptrA[8][6] = 1;
    ptrA[8][8] = -directionVector(2);
    ptrB[0][0] = line1Point(0);
    ptrB[1][0] = line1Point(1);
    ptrB[2][0] = line1Point(2);
    ptrB[3][0] = line2Point(0);
    ptrB[4][0] = line2Point(1);
    ptrB[5][0] = line2Point(2);

    cv::solve(A, B, X, cv::DECOMP_SVD);
    point1(0) = X.at<float>(0);
    point1(1) = X.at<float>(1);
    point1(2) = X.at<float>(2);
    point2(0) = X.at<float>(4);
    point2(1) = X.at<float>(5);
    point2(2) = X.at<float>(6);
    minLength = (point1 - point2).norm();

    return 0;
}


float g_angleThreshold = 0.0076;//5 degree
bool EnforceSimilarity(const pcl::PointXYZINormal &point_a, const pcl::PointXYZINormal &point_b,
                       float squared_distance) {
    Eigen::VectorXf temp(3);
    temp[0] = point_a.normal_x - point_b.normal_x;
    temp[1] = point_a.normal_y - point_b.normal_y;
    temp[2] = point_a.normal_z - point_b.normal_z;
    if (temp.squaredNorm() < g_angleThreshold) {
        return true;
    }
    return false;
}

int ClusterTransformation(std::vector<Eigen::Matrix3f> &Rs,
                          std::vector<Eigen::Vector3f> &Ts,
                          float distanceThreshold,
                          float angleThreshold,
                          pcl::IndicesClusters &clusters) {
    if (Rs.empty() || Rs.size() != Ts.size()) {
        return -1;
    }
    //////////////////////////////////////////////////////////////////////////
    size_t transNum = Rs.size();
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr trans(new pcl::PointCloud<pcl::PointXYZINormal>);
    trans->resize(transNum);
    for (size_t i = 0; i < transNum; i++) {
        (*trans)[i].x = Ts[i][0];
        (*trans)[i].y = Ts[i][1];
        (*trans)[i].z = Ts[i][2];
        Eigen::Transform<float, 3, Eigen::Affine> R(Rs[i]);
        pcl::getEulerAngles<float>(R, (*trans)[i].normal_x, (*trans)[i].normal_y, (*trans)[i].normal_z);
    }

    pcl::ConditionalEuclideanClustering<pcl::PointXYZINormal> cec(true);
    cec.setInputCloud(trans);
    cec.setConditionFunction(&EnforceSimilarity);
    // Points within this distance from one another are going to need to validate the enforceIntensitySimilarity function to be part of the same cluster:
    cec.setClusterTolerance(distanceThreshold);
    g_angleThreshold = angleThreshold;
    // Size constraints for the clusters:
    cec.setMinClusterSize(1);
    cec.setMaxClusterSize(static_cast<int>(transNum));
    // The resulting clusters (an array of pointindices):
    cec.segment(clusters);
    return 0;
}

int AreTwoPlanesPenetrable(Eigen::Vector4f &plane1, Eigen::Vector4f &plane2,
                           std::vector<Eigen::Vector3f> &cornerPoints1,
                           std::vector<Eigen::Vector3f> &cornerPoints2,
                           pcl::search::KdTree<pcl::PointXYZ>::Ptr KdTree1,
                           pcl::search::KdTree<pcl::PointXYZ>::Ptr KdTree2,
                           bool &isPentrable, float searchRadius, int minPointsNum, float minDistance) {

    isPentrable = false;
    if (cornerPoints1.empty() || cornerPoints2.empty()) {
        return -1;
    }
    //////////////////////////////////////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points1 = KdTree1->getInputCloud();
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr points2 = KdTree2->getInputCloud();

    //compute the intersection line of two planes
    Eigen::Vector3f lineVec, linePoint;
    if (0 != ComputeIntersectionLineOfTwoPlanes(plane1, plane2, lineVec, linePoint)) {
        return -1;
    }
    //check the intersection line with first plane
    size_t num1 = cornerPoints1.size();
    std::vector<Eigen::Vector3f> intersectionPoints1;
    intersectionPoints1.reserve(2);
    for (size_t i = 1; i <= num1; i++) {
        Eigen::Vector3f tempLine;
        tempLine = cornerPoints1[i % num1] - cornerPoints1[(i - 1) % num1];
        tempLine.normalize();
        Eigen::Vector3f ip;
        if (0 != ComputeIntersectionPointOf23DLine(lineVec, linePoint, tempLine, cornerPoints1[i - 1], ip)) {
            continue;
        }

        if ((cornerPoints1[(i - 1) % num1] - ip).dot(cornerPoints1[i % num1] - ip) > 0) {
            continue;
        }

        intersectionPoints1.push_back(ip);
    }


    size_t num2 = cornerPoints2.size();
    std::vector<Eigen::Vector3f> intersectionPoints2;
    intersectionPoints2.reserve(2);
    for (size_t i = 1; i <= num2; i++) {
        Eigen::Vector3f tempLine;

        tempLine = cornerPoints2[i % num2] - cornerPoints2[(i - 1) % num2];

        tempLine.normalize();
        Eigen::Vector3f ip;
        if (0 != ComputeIntersectionPointOf23DLine(lineVec, linePoint, tempLine, cornerPoints2[i - 1], ip)) {
            continue;
        }
        if ((cornerPoints2[(i - 1) % num2] - ip).dot(cornerPoints2[i % num2] - ip) > 0) {
            continue;
        }
        intersectionPoints2.push_back(ip);
    }

    if (intersectionPoints1.empty()) {
        isPentrable = false;
        return 0;
    } else if (2 != intersectionPoints1.size()) {
        return -1;
    }

    if (intersectionPoints2.empty()) {
        isPentrable = false;
        return 0;
    } else if (2 != intersectionPoints2.size()) {
        return -1;
    }

    //further check
    //check the intersection part
    Eigen::Vector3f direc = intersectionPoints1[1] - intersectionPoints1[0];
    direc.normalize();

    std::vector<Eigen::Vector3f> interPoints = intersectionPoints1;
    interPoints.push_back(intersectionPoints2[0]);
    interPoints.push_back(intersectionPoints2[1]);
    std::vector<LENGTHINDEX> lengthVector(interPoints.size());
    for (size_t i = 0; i < interPoints.size(); i++) {
        lengthVector[i].length = (interPoints[i] - interPoints[0]).dot(direc);
        lengthVector[i].index = static_cast<int>(i);
    }
    sort(lengthVector.begin(), lengthVector.end(), myCompareLess);// ��С��������
    if (0 == (lengthVector[0].index / 2 - lengthVector[1].index / 2)) {
        //no intersection
        isPentrable = false;
        return 0;
    }
    Eigen::Vector3f &startPoint = interPoints[lengthVector[1].index];
    Eigen::Vector3f &endPoint = interPoints[lengthVector[2].index];

    Eigen::Vector3f searchPoint;
    std::vector<int> neighbor;
    std::vector<float> neighborLength;
    int negativeNum = 0;
    int positiveNum = 0;
    float length = (endPoint - startPoint).norm();
    std::vector<bool> checkIndex1(points1->size(), true);
    int count = 0;
    for (float dist = 0; dist < length; dist += searchRadius) {
        searchPoint = startPoint + dist * direc;
        if (KdTree2->radiusSearch(pcl::PointXYZ(searchPoint(0), searchPoint(1), searchPoint(2)),
                                  searchRadius / 2, neighbor, neighborLength, 2) < 2) {
            count++;
            continue;
        }
        KdTree1->radiusSearch(pcl::PointXYZ(searchPoint(0), searchPoint(1), searchPoint(2)),
                              searchRadius, neighbor, neighborLength);
        for (size_t i = 0; i < neighbor.size(); i++) {
            if (checkIndex1[neighbor[i]]) {
                checkIndex1[neighbor[i]] = false;
                const pcl::PointXYZ &p = points1->at(neighbor[i]);
                float tempDistance = plane2[0] * p.x + plane2[1] * p.y + plane2[2] * p.z + plane2[3];
                if (abs(tempDistance) > minDistance) {
                    if (tempDistance >= 0) {
                        positiveNum++;
                    } else {
                        negativeNum++;
                    }
                }
            }
        }
    }

    if (positiveNum < minPointsNum || negativeNum < minPointsNum) {
        isPentrable = false;
        return 0;
    }
    if (double(MAX(positiveNum, negativeNum)) / MIN(positiveNum, negativeNum + 1) > 5) {
        isPentrable = false;
        return 0;
    }

    negativeNum = 0;
    positiveNum = 0;
    std::vector<bool> checkIndex2(points2->size(), true);
    count = 0;
    for (float dist = 0; dist < length; dist += searchRadius) {
        searchPoint = startPoint + dist * direc;
        if (KdTree1->radiusSearch(pcl::PointXYZ(searchPoint(0), searchPoint(1), searchPoint(2)),
                                  searchRadius / 2, neighbor, neighborLength, 2) < 2) {
            count++;
            continue;
        }
        KdTree2->radiusSearch(pcl::PointXYZ(searchPoint(0), searchPoint(1), searchPoint(2)),
                              searchRadius, neighbor, neighborLength);
        for (size_t i = 0; i < neighbor.size(); i++) {
            if (checkIndex2[neighbor[i]]) {
                checkIndex2[neighbor[i]] = false;
                const pcl::PointXYZ &p = points2->at(neighbor[i]);
                float tempDistance = plane1[0] * p.x + plane1[1] * p.y + plane1[2] * p.z + plane1[3];
                if (abs(tempDistance) > minDistance) {
                    if (tempDistance >= 0) {
                        positiveNum++;
                    } else {
                        negativeNum++;
                    }
                }
            }
        }
    }

    if (positiveNum < minPointsNum && negativeNum < minPointsNum) {
        isPentrable = false;
        return 0;
    }
    if (double(MAX(positiveNum, negativeNum)) / MIN(positiveNum, negativeNum + 1) > 5) {
        isPentrable = false;
        return 0;
    }

    isPentrable = true;

    return 0;
}


int ComputeIntersectionPointOf23DLine(Eigen::Vector3f &lineVec1, Eigen::Vector3f &linePoint1,
                                      Eigen::Vector3f &lineVec2, Eigen::Vector3f &linePoint2,
                                      Eigen::Vector3f &intersectionPoint) {
    if (abs(lineVec1.dot(lineVec2)) > 0.9999) {
        return -1;//parallel
    }
    cv::Mat A(6, 5, CV_32F, cv::Scalar(0));
    cv::Mat B(6, 1, CV_32F, cv::Scalar(0));
    cv::Mat X;
    std::vector<float *> ptrA(6), ptrB(6);
    for (int i = 0; i < 6; i++) {
        ptrA[i] = A.ptr<float>(i);
        ptrB[i] = B.ptr<float>(i);
    }
    ptrA[0][0] = 1;
    ptrA[0][3] = -lineVec1(0);
    ptrA[1][1] = 1;
    ptrA[1][3] = -lineVec1(1);
    ptrA[2][2] = 1;
    ptrA[2][3] = -lineVec1(2);
    ptrA[3][0] = 1;
    ptrA[3][4] = -lineVec2(0);
    ptrA[4][1] = 1;
    ptrA[4][4] = -lineVec2(1);
    ptrA[5][2] = 1;
    ptrA[5][4] = -lineVec2(2);
    ptrB[0][0] = linePoint1(0);
    ptrB[1][0] = linePoint1(1);
    ptrB[2][0] = linePoint1(2);
    ptrB[3][0] = linePoint2(0);
    ptrB[4][0] = linePoint2(1);
    ptrB[5][0] = linePoint2(2);

    cv::solve(A, B, X, cv::DECOMP_SVD);
    intersectionPoint(0) = X.at<float>(0);
    intersectionPoint(1) = X.at<float>(1);
    intersectionPoint(2) = X.at<float>(2);
    return 0;

}

float random_float(float min, float max) {
    return min + float(std::rand()) / float(RAND_MAX) * (max - min);
}

void save_vg(const pcl::PointCloud<pcl::PointNormal>& cloud, const std::vector<std::vector<int>>& planes, const std::string& file_name) {
    /*
group_type: type (integer: 	PLANE = 0, CYLINDER = 1, SPHERE = 2, CONE = 3, TORUS = 4, GENERAL = 5)
num_group_parameters: NUM_GROUP_PARAMETERS   // number of floating point values (integer)
group_parameters: float[NUM_GROUP_PARAMETERS]
group_label: label  // the first group info
group_color: color (r, g, b)
group_num_points: num
idx ...
*/
    auto write_ascii_group = [](std::ostream& out, const std::vector<int>& plane) -> void {
        out << "group_type: " << 0 << std::endl; // 0 for plane

        out << "num_group_parameters: " << 4 << std::endl;
        out << "group_parameters: ";
        for (std::size_t i = 0; i < 4; ++i)
            out << 0 << " ";
        out << std::endl;

        out << "group_label: unknown"  << std::endl;

        float r = random_float(0.3f, 1.0f);
        float g = random_float(0.3f, 1.0f);
        float b = random_float(0.3f, 1.0f);
        out << "group_color: " << r << " " << g << " " << b << std::endl;

        out << "group_num_point: " << plane.size() << std::endl;
        for (std::size_t i = 0; i < plane.size(); ++i)
            out << plane[i] << " ";
        out << std::endl;
    };

    // open file
    std::ofstream output(file_name.c_str());
    if (output.fail()) {
        std::cerr << "could not open file: " << file_name;
        return;
    }
    output.precision(16);

    output << "num_points: " << cloud.size() << std::endl;
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const auto& pn = cloud.at(i);
        output << pn.x << " " << pn.y << " " << pn.z << " ";
    }
    output << std::endl;

    output << "num_colors: " << 0 << std::endl;
    output << "num_normals: " << cloud.size() << std::endl;
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const auto& n = cloud.at(i).normal;
        output << n[0] << " " << n[1] << " " << n[2] << " ";
    }
    output << std::endl;

    output << "num_groups: " << planes.size() << std::endl;
    for (std::size_t i = 0; i < planes.size(); ++i) {
        const std::vector<int>& plane = planes[i];
        write_ascii_group(output, plane);

        // children
        output << "num_children: " << 0 << std::endl;
    }
}


float average_spacing(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud, int k, bool accurate, int samples) {
    pcl::search::KdTree<pcl::PointNormal> kdtree;
    kdtree.setInputCloud(cloud);

    double total = 0.0;
    size_t num = cloud->size();

    size_t step = 1;
    if (!accurate && num > samples)
        step = num / samples;
    size_t total_count = 0;
    for (size_t i = 0; i < num; i += step) {
        const auto &pn = cloud->at(i);
        std::vector<int> k_indices;
        std::vector<float> k_sqr_distances;
        int nbs = kdtree.nearestKSearch(pn, k, k_indices, k_sqr_distances);
        if (nbs <= 1 || nbs != k_sqr_distances.size()) {// in case we get less than k+1 neighbors
            continue;
        }

        double avg = 0.0;
        for (unsigned int i = 1; i < nbs; ++i) { // starts from 1 to exclude itself
            avg += std::sqrt(k_sqr_distances[i]);
        }
        total += (avg / nbs);
        ++total_count;
    }

    return static_cast<float>(total / total_count);
}


void print_progress(float percentage) {
    static const int width = 60;
    const int lpad = static_cast<int>(percentage * width);
    const int rpad = width - lpad;
    const int value = static_cast<int>(percentage * 100);
#if 1
    // The string in the square brackets has two parts; left and right.
    // The left part consists of lpad characters of the filling string printed using the %.*s specifier,
    // while the right part consists of rpad length of a space left-padded string which we chose to be empty ""
    // so that we only print rpad spaces using the %*s specifier.
    static const std::string filling_str(width, '|');;
    printf("\r\t%3d%% [%.*s%*s]", value, lpad, filling_str.data(), rpad, "");
    fflush(stdout);
#else
    // explicitly construct filling and padding strings. This might be less efficient.
        printf("\r%3d%% [%s%s] ", value, std::string(lpad, '|').c_str(), std::string(rpad, '-').c_str());
        fflush(stdout);
#endif
}

#include <cmath>
#include <sstream>
#include <iomanip>

// The windows.h has to come after <easy3d/core/types.h>. Otherwise the compiler
// will be confused by the min/max micros and the std::min, std::max.
#ifdef _WIN32
#include <Windows.h>
#endif // _WIN32


StopWatch::StopWatch() {
    start();
}

StopWatch::~StopWatch() {}


void StopWatch::start() {
#ifdef _WIN32
    LARGE_INTEGER  largeInteger;
    QueryPerformanceFrequency(&largeInteger);
    freq_ = largeInteger.QuadPart;
    QueryPerformanceCounter(&largeInteger);
    start_count_ = largeInteger.QuadPart;
#else
    gettimeofday(&start_time_, nullptr);
#endif // _WIN32
}


void StopWatch::restart() {
    start();
}


double StopWatch::seconds() const {
#ifdef _WIN32
    LARGE_INTEGER  largeInteger;
    QueryPerformanceCounter(&largeInteger);
    LONGLONG now_count = largeInteger.QuadPart;
    return static_cast<double>((now_count - start_count_) / static_cast<double>(freq_));
#else
    timeval now;
    gettimeofday(&now, nullptr);
    return (now.tv_sec - start_time_.tv_sec) + (now.tv_usec - start_time_.tv_usec) / 1.0e6;
#endif  // _WIN32
}


/** \brief Rounds the given floating point number \p v to have \p num digits.*/
template<class T>
inline T truncate_digits(const T &v, int num) {
    T tmp = std::pow(10.0, num);
    long long des = static_cast<long long>((v < 0) ? (v * tmp - 0.5) : (v * tmp + 0.5));
    T result = T(des) / tmp;
    return result;
}



double StopWatch::elapsed_seconds(int num_digits /* = 1*/) const {
    return truncate_digits(seconds(), num_digits);
}


std::string StopWatch::time_string(int num_digits /* = 1*/) const {
    double time = seconds() * 1000;
    if (std::isnan(time) || std::isinf(time))
        return "inf";

    std::string suffix = "ms";
    if (time > 1000) {
        time /= 1000;
        suffix = "s";
        if (time > 60) {
            time /= 60;
            suffix = "m";
            if (time > 60) {
                time /= 60;
                suffix = "h";
                if (time > 12) {
                    time /= 12;
                    suffix = "d";
                }
            }
        }
    }

    std::ostringstream os;
    os << std::setprecision(num_digits)
       << std::fixed << time << suffix;

    return os.str();
}
