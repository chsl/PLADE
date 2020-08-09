#include "stdafx.h"
//#define COMPILE
#ifdef COMPILE
#include "common.h"

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

int ChangeCvPoint2PclPoint(std::vector<cv::Point3f>& points1,pcl::PointCloud<pcl::PointXYZ> &points2)
{
	int pointsNum=points1.size();
	try
	{
		points2.resize(pointsNum);
		for (int i=0;i<pointsNum;i++)
		{
			points2[i].x=points1[i].x;
			points2[i].y=points1[i].y;
			points2[i].z=points1[i].z;
		}
	}
	catch (...)
	{
		return -1;
	}
	return 0;
}

int ChangePclPoint2CvPoint(pcl::PointCloud<pcl::PointXYZ> &points1,std::vector<cv::Point3f>& points2)
{
	int pointsNum=points1.size();
	try
	{
		points2.resize(pointsNum);
		for (int i=0;i<pointsNum;i++)
		{
			points2[i].x=points1[i].x;
			points2[i].y=points1[i].y;
			points2[i].z=points1[i].z;
		}
	}
	catch (...)
	{
		return -1;
	}
	return 0;
}


pcl::PointCloud<pcl::Normal>::Ptr computeNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr points,double normalEstimationRadius)
{
	SYSTEM_INFO si;  
	GetSystemInfo(&si);  
	int numThred=si.dwNumberOfProcessors;  
	pcl::PointCloud<pcl::Normal>::Ptr point_normal (new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::Normal> est_normal;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	est_normal.setInputCloud(points);
	est_normal.setNumberOfThreads(numThred);
	est_normal.setSearchMethod(tree);
	est_normal.setRadiusSearch (normalEstimationRadius);
	est_normal.compute(*point_normal);
	return point_normal;
}


int transformPlaneModel(pcl::ModelCoefficients &input, pcl::ModelCoefficients &outPut, Eigen::Matrix3f &R, Eigen::Vector3f &T)
{
	if (input.values.size()<4)
	{
		return -1;
	}
	Eigen::Vector3f plane(input.values[0],input.values[1],input.values[2]);
	Eigen::Vector3f transPlane=R*plane;
	outPut.values.clear();
	outPut.values.push_back(transPlane[0]);
	outPut.values.push_back(transPlane[1]);
	outPut.values.push_back(transPlane[2]);
	outPut.values.push_back(-(-(input.values[3])+(transPlane.transpose()*T)(0)));
	return 0;
}

struct NearstPointsTwoLine
{
	Eigen::Vector3f points1;
	Eigen::Vector3f points2;
	double angle;
	double length;
	double minAngle;
};

struct MatchInformation
{
	//lines
	std::vector<PCP::INTERSECTION_LINE> *pIntersectionLine;
	std::vector<std::vector<NearstPointsTwoLine>> *pNearstPointsTwoLines;

	//planes
	std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>> *pPlanes;
	std::vector<std::vector<int>> *pSupportPlaneIndex;
	std::vector<pcl::ModelCoefficients> *pModelPlaneCoff;//for visualization

	//points
	pcl::PointCloud<pcl::PointXYZ>::Ptr points;
	pcl::PointCloud<pcl::PointXYZ>::Ptr originalPoints;

	//boundingBox
	Eigen::Vector3f boundingCenter;//model
	//for each plane
	std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> *pDownSampleKdTree;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> *pDownSamplePlanePoints;
	std::vector<std::vector<Eigen::Vector3f>>*pBoundingBoxFourCornerPoints;
	std::vector<Eigen::Vector3f>*pBoundingBoxCenterForEachPlane;
	std::vector<float>*pBoundingBoxRadiusForEachPlane;
};

struct Parameter
{
	double lengthThreshold;
	double angleThreshold;
	double cosAngleThreshold;
	int    maxCandidateResultNum;
	std::vector<KdTreeSearchNDim<Eigen::VectorXf,8>> *kdtrees8;
	std::vector<KdTreeSearchNDim<Eigen::VectorXf,6>> *kdtrees6;
	std::vector<KdTreeSearchNDim<Eigen::VectorXf,4>> *kdtrees4;
	std::vector<std::vector<PCP::PAIRLINE>> *mainLinesInformation;
	int   maxNeighbor;
	float maxRadius;
};

struct MatchedResult
{
	std::vector<std::pair<int,int>>matchedPlanes;
	Eigen::Matrix3f R;
	Eigen::Vector3f T;
	float planeScore;
};

struct QueryResult
{
	PCP::PAIRLINE queryLine;
	std::vector<int> neighbor;
	std::vector<float> neighborDistance;
	QueryResult(PCP::PAIRLINE queryLine,std::vector<int> neighbor,std::vector<float> neighborDistance)
		:queryLine(queryLine),neighbor(neighbor),neighborDistance(neighborDistance){}
};

void MatchingLines(MatchInformation current,
				   MatchInformation main, 
				   std::vector<std::pair<int,int>> linesTobeMatched,
				   std::vector<std::vector<int>>coarseMatches,
				   std::vector<MatchedResult>* pMatchedResult,
				   Parameter Parameter)
{
	std::vector<PCP::INTERSECTION_LINE> &currentIntersectionLines=*(current.pIntersectionLine);
	std::vector<std::vector<NearstPointsTwoLine>> &currentLineLength=*(current.pNearstPointsTwoLines);
	std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>> &planes=*(current.pPlanes);
	std::vector<std::vector<int>> &supportPlaneIndex=*(current.pSupportPlaneIndex);
	pcl::PointCloud<pcl::PointXYZ>::Ptr points=current.points;
	pcl::PointCloud<pcl::PointXYZ>::Ptr currentOriginalPoints=current.originalPoints;
	Eigen::Vector3f currentBoundingCenter=current.boundingCenter;
	std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> &currentDownSampleKdTree=*(current.pDownSampleKdTree);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &downSampleCurrentPlanePoints=*(current.pDownSamplePlanePoints);
	std::vector<std::vector<Eigen::Vector3f>> &currentBoundingBoxFourCornerPoints=*(current.pBoundingBoxFourCornerPoints);
	std::vector<Eigen::Vector3f> &currentBoundingBoxCenterForEachPlane=*(current.pBoundingBoxCenterForEachPlane);
	std::vector<float> &currentBoundingBoxRadiusForEachPlane=*(current.pBoundingBoxRadiusForEachPlane);


	std::vector<PCP::INTERSECTION_LINE> &mainIntersectionLines=*(main.pIntersectionLine);
	std::vector<std::vector<NearstPointsTwoLine>> &mainLineLength=*(main.pNearstPointsTwoLines);
	std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>> &mainPlanes=*(main.pPlanes);
	std::vector<std::vector<int>> &supportMainPlaneIndex=*(main.pSupportPlaneIndex);
	pcl::PointCloud<pcl::PointXYZ>::Ptr mainPoints=main.points;
	pcl::PointCloud<pcl::PointXYZ>::Ptr mainOriginalPoints=main.originalPoints;
	Eigen::Vector3f mianBoundingCenter=main.boundingCenter;
	std::vector<std::vector<Eigen::Vector3f>> &mainBoundingBoxFourCornerPoints=*(main.pBoundingBoxFourCornerPoints);
	std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> &mainDownSampleKdTree=*(main.pDownSampleKdTree);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &downSampleMainPlanePoints=*(main.pDownSamplePlanePoints);
	std::vector<Eigen::Vector3f> &mainBoundingBoxCenterForEachPlane=*(main.pBoundingBoxCenterForEachPlane);
	std::vector<float> &mainBoundingBoxRadiusForEachPlane=*(main.pBoundingBoxRadiusForEachPlane);

	//kdtrees8 22
	//kdtrees6 12 22-12 21 22-21
	//kdtrees4 11 12-11 21-11 22-11
	//linesInformation 22 12 22-12 21 22-21 11 12-11 21-11 22-11
	std::vector<KdTreeSearchNDim<Eigen::VectorXf,8>> &kdtrees8=*Parameter.kdtrees8;
	std::vector<KdTreeSearchNDim<Eigen::VectorXf,6>> &kdtrees6=*Parameter.kdtrees6;
	std::vector<KdTreeSearchNDim<Eigen::VectorXf,4>> &kdtrees4=*Parameter.kdtrees4;
	std::vector<std::vector<PCP::PAIRLINE>> &mainLinesInformation=*Parameter.mainLinesInformation;
	KdTreeSearchNDim<Eigen::VectorXf,8> &kdtree22=kdtrees8[0]; 
	KdTreeSearchNDim<Eigen::VectorXf,6> &kdtree12=kdtrees6[0];
	KdTreeSearchNDim<Eigen::VectorXf,6> &kdtree2212=kdtrees6[1];
	KdTreeSearchNDim<Eigen::VectorXf,6> &kdtree21=kdtrees6[2]; 
	KdTreeSearchNDim<Eigen::VectorXf,6> &kdtree2221=kdtrees6[3]; 
	KdTreeSearchNDim<Eigen::VectorXf,4> &kdtree11=kdtrees4[0]; 
	KdTreeSearchNDim<Eigen::VectorXf,4> &kdtree1211=kdtrees4[1]; 
	KdTreeSearchNDim<Eigen::VectorXf,4> &kdtree2111=kdtrees4[2]; 
	KdTreeSearchNDim<Eigen::VectorXf,4> &kdtree2211=kdtrees4[3]; 

	std::vector<PCP::PAIRLINE> &lf22=mainLinesInformation[0];
	std::vector<PCP::PAIRLINE> &lf12=mainLinesInformation[1];
	std::vector<PCP::PAIRLINE> &lf2212=mainLinesInformation[2];
	std::vector<PCP::PAIRLINE> &lf21=mainLinesInformation[3];
	std::vector<PCP::PAIRLINE> &lf2221=mainLinesInformation[4];
	std::vector<PCP::PAIRLINE> &lf11=mainLinesInformation[5];
	std::vector<PCP::PAIRLINE> &lf1211=mainLinesInformation[6];
	std::vector<PCP::PAIRLINE> &lf2111=mainLinesInformation[7];
	std::vector<PCP::PAIRLINE> &lf2211=mainLinesInformation[8];

	std::vector<std::vector<QueryResult>> queryResult(9);
	std::vector<QueryResult> &query22=queryResult[0];
	std::vector<QueryResult> &query12=queryResult[1];
	std::vector<QueryResult> &query2212=queryResult[2];
	std::vector<QueryResult> &query21=queryResult[3];
	std::vector<QueryResult> &query2221=queryResult[4];
	std::vector<QueryResult> &query11=queryResult[5];
	std::vector<QueryResult> &query1211=queryResult[6];
	std::vector<QueryResult> &query2111=queryResult[7];
	std::vector<QueryResult> &query2211=queryResult[8];

	if (pMatchedResult)
	{
		pMatchedResult->clear();
		pMatchedResult->reserve(200);
	}
	else
	{
		cout<<"fatal error"<<endl;
		return;
	}
	int currentPlanesNum=planes.size();
	int mainPlanesNum=mainPlanes.size();

	//Parameter
	float lengthThreshold=Parameter.lengthThreshold;
	float angleThreshold=Parameter.angleThreshold;
	float cosAngleTh=Parameter.cosAngleThreshold;
	int maxNeighbor=Parameter.maxNeighbor;
	double maxNeighborDistance=0.04;
	double squaredMaxNeighbordistance=maxNeighborDistance*maxNeighborDistance;

	std::vector<Eigen::Vector3f> suppotPlanes(planes.size());
	for (size_t i=0;i<planes.size();i++)
	{
		suppotPlanes[i]=planes[i].block(0,0,3,1);
	}
	std::vector<PCP::PAIRLINE> queryLines;
	queryLines.reserve(linesTobeMatched.size()*4);
	std::vector<std::vector<int>> matchedLinesIndex;
	matchedLinesIndex.reserve(linesTobeMatched.size()*4);
	std::vector<std::vector<float>> matchedLinesDistance;
	matchedLinesDistance.reserve(linesTobeMatched.size()*4);
	int totalQueryDescriptors=0;
	for (size_t k=0;k<linesTobeMatched.size();k++)
	{
		size_t i=linesTobeMatched[k].first;
		size_t j=linesTobeMatched[k].second;

		//find matches
		PCP::INTERSECTION_LINE &line1=currentIntersectionLines[i];
		PCP::INTERSECTION_LINE &line2=currentIntersectionLines[j];
		PCP::PAIRLINE pairLine;
		pairLine.linePoints1=currentLineLength[i][j].points1;
		pairLine.linePoints2=currentLineLength[i][j].points2;
		pairLine.originalIndex1=i;
		pairLine.originalIndex2=j;
		std::vector<int> neighbor;
		std::vector<float> neighborDistance;
		if (line1.isIntersectionLine)
		{
			if (line2.isIntersectionLine)//22
			{
				int l1sp1=line1.supportPlanes[0];
				int l1sp2=line1.supportPlanes[1];
				int l2sp1=line2.supportPlanes[0];
				int l2sp2=line2.supportPlanes[1];

				//22
				Eigen::VectorXf *p22=new Eigen::VectorXf(8);
				ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],suppotPlanes[l1sp2],
					suppotPlanes[l2sp1],suppotPlanes[l2sp2],*p22,
					pairLine.lineVec1,pairLine.lineVec2,PCP::method22);
				(*p22)[0]=currentLineLength[i][j].length;
				kdtree22.find_neighbors(*p22,maxNeighbor,maxNeighborDistance,neighbor,neighborDistance);
				if (!neighbor.empty())
				{
					pairLine.descriptor=*p22;
					query22.push_back(QueryResult(pairLine,neighbor,neighborDistance));
				}
				delete p22;

				//22-21 search 21
				Eigen::VectorXf *p2221_1=new Eigen::VectorXf(6);
				ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],suppotPlanes[l1sp2],
					suppotPlanes[l2sp1],line2.lineVec.cross(suppotPlanes[l2sp1]),*p2221_1,
					pairLine.lineVec1,pairLine.lineVec2,PCP::method21);
				(*p2221_1)[0]=currentLineLength[i][j].length;
				kdtree21.find_neighbors(*p2221_1,maxNeighbor,maxNeighborDistance,neighbor,neighborDistance);
				if (!neighbor.empty())
				{
					pairLine.descriptor=*p2221_1;
					query21.push_back(QueryResult(pairLine,neighbor,neighborDistance));
				}
				delete p2221_1;

				Eigen::VectorXf *p2221_2=new Eigen::VectorXf(6);
				ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],suppotPlanes[l1sp2],
					suppotPlanes[l2sp2],line2.lineVec.cross(suppotPlanes[l2sp2]),*p2221_2,
					pairLine.lineVec1,pairLine.lineVec2,PCP::method21);
				(*p2221_2)[0]=currentLineLength[i][j].length;
				kdtree21.find_neighbors(*p2221_2,maxNeighbor,maxNeighborDistance,neighbor,neighborDistance);
				if (!neighbor.empty())
				{
					pairLine.descriptor=*p2221_2;
					query21.push_back(QueryResult(pairLine,neighbor,neighborDistance));
				}
				delete p2221_2;

				//22-12 search 12
				Eigen::VectorXf *p2212_1=new Eigen::VectorXf(6);
				ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],line1.lineVec.cross(suppotPlanes[l1sp1]),
					suppotPlanes[l2sp1],suppotPlanes[l2sp2],*p2212_1,
					pairLine.lineVec1,pairLine.lineVec2,PCP::method12);
				(*p2212_1)[0]=currentLineLength[i][j].length;
				kdtree12.find_neighbors(*p2212_1,maxNeighbor,maxNeighborDistance,neighbor,neighborDistance);
				if (!neighbor.empty())
				{
					pairLine.descriptor=*p2212_1;
					query12.push_back(QueryResult(pairLine,neighbor,neighborDistance));
				}

				delete p2212_1;

				Eigen::VectorXf *p2212_2=new Eigen::VectorXf(6);
				ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp2],line1.lineVec.cross(suppotPlanes[l1sp2]),
					suppotPlanes[l2sp1],suppotPlanes[l2sp2],*p2212_2,
					pairLine.lineVec1,pairLine.lineVec2,PCP::method12);
				(*p2212_2)[0]=currentLineLength[i][j].length;
				kdtree12.find_neighbors(*p2212_2,maxNeighbor,maxNeighborDistance,neighbor,neighborDistance);
				if (!neighbor.empty())
				{
					pairLine.descriptor=*p2212_2;
					query12.push_back(QueryResult(pairLine,neighbor,neighborDistance));
				}

				delete p2212_2;
				totalQueryDescriptors+=5;

			}
			else//21
			{
				int l1sp1=line1.supportPlanes[0];
				int l1sp2=line1.supportPlanes[1];
				int l2sp1=line2.supportPlanes[0];
				//21 search 21
				Eigen::VectorXf *p21=new Eigen::VectorXf(6);
				ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],suppotPlanes[l1sp2],
					suppotPlanes[l2sp1],line2.lineVec.cross(suppotPlanes[l2sp1]),*p21,
					pairLine.lineVec1,pairLine.lineVec2,PCP::method21);
				(*p21)[0]=currentLineLength[i][j].length;
				kdtree21.find_neighbors(*p21,maxNeighbor,maxNeighborDistance,neighbor,neighborDistance);
				if (!neighbor.empty())
				{
					pairLine.descriptor=*p21;
					query21.push_back(QueryResult(pairLine,neighbor,neighborDistance));
				}
				//search 22-21
				kdtree2221.find_neighbors(*p21,maxNeighbor,maxNeighborDistance,neighbor,neighborDistance);
				if (!neighbor.empty())
				{
					pairLine.descriptor=*p21;
					query2221.push_back(QueryResult(pairLine,neighbor,neighborDistance));
				}

				delete p21;
				totalQueryDescriptors+=2;
			}
		}
		else
		{
			if (line2.isIntersectionLine)//12
			{
				int l1sp1=line1.supportPlanes[0];
				int l2sp1=line2.supportPlanes[0];
				int l2sp2=line2.supportPlanes[1];
				//12 
				Eigen::VectorXf *p12=new Eigen::VectorXf(6);
				ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],line1.lineVec.cross(suppotPlanes[l1sp1]),
					suppotPlanes[l2sp1],suppotPlanes[l2sp2],*p12,
					pairLine.lineVec1,pairLine.lineVec2,PCP::method12);
				(*p12)[0]=currentLineLength[i][j].length;
				kdtree12.find_neighbors(*p12,maxNeighbor,maxNeighborDistance,neighbor,neighborDistance);
				if (!neighbor.empty())
				{
					pairLine.descriptor=*p12;
					query12.push_back(QueryResult(pairLine,neighbor,neighborDistance));
				}

				//search 22-12
				kdtree2212.find_neighbors(*p12,maxNeighbor,maxNeighborDistance,neighbor,neighborDistance);
				if (!neighbor.empty())
				{
					pairLine.descriptor=*p12;
					query2212.push_back(QueryResult(pairLine,neighbor,neighborDistance));
				}

				delete p12;
				totalQueryDescriptors+=2;
			}

		}
	}

	//cout<<"totalQueryDescriptors:"<<totalQueryDescriptors<<endl;

	Eigen::Matrix3f R;
	Eigen::Vector3f T;
	std::vector<Eigen::Matrix3f>initialRs;
	std::vector<Eigen::Vector3f>initialTs;
	std::vector<std::string> displayInfor;
	//kdtrees8 22
	//kdtrees6 12 22-12 21 22-21
	//kdtrees4 11 12-11 21-11 22-11
	displayInfor.push_back("22");
	displayInfor.push_back("12");
	displayInfor.push_back("22-12");
	displayInfor.push_back("21");
	displayInfor.push_back("22-21");
	displayInfor.push_back("11");
	displayInfor.push_back("12-11");
	displayInfor.push_back("21-11");
	displayInfor.push_back("22-11");
	for (size_t i=0;i<queryResult.size();i++)
	{
		//cout<<displayInfor[i]<<endl;
		std::vector<QueryResult> &currentQuery=queryResult[i];
		std::vector<PCP::PAIRLINE> &currentMatched=mainLinesInformation[i];
		size_t currentQuerySize=currentQuery.size();
		for (size_t j=0;j<currentQuerySize;j++)
		{
			QueryResult &res=currentQuery[j];
			size_t neighborNum=res.neighbor.size();
			for (size_t k=0;k<neighborNum;k++)
			{
				// compute the transformation
				PCP::ComputeTransformationUsingTwoVecAndOnePoint(res.queryLine.lineVec1,
					res.queryLine.lineVec2,
					currentMatched[res.neighbor[k]].lineVec1,
					currentMatched[res.neighbor[k]].lineVec2,
					res.queryLine.linePoints1,
					currentMatched[res.neighbor[k]].linePoints1,
					R,T);
				initialRs.push_back(R);
				initialTs.push_back(T);
			}
		}
	}

	//cout<<"initial matches:"<<initialRs.size()<<endl;
	pcl::IndicesClusters cluster;
	PCP::ClusterTransformation(initialRs,initialTs,Parameter.lengthThreshold/2,Parameter.angleThreshold/2,cluster);
	//cout<<"total cluster Num:"<<cluster.size();
	std::vector<std::vector<std::pair<int,int>>>matches;//for each match,save the all matched planes index
	matches.reserve(cluster.size());
	std::vector<LENGTHINDEX>sortVec(cluster.size());
	int clusterNumOfGreaterThan1=0;
	for (size_t i=0;i<sortVec.size();i++)
	{
		sortVec[i].index=i;
		sortVec[i].length=cluster[i].indices.size();
		if (sortVec[i].length>1)
		{
			clusterNumOfGreaterThan1++;
		}
	}
	//cout<<" greater than 1 cluster num: "<<clusterNumOfGreaterThan1<<endl;
	sort(sortVec.begin(),sortVec.end(),myCompareGreater);
	std::vector<Eigen::Matrix3f>Rs;
	Rs.reserve(cluster.size());
	std::vector<Eigen::Vector3f>Ts;
	Ts.reserve(cluster.size());
	for (size_t i=0;i<sortVec.size();i++)
	{
		int index=sortVec[i].index;
		int k=cluster[index].indices[0];
		R=initialRs[k];
		T=initialTs[k];
		// check 
		Eigen::Vector3f tc=R*currentBoundingCenter+T;
		if ((tc-mianBoundingCenter).norm()>Parameter.maxRadius)//check whether point cloud is transformed to the location which is far away from destPointCloud
		{
			continue;
		}

		// count the number of consistent planes 
		std::vector<std::pair<int,int>> currentPlaneMatches;
		for (int i1=0;i1<currentPlanesNum;i1++)
		{
			Eigen::Vector3f plane1=R*planes[i1].block(0,0,3,1);
			float d=-(-planes[i1](3)+(plane1.block(0,0,3,1).transpose()*T)(0));

			//check center to plane distance
			Eigen::Vector3f srcCenter2Dest=R*currentBoundingBoxCenterForEachPlane[i1]+T;
			for (size_t j1=0;j1<mainPlanesNum;j1++)
			{
				if (plane1.dot(mainPlanes[j1].block(0,0,3,1))<cosAngleTh)
				{
					continue;
				}
				double center2PlaneDistance=(abs(mainPlanes[j1].block(0,0,3,1).dot(srcCenter2Dest)+mainPlanes[j1](3))
					+abs(plane1.block(0,0,3,1).dot(mainBoundingBoxCenterForEachPlane[j1])+d))/2;
				if (center2PlaneDistance>lengthThreshold)
				{
					continue;
				}
				double distance=(srcCenter2Dest-mainBoundingBoxCenterForEachPlane[j1]).norm();
				if (distance/(currentBoundingBoxRadiusForEachPlane[i1]+mainBoundingBoxRadiusForEachPlane[j1])>1)
				{
					//cout<<"reject one "<<endl;
					continue;
				}
				currentPlaneMatches.push_back(std::pair<int,int>(i1,j1));
				break;
			}
		}
		matches.push_back(currentPlaneMatches);
		Rs.push_back(R);
		Ts.push_back(T);
	}

	// find the correct matches
	size_t maxMatchNum=0;
	for (size_t i=0;i<matches.size();i++)
	{
		//cout<<matches[i].size()<<endl;
		if (maxMatchNum<matches[i].size())
		{
			maxMatchNum=matches[i].size();
		}
	}
	std::vector<std::vector<int>> matchedPlanes;
	matchedPlanes.reserve(Parameter.maxCandidateResultNum*2);
	if (maxMatchNum>0)
	{
		//cout<<"total planes:"<<planes.size()<<endl;
		///////////////////////////////////////////////////////
		int matchedCount=0;
		for (size_t i=maxMatchNum;i>=2;i--)
		{
			std::vector<int> tempMatchs;
			for (size_t j=0;j<matches.size();j++)
			{
				if (i==matches[j].size())
				{
					tempMatchs.push_back(j);
					matchedCount++;
				}
			}
			matchedPlanes.push_back(tempMatchs);
			//cout<<i<<"matched planes "<<tempMatchs.size()<<endl;
			if (matchedCount>=Parameter.maxCandidateResultNum)
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
	int count=0;
	for (size_t m=0;m<matchedPlanes.size();m++)
	{
		for (size_t i=0;i<matchedPlanes[m].size();i++)
		{
			if (count++>Parameter.maxCandidateResultNum)
			{
				return;
			}
			int index=matchedPlanes[m][i];
			pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformation;
			transformation=pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4::Identity();
			transformation.block(0,0,3,3)=Rs[index];
			transformation.block(0,3,3,1)=Ts[index];
			//check penetration
			bool isPentrable=false;
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tempKdtree(new pcl::search::KdTree<pcl::PointXYZ>);
			for (int i1=0;i1<currentPlanesNum;i1++)
			{
				isPentrable=false;
				Eigen::Vector4f plane1;
				plane1.block(0,0,3,1)=Rs[index]*planes[i1].block(0,0,3,1);
				plane1(3)=-(-planes[i1](3)+(plane1.block(0,0,3,1).transpose()*Ts[index])(0));
				pcl::PointCloud<pcl::PointXYZ>::Ptr tempTransPoints(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::transformPointCloud(*downSampleCurrentPlanePoints[i1],*tempTransPoints,transformation);
				pcl::PointCloud<pcl::PointXYZ> tempPclFourCorner;
				PCP::ExchnageBetweentPCLPointXYZwithEigenVector3f(tempPclFourCorner,currentBoundingBoxFourCornerPoints[i1]);
				pcl::transformPointCloud(tempPclFourCorner,tempPclFourCorner,transformation);
				std::vector<Eigen::Vector3f>tempEigenFourCorner;
				PCP::ExchnageBetweentPCLPointXYZwithEigenVector3f(tempPclFourCorner,tempEigenFourCorner);
				tempKdtree->setInputCloud(tempTransPoints);
				Eigen::Vector3f currentCenter2Main=Rs[index]*currentBoundingBoxCenterForEachPlane[i1]+Ts[index];
				for (size_t j1=0;j1<mainPlanesNum;j1++)
				{
					double center2PlaneDistance=(abs(mainPlanes[j1].block(0,0,3,1).dot(currentCenter2Main)+mainPlanes[j1](3))
						+abs(plane1.block(0,0,3,1).dot(mainBoundingBoxCenterForEachPlane[j1])+plane1(3)))/2;
					if (center2PlaneDistance<lengthThreshold&&plane1.block(0,0,3,1).dot(mainPlanes[j1].block(0,0,3,1))>angleThreshold)
					{
						continue;
					}
					if (0!=PCP::AreTwoPlanesPenetrable(plane1,mainPlanes[j1],tempEigenFourCorner,
						mainBoundingBoxFourCornerPoints[j1],tempKdtree,mainDownSampleKdTree[j1],isPentrable,Parameter.lengthThreshold,10,Parameter.lengthThreshold/2))
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

			MatchedResult MatchedResult;
			MatchedResult.matchedPlanes=matches[index];
			MatchedResult.R=Rs[index];
			MatchedResult.T=Ts[index];
			pMatchedResult->push_back(MatchedResult);

		}
	}
}


int main (int argc, char **argv)
{
	if (argc != 4){
		cout<<"usage: **.exe target_point_cloud source_point_cloud average_space"<<endl;
		return -1;
	}
	//Parameters
	float averageSpace=0.1;
	std::string targetFile,sourceFile;
	targetFile = argv[1];
	cout<<"target point_cloud: "<<targetFile<<endl;

	sourceFile = argv[2];
	cout<<"source point_cloud: "<<sourceFile<<endl;

	std::stringstream ss;
	ss<<argv[3];
	ss>>averageSpace;
	cout<<"averageSpace: "<<averageSpace<<endl;

	char szPath[MAX_PATH] = "";
	char drive[MAX_PATH] = "";
	char dir[MAX_PATH] = "";
	char fileName[MAX_PATH] = "";
	char ext[MAX_PATH] = "";
	_splitpath(targetFile.c_str(), drive, dir, fileName, ext);
	string targetFileName = fileName;
	string targetFilePath = std::string(szPath) + std::string(dir);
	if (0!=string(ext).compare(".bpn")){
		cout<< "only support .bpn file"<<endl;
		return -1;
	}
	_splitpath(sourceFile.c_str(), drive, dir, fileName, ext);
	string sourceFileName = fileName;
	string sourceFilePath = std::string(szPath) + std::string(dir);
	if (0!=string(ext).compare(".bpn")){
		cout<< "only support .bpn file"<<endl;
		return -1;
	}

	float downSampleDistance=averageSpace*4;
	float minLineConfidence=1.0;
	int   minSupportNumOfRansacLine=50;
	float lengthThreshold=averageSpace*5;
	float angleThreshold=5.0/180*M_PI;
	float cosAngleThreshold=cos(angleThreshold);
	float faceMatchesWeight=0.2;
	float inlierDistanceOfFittingLine=averageSpace*2;
	int   maxCandidateResultNum=200;
	int   maxKdtreeNeighbor=0;
	float scale=lengthThreshold/cos(M_PI_2-angleThreshold);

	std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>> mainPlanes;
	std::vector<std::vector<int>> supportMainPlaneIndex;
	pcl::PointCloud<pcl::PointXYZ>::Ptr mainPoints(new pcl::PointCloud<pcl::PointXYZ>);
	if (0!=PCP::load_bpn(targetFile,mainPoints))
	{
		cout<<"load target pointCloud error"<<endl;
		return 0;
	}
	if (0!=PCP::load_plane(drive+targetFilePath+"\\"+targetFileName+"_plane.txt",mainPlanes))
	{
		cout<<"load main plane error"<<endl;
		return 0;
	}
	if (0!=PCP::load_support_points_index(drive+targetFilePath+"\\"+targetFileName+"_planeIndex.txt",supportMainPlaneIndex))
	{
		cout<<"load main support point index error"<<endl;
		return 0;
	}
	// normilize plane Parameters, px-d=0, d>0, represent the length from origin of coordinates to plane
	for (size_t i=0;i<mainPlanes.size();i++)
	{
		double norm=sqrt(mainPlanes[i](0)*mainPlanes[i](0)+mainPlanes[i](1)*mainPlanes[i](1)+mainPlanes[i](2)*mainPlanes[i](2));
		mainPlanes[i]=mainPlanes[i]/norm;
	}

	INIT_RUN_TIME
	//compute the bounding box
	double width,height,depth;
	Eigen::Vector3f mainBoundingCenter;
	pcl::PointCloud<pcl::PointXYZ>::Ptr downSampleMainPoints(new pcl::PointCloud<pcl::PointXYZ>);
	PCP::DownSamplePointCloud(mainPoints,downSampleMainPoints,downSampleDistance,downSampleDistance,downSampleDistance);
	//cout<<"first_downsample"<<endl;
	if (0!=PCP::ComputeBoundingBox(downSampleMainPoints,mainBoundingCenter,width,height,depth))
	{
		return -1;
	}
	double radius=MAX(MAX(width,height),depth)/2;

	//compute the bounding box for each plane
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>downSampleMainPlanePoints(mainPlanes.size());
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>mainBoundingBoxCornerPoints(mainPlanes.size());
	std::vector<std::vector<Eigen::Vector3f>>mainBoundingBoxFourCornerPoints(mainPlanes.size());
	std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr>mainDownSampleKdTree(mainPlanes.size());
	std::vector<Eigen::Vector3f> mainBoundingBoxCenterForEachPlane(mainPlanes.size());
	std::vector<float> mainBoundingBoxRadiusForEachPlane(mainPlanes.size());
	for (size_t i=0;i<mainPlanes.size();i++)
	{
		downSampleMainPlanePoints[i]=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		mainBoundingBoxCornerPoints[i]=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr tempPoints1(new pcl::PointCloud<pcl::PointXYZ>); 
		tempPoints1->reserve(supportMainPlaneIndex[i].size());
		for (size_t j=0;j<supportMainPlaneIndex[i].size();j++)
		{
			tempPoints1->push_back((*mainPoints)[supportMainPlaneIndex[i][j]]);
		}
		PCP::DownSamplePointCloud(tempPoints1,downSampleMainPlanePoints[i],downSampleDistance,downSampleDistance,downSampleDistance);
		Eigen::Vector3f centerPoint;
		double width,height,depth;
		PCP::ComputeBoundingBox(downSampleMainPlanePoints[i],centerPoint,width,height,depth,&(*mainBoundingBoxCornerPoints[i]));
		int start=0,end=4;
		PCP::ProjectPoints2Plane(*mainBoundingBoxCornerPoints[i],mainPlanes[i],mainBoundingBoxFourCornerPoints[i],&start,&end);
		mainBoundingBoxCenterForEachPlane[i]=(mainBoundingBoxFourCornerPoints[i][0]+mainBoundingBoxFourCornerPoints[i][2])/2;
		mainBoundingBoxRadiusForEachPlane[i]=(mainBoundingBoxFourCornerPoints[i][0]-mainBoundingBoxFourCornerPoints[i][2]).norm()/2;
		//construct kdTree;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tempKdTree(new pcl::search::KdTree<pcl::PointXYZ>);
		tempKdTree->setInputCloud(downSampleMainPlanePoints[i]);
		mainDownSampleKdTree[i]=tempKdTree;
	}

	// compute intersection lines
	int mainPlanesNum=mainPlanes.size();
	std::vector<PCP::INTERSECTION_LINE> mainIntersectionLines;
	mainIntersectionLines.reserve(mainPlanesNum*mainPlanesNum/2);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr downsampleKdtree(new pcl::search::KdTree<pcl::PointXYZ>);

	for (size_t i=0;i<mainPlanesNum;i++)
	{
		for (size_t j=i+1;j<mainPlanesNum;j++)
		{
			PCP::INTERSECTION_LINE tempIntersect;
			if (0!=PCP::ComputeIntersectionLineOfTwoPlanes(mainPlanes[i],mainPlanes[j],tempIntersect.lineVec,tempIntersect.linePoint))
			{
				continue;
			}
			//filter out lines far away from the center
			Eigen::Vector3f tempVector=tempIntersect.linePoint-mainBoundingCenter;
			double distance=sqrt(tempVector.squaredNorm()-std::pow(tempVector.dot(tempIntersect.lineVec),2));
			if (distance>radius)
			{
				continue;
			}
			//compute the confidence for each line and filter out the lines with low confidence 
			downsampleKdtree->setInputCloud(downSampleMainPlanePoints[i]);
			tempIntersect.distance.resize(2);
			if (0!=PCP::ComputeMeanDistanceOfLine2Plane(tempIntersect,*mainBoundingBoxCornerPoints[i],
				downsampleKdtree,tempIntersect.distance[0],0.5))
			{
				continue;
			}
			downsampleKdtree->setInputCloud(downSampleMainPlanePoints[j]);
			if (0!=PCP::ComputeMeanDistanceOfLine2Plane(tempIntersect,*mainBoundingBoxCornerPoints[j],
				downsampleKdtree,tempIntersect.distance[1],0.5))
			{
				continue;
			}
			tempIntersect.confidence.push_back(downSampleMainPlanePoints[i]->size()*downSampleDistance*downSampleDistance/tempIntersect.distance[0]);
			tempIntersect.confidence.push_back(downSampleMainPlanePoints[j]->size()*downSampleDistance*downSampleDistance/tempIntersect.distance[1]);
			if (MIN(tempIntersect.confidence[0],tempIntersect.confidence[1])<minLineConfidence)
			{
				//continue;
			}
			tempIntersect.supportPlanes.push_back(i);
			tempIntersect.supportPlanes.push_back(j);
			if (0!=ComputeAngleOfTwo3DVector(mainPlanes[i],mainPlanes[j],tempIntersect.planeAngle))
			{
				//continue;
			}
			tempIntersect.isIntersectionLine=true;
			mainIntersectionLines.push_back(tempIntersect);
		}
	}
	//cout<<"mainIntersectionLines:"<<mainIntersectionLines.size()<<endl;

	std::vector<int> boundaryPointsIndex,filteredBoundaryPointsIndex;
	//PCP::DetectBoundaryPoints(mainPoints,boundaryPointsIndex,0.35,0.35);
	filteredBoundaryPointsIndex=boundaryPointsIndex;
	//PCP::FilterPoints(mainPoints,boundaryPointsIndex,pcl::PointXYZ(0,0,0),filteredBoundaryPointsIndex,true);
	//assign each boundary point to a plane
	std::vector<int> pointToPlaneIndex(mainPoints->size(),-1);
	for (size_t i=0;i<supportMainPlaneIndex.size();i++)
	{
		std::vector<int>& current=supportMainPlaneIndex[i];
		for (size_t j=0;j<supportMainPlaneIndex[i].size();j++)
		{
			pointToPlaneIndex[current[j]]=i;
		}
	}
	std::vector<std::vector<int>> boundaryPointsOfEachPlane(supportMainPlaneIndex.size());
	for (size_t i=0;i<filteredBoundaryPointsIndex.size();i++)
	{
		if (pointToPlaneIndex[filteredBoundaryPointsIndex[i]]<0)
		{
			continue;
		}
		boundaryPointsOfEachPlane[pointToPlaneIndex[filteredBoundaryPointsIndex[i]]].push_back(filteredBoundaryPointsIndex[i]);
	}


	std::vector<PCP::INTERSECTION_LINE> mainBoundaryIntersectionLines;
	for (size_t i=0;i<boundaryPointsOfEachPlane.size();i++)
	{
		std::vector<std::vector<int>> extractLines;
		PCP::RansacExtract3Dline(*mainPoints,boundaryPointsOfEachPlane[i],extractLines,20,
			inlierDistanceOfFittingLine,minSupportNumOfRansacLine,100);
		for (size_t j=0;j<extractLines.size();j++)
		{
			std::vector<cv::Point3f> currentLinePoints(extractLines[j].size());
			for (size_t k=0;k<extractLines[j].size();k++)
			{
				pcl::PointXYZ &p=(*mainPoints)[extractLines[j][k]];
				currentLinePoints[k].x=p.x;
				currentLinePoints[k].y=p.y;
				currentLinePoints[k].z=p.z;
			}
			cv::Vec6f line;
			if (0!=Fit3DLine(currentLinePoints,line))
			{
				continue;
			}
			Eigen::Vector3f lineVec(line[0],line[1],line[2]);
			Eigen::Vector3f start(line[3],line[4],line[5]);
			Eigen::Vector3f end=start+lineVec;
			pcl::PointCloud<pcl::PointXYZ> startEndPoint;
			startEndPoint.reserve(2);
			startEndPoint.push_back(pcl::PointXYZ(start[0],start[1],start[2]));
			startEndPoint.push_back(pcl::PointXYZ(end[0],end[1],end[2]));
			std::vector<Eigen::Vector3f> projectStartEnd;
			if (0!=PCP::ProjectPoints2Plane(startEndPoint,mainPlanes[i],projectStartEnd))
			{
				continue;
			}
			PCP::INTERSECTION_LINE boundaIntersectionLine;
			boundaIntersectionLine.lineVec=(projectStartEnd[1]-projectStartEnd[0]);
			boundaIntersectionLine.lineVec.normalize();
			boundaIntersectionLine.linePoint=projectStartEnd[0];
			boundaIntersectionLine.supportPlanes.push_back(i);
			boundaIntersectionLine.isIntersectionLine=false;
			mainBoundaryIntersectionLines.push_back(boundaIntersectionLine);
		}
	}

	size_t mainBoundaryIntersectionLinesNum=mainBoundaryIntersectionLines.size();
	//cout<<"mainBoundaryIntersectionLinesNum:"<<mainBoundaryIntersectionLinesNum<<endl;

	//copy
	std::copy(mainBoundaryIntersectionLines.begin(),mainBoundaryIntersectionLines.end(),
		std::back_inserter(mainIntersectionLines));

	size_t mainLinesNum=mainIntersectionLines.size();
	//cout<<"totalLinesNum:"<<mainLinesNum<<endl;

	std::vector<KdTreeSearchNDim<Eigen::VectorXf,8>> kdtrees8;
	std::vector<KdTreeSearchNDim<Eigen::VectorXf,6>> kdtrees6;
	std::vector<KdTreeSearchNDim<Eigen::VectorXf,4>> kdtrees4;
	std::vector<std::vector<PCP::PAIRLINE>> mainLinesInformation;
	//cout<<"pairlines"<<endl;
	PCP::ConstructPairLinesKdTree(mainIntersectionLines,mainPlanes,kdtrees8,kdtrees6,kdtrees4,mainLinesInformation,scale);
	//cout<<"pairlinesend"<<endl;
	//compute length of each pair of lines
	std::vector<std::vector<NearstPointsTwoLine>> mainLineLength(mainLinesNum);
	for (size_t i=0;i<mainLinesNum;i++)
	{
		mainLineLength[i].resize(mainLinesNum);
		for (int j=0;j<mainLinesNum;j++)
		{
			if (i>j)
			{
				mainLineLength[i][j].points1=mainLineLength[j][i].points2;
				mainLineLength[i][j].points2=mainLineLength[j][i].points1;
				mainLineLength[i][j].length=mainLineLength[j][i].length;
				mainLineLength[i][j].angle=mainLineLength[j][i].angle;
				mainLineLength[i][j].minAngle=mainLineLength[j][i].minAngle;
			}
			else if (i<j)
			{
				if (0!=PCP::ComputeNearstTwoPointsOfTwo3DLine(mainIntersectionLines[i].lineVec,mainIntersectionLines[i].linePoint,
					mainIntersectionLines[j].lineVec,mainIntersectionLines[j].linePoint,mainLineLength[i][j].points1,
					mainLineLength[i][j].points2,mainLineLength[i][j].length))
				{
					mainLineLength[i][j].length=-1;
				}
				ComputeAngleOfTwo3DVector(mainIntersectionLines[i].lineVec,mainIntersectionLines[j].lineVec,
					mainLineLength[i][j].angle);
				if (mainLineLength[i][j].angle>M_PI_2)
				{
					mainLineLength[i][j].minAngle=M_PI-mainLineLength[i][j].angle;
				}
			}
		}
	}
	ofstream fout;
	fout.open(drive+targetFilePath+"\\transformation.txt");
	if (!fout.is_open())
	{
		cout<<"can not open file"<<endl;
		return -1;
	}
	//fout<<2<<endl;
	//fout<<targetFileName+".bpn"<<endl;
	//fout<<Eigen::Matrix4f::Identity()<<endl;

	RUN_TIME
	for (int partNum=0;partNum<=0;partNum++)
	{
		//fout<<sourceFileName+".bpn"<<endl;
		std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > planes;
		std::vector<std::vector<int>> supportPlaneIndex;
		pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);

		if (0!=PCP::load_bpn(sourceFile,points))
		{
			cout<<"load source pointCloud error"<<endl;
			return 0;
		}
		if (0!=PCP::load_plane(drive+sourceFilePath+"\\"+sourceFileName+"_plane.txt",planes))
		{
			cout<<"load source plane error"<<endl;
			return 0;
		}
		if (0!=PCP::load_support_points_index(drive+sourceFilePath+"\\"+sourceFileName+"_planeIndex.txt",supportPlaneIndex))
		{
			cout<<"load main support point index error"<<endl;
			return 0;
		}
		// normilize plane Parameters, px-d=0, d>0, represent the length from origin of coordinates to plane
		for (size_t i=0;i<planes.size();i++)
		{
			double norm=sqrt(planes[i](0)*planes[i](0)+planes[i](1)*planes[i](1)+planes[i](2)*planes[i](2));
			planes[i]=planes[i]/norm;
		}
		size_t currentPlanesNum=planes.size();

		//compute the bounding box
		double currentWidth,currentHeight,currentDepth;
		Eigen::Vector3f currentBoundingCenter;
		pcl::PointCloud<pcl::PointXYZ>::Ptr downSamplePoints(new pcl::PointCloud<pcl::PointXYZ>);
		PCP::DownSamplePointCloud(points,downSamplePoints,downSampleDistance,downSampleDistance,downSampleDistance);
		if (0!=PCP::ComputeBoundingBox(downSamplePoints,currentBoundingCenter,currentWidth,currentHeight,currentDepth))
		{
			return -1;
		}
		double currentRadius=MAX(MAX(currentWidth,currentHeight),currentDepth)/2;

		//compute the bounding box for each plane
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>downSampleCurrentPlanePoints(planes.size());
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>currentBoundingBoxCornerPoints(planes.size());
		std::vector<std::vector<Eigen::Vector3f>>currentBoundingBoxFourCornerPoints(planes.size());
		std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr>currentDownSampleKdTree(planes.size());
		std::vector<Eigen::Vector3f> currentBoundingBoxCenterForEachPlane(planes.size());
		std::vector<float> currentBoundingBoxRadiusForEachPlane(planes.size());
		for (size_t i=0;i<planes.size();i++)
		{
			downSampleCurrentPlanePoints[i]=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
			currentBoundingBoxCornerPoints[i]=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr tempPoints1(new pcl::PointCloud<pcl::PointXYZ>); 
			tempPoints1->reserve(supportPlaneIndex[i].size());
			for (size_t j=0;j<supportPlaneIndex[i].size();j++)
			{
				tempPoints1->push_back((*points)[supportPlaneIndex[i][j]]);
			}
			PCP::DownSamplePointCloud(tempPoints1,downSampleCurrentPlanePoints[i],
				downSampleDistance,downSampleDistance,downSampleDistance);
			Eigen::Vector3f centerPoint;
			double width,height,depth;
			PCP::ComputeBoundingBox(downSampleCurrentPlanePoints[i],centerPoint,width,height,depth,&(*currentBoundingBoxCornerPoints[i]));
			int start=0,end=4;
			PCP::ProjectPoints2Plane(*currentBoundingBoxCornerPoints[i],planes[i],currentBoundingBoxFourCornerPoints[i],&start,&end);
			currentBoundingBoxCenterForEachPlane[i]=(currentBoundingBoxFourCornerPoints[i][0]+currentBoundingBoxFourCornerPoints[i][2])/2;
			currentBoundingBoxRadiusForEachPlane[i]=(currentBoundingBoxFourCornerPoints[i][0]-currentBoundingBoxFourCornerPoints[i][2]).norm()/2;
			//construct kdTree;
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tempKdTree(new pcl::search::KdTree<pcl::PointXYZ>);
			tempKdTree->setInputCloud(downSampleCurrentPlanePoints[i]);
			currentDownSampleKdTree[i]=tempKdTree;
		}

		//compute intersection lines
		std::vector<PCP::INTERSECTION_LINE> currentIntersectionLines;
		for (size_t i=0;i<currentPlanesNum;i++)
		{
			for (size_t j=i+1;j<currentPlanesNum;j++)
			{
				PCP::INTERSECTION_LINE tempIntersect;
				if (0!=PCP::ComputeIntersectionLineOfTwoPlanes(planes[i],planes[j],tempIntersect.lineVec,tempIntersect.linePoint))
				{
					continue;
				}
				//filter out lines far away from the center
				Eigen::Vector3f tempVector=tempIntersect.linePoint-currentBoundingCenter;
				double distance=sqrt(tempVector.squaredNorm()-std::pow(tempVector.dot(tempIntersect.lineVec),2));
				if (distance>currentRadius)
				{
					continue;
				}
				//compute the confidence for each line and filter out the lines with low confidence 
				downsampleKdtree->setInputCloud(downSampleCurrentPlanePoints[i]);
				tempIntersect.distance.resize(2);
				if (0!=PCP::ComputeMeanDistanceOfLine2Plane(tempIntersect,*currentBoundingBoxCornerPoints[i],
					downsampleKdtree,tempIntersect.distance[0],0.5))
				{
					continue;
				}
				downsampleKdtree->setInputCloud(downSampleCurrentPlanePoints[j]);
				if (0!=PCP::ComputeMeanDistanceOfLine2Plane(tempIntersect,*currentBoundingBoxCornerPoints[j],
					downsampleKdtree,tempIntersect.distance[1],0.5))
				{
					continue;
				}
				tempIntersect.confidence.push_back(downSampleCurrentPlanePoints[i]->size()*downSampleDistance*downSampleDistance/tempIntersect.distance[0]);
				tempIntersect.confidence.push_back(downSampleCurrentPlanePoints[j]->size()*downSampleDistance*downSampleDistance/tempIntersect.distance[1]);
				if (MIN(tempIntersect.confidence[0],tempIntersect.confidence[1])<minLineConfidence)
				{
					//continue;
				}
				tempIntersect.supportPlanes.push_back(i);
				tempIntersect.supportPlanes.push_back(j);
				if (0!=ComputeAngleOfTwo3DVector(planes[i],planes[j],tempIntersect.planeAngle))
				{
					//continue;
				}
				tempIntersect.isIntersectionLine=true;
				currentIntersectionLines.push_back(tempIntersect);
			}
		}

		std::vector<int> currentBoundaryPointsIndex;
		//PCP::DetectBoundaryPoints(points,currentBoundaryPointsIndex,0.35,0.35);
		//assign each boundary point to a plane
		std::vector<int> pointToPlaneIndex(points->size(),-1);
		for (size_t i=0;i<supportPlaneIndex.size();i++)
		{
			std::vector<int>& current=supportPlaneIndex[i];
			for (size_t j=0;j<supportPlaneIndex[i].size();j++)
			{
				pointToPlaneIndex[current[j]]=i;
			}
		}
		std::vector<std::vector<int>> currentBoundaryPointsOfEachPlane(supportPlaneIndex.size());
		for (size_t i=0;i<currentBoundaryPointsIndex.size();i++)
		{
			if (pointToPlaneIndex[currentBoundaryPointsIndex[i]]<0)
			{
				continue;
			}
			currentBoundaryPointsOfEachPlane[pointToPlaneIndex[currentBoundaryPointsIndex[i]]].push_back(currentBoundaryPointsIndex[i]);
		}

		std::vector<PCP::INTERSECTION_LINE> currentBoundaryIntersectionLines;
		for (size_t i=0;i<currentBoundaryPointsOfEachPlane.size();i++)
		{
			std::vector<std::vector<int>> extractLines;
			PCP::RansacExtract3Dline(*points,currentBoundaryPointsOfEachPlane[i],extractLines,20,
				inlierDistanceOfFittingLine,minSupportNumOfRansacLine,100);
			for (size_t j=0;j<extractLines.size();j++)
			{
				std::vector<cv::Point3f> currentLinePoints(extractLines[j].size());
				for (size_t k=0;k<extractLines[j].size();k++)
				{
					pcl::PointXYZ &p=(*points)[extractLines[j][k]];
					currentLinePoints[k].x=p.x;
					currentLinePoints[k].y=p.y;
					currentLinePoints[k].z=p.z;
				}
				cv::Vec6f line;
				if (0!=Fit3DLine(currentLinePoints,line))
				{
					continue;
				}
				Eigen::Vector3f lineVec(line[0],line[1],line[2]);
				Eigen::Vector3f start(line[3],line[4],line[5]);
				Eigen::Vector3f end=start+lineVec;
				pcl::PointCloud<pcl::PointXYZ> startEndPoint;
				startEndPoint.reserve(2);
				startEndPoint.push_back(pcl::PointXYZ(start[0],start[1],start[2]));
				startEndPoint.push_back(pcl::PointXYZ(end[0],end[1],end[2]));
				std::vector<Eigen::Vector3f> projectStartEnd;
				if (0!=PCP::ProjectPoints2Plane(startEndPoint,planes[i],projectStartEnd))
				{
					continue;
				}
				PCP::INTERSECTION_LINE boundaIntersectionLine;
				boundaIntersectionLine.lineVec=(projectStartEnd[1]-projectStartEnd[0]);
				boundaIntersectionLine.lineVec.normalize();
				boundaIntersectionLine.linePoint=projectStartEnd[0];
				boundaIntersectionLine.supportPlanes.push_back(i);
				boundaIntersectionLine.isIntersectionLine=false;
				currentBoundaryIntersectionLines.push_back(boundaIntersectionLine);
			}
		}

		//cout<<"currentIntersectionLines:"<<currentIntersectionLines.size()<<endl;
		//copy
		std::copy(currentBoundaryIntersectionLines.begin(),currentBoundaryIntersectionLines.end(),std::back_inserter(currentIntersectionLines));

		size_t currentLinesNum=currentIntersectionLines.size();
		size_t currentBoundaryIntersectionLinesNum=currentBoundaryIntersectionLines.size();
		//cout<<"currentBoundaryIntersectionLinesNum:"<<currentBoundaryIntersectionLinesNum<<endl;
		//cout<<"totalLinesNum:"<<currentLinesNum<<endl;

		// find initial matches for each line
		std::vector<std::vector<int>> coarseMatches(currentLinesNum);
		std::vector<float> matchesLength;

		//compute lineLength;
		std::vector<std::vector<NearstPointsTwoLine>> currentLineLength(currentLinesNum);
		for (size_t i=0;i<currentLinesNum;i++)
		{
			currentLineLength[i].resize(currentLinesNum);
			for (int j=0;j<currentLinesNum;j++)
			{
				if (i>j)
				{
					currentLineLength[i][j].points1=currentLineLength[j][i].points2;
					currentLineLength[i][j].points2=currentLineLength[j][i].points1;
					currentLineLength[i][j].length=currentLineLength[j][i].length;
					currentLineLength[i][j].angle=currentLineLength[j][i].angle;
					currentLineLength[i][j].minAngle=currentLineLength[j][i].minAngle;
				}
				else if (i<j)
				{
					if (0!=PCP::ComputeNearstTwoPointsOfTwo3DLine(currentIntersectionLines[i].lineVec,currentIntersectionLines[i].linePoint,
						currentIntersectionLines[j].lineVec,currentIntersectionLines[j].linePoint,currentLineLength[i][j].points1,
						currentLineLength[i][j].points2,currentLineLength[i][j].length))
					{
						currentLineLength[i][j].length=-1;
					}
					currentLineLength[i][j].length=currentLineLength[i][j].length/scale;
					ComputeAngleOfTwo3DVector(currentIntersectionLines[i].lineVec,currentIntersectionLines[j].lineVec,
						currentLineLength[i][j].angle);
					currentLineLength[i][j].minAngle=(currentLineLength[i][j].angle>M_PI_2?M_PI-currentLineLength[i][j].angle:currentLineLength[i][j].angle);
				}

			}
		}

		MatchInformation current,main;
		current.boundingCenter=currentBoundingCenter;
		current.pBoundingBoxFourCornerPoints=&currentBoundingBoxFourCornerPoints;
		current.pDownSampleKdTree=&currentDownSampleKdTree;
		current.pDownSamplePlanePoints=&downSampleCurrentPlanePoints;
		current.pIntersectionLine=&currentIntersectionLines;
		current.pModelPlaneCoff=NULL;
		current.pNearstPointsTwoLines=&currentLineLength;
		current.points=downSamplePoints;
		current.originalPoints=points;
		current.pPlanes=&planes;
		current.pSupportPlaneIndex=&supportPlaneIndex;
		current.pBoundingBoxCenterForEachPlane=&currentBoundingBoxCenterForEachPlane;
		current.pBoundingBoxRadiusForEachPlane=&currentBoundingBoxRadiusForEachPlane;

		main.boundingCenter=mainBoundingCenter;
		main.pBoundingBoxFourCornerPoints=&mainBoundingBoxFourCornerPoints;
		main.pDownSampleKdTree=&mainDownSampleKdTree;
		main.pDownSamplePlanePoints=&downSampleMainPlanePoints;
		main.pIntersectionLine=&mainIntersectionLines;
		main.pModelPlaneCoff=NULL;
		main.pNearstPointsTwoLines=&mainLineLength;
		main.points=downSampleMainPoints;
		main.originalPoints=mainPoints;
		main.pPlanes=&mainPlanes;
		main.pSupportPlaneIndex=&supportMainPlaneIndex;
		main.pBoundingBoxCenterForEachPlane=&mainBoundingBoxCenterForEachPlane;
		main.pBoundingBoxRadiusForEachPlane=&mainBoundingBoxRadiusForEachPlane;

		std::vector<std::pair<int,int>> linesToBeMatched;
		linesToBeMatched.reserve(currentLinesNum*currentLinesNum/2);
		float angleThresh=cos(10.0/180*M_PI);
		for (int i=0;i<currentLinesNum;i++)
		{
			for (int j = i+1; j < currentLinesNum; j++)
			{
				if (abs(currentIntersectionLines[i].lineVec.dot(currentIntersectionLines[j].lineVec))>angleThresh)
				{
					continue;
				}
				linesToBeMatched.push_back(std::pair<int,int>(i,j));
			}
		}

		int numThread=1;
		//use multi thread to compute
		//cout<<"Use multi thread? y or n: ";
		char answer='n';
		//cin>>answer;
		if (answer=='y')
		{
			SYSTEM_INFO si;  
			GetSystemInfo(&si);  
			numThread=si.dwNumberOfProcessors; 
		}
		Parameter Parameter;
		Parameter.lengthThreshold=lengthThreshold;
		Parameter.angleThreshold=angleThreshold;
		Parameter.cosAngleThreshold=cosAngleThreshold;
		Parameter.maxCandidateResultNum=int(double(maxCandidateResultNum)/numThread);
		Parameter.mainLinesInformation=&mainLinesInformation;
		Parameter.kdtrees8=&kdtrees8;
		Parameter.kdtrees6=&kdtrees6;
		Parameter.kdtrees4=&kdtrees4;
		Parameter.maxNeighbor=maxKdtreeNeighbor;
		Parameter.maxRadius=radius;
		std::vector<boost::thread> threadPool(numThread);
		std::vector<std::vector<std::pair<int,int>>>templinesTobeMatched(numThread);
		std::vector<std::vector<MatchedResult>> matchedResult(numThread);
		int increment=linesToBeMatched.size()/double(numThread);
		int startIndex;
		startIndex=0;
		//cout<<"total task num: "<<linesToBeMatched.size()<<endl;
		for (int i=0;i<numThread;i++)
		{
			int currentStart=startIndex;
			int currentStop;
			if (i==numThread-1)
			{
				currentStop=linesToBeMatched.size();
			}
			else
			{
				currentStop=startIndex+increment;
			}
			startIndex=currentStop;
			std::copy(linesToBeMatched.begin()+currentStart,linesToBeMatched.begin()+currentStop,std::back_inserter(templinesTobeMatched[i]));
			threadPool[i]=boost::thread(MatchingLines,current,main,templinesTobeMatched[i],
				coarseMatches,&matchedResult[i],Parameter);
			//cout<<"Thread "<<i<<" started"<<endl;
			//cout<<"task:from "<<currentStart<<" to "<<currentStop<<endl;
		}
		for (int i=0;i<numThread;i++)
		{
			threadPool[i].join();
			//cout<<"thread "<<i+1<<" completed"<<endl;
		}
		//cout<<"All threads have exited..."<<endl;

		std::vector<MatchedResult> combinedResults;
		for (int i=0;i<numThread;i++)
		{
			std::copy(matchedResult[i].begin(),matchedResult[i].end(),std::back_inserter(combinedResults));
		}
		int combinedResultsNum=combinedResults.size();
		//cout<<"results num:"<<combinedResultsNum<<endl;
		if (0==combinedResultsNum)
		{
			fout<<Eigen::Matrix4f::Identity()<<endl;
			cout<<"registration error"<<endl;
			continue;
		}
		//visualization
		downsampleKdtree->setInputCloud(downSampleMainPoints);
		std::vector<LENGTHINDEX> overlapLength(combinedResultsNum);
		for (int i=0;i<combinedResultsNum;i++)
		{
			pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformation;
			transformation=pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4::Identity();
			transformation.block(0,0,3,3)=combinedResults[i].R;
			transformation.block(0,3,3,1)=combinedResults[i].T;

			pcl::PointCloud<pcl::PointXYZ>::Ptr transPoints(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::transformPointCloud(*downSamplePoints,*transPoints,transformation);
			Eigen::Vector3f currentCenter=combinedResults[i].R*currentBoundingCenter+combinedResults[i].T;
			pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
			kdtree->setInputCloud(transPoints);
			PCP::ComputeOverlap(kdtree,downsampleKdtree,currentCenter,currentRadius,downSampleDistance,overlapLength[i].length);
			overlapLength[i].index=i;
			overlapLength[i].length=0.2*(combinedResults[i].matchedPlanes.size()/double(currentPlanesNum))+0.8*overlapLength[i].length;
		}
		sort(overlapLength.begin(),overlapLength.end(),myCompareGreater);
		//cout<<"overlapLength:"<<overlapLength[0].length<<endl;
		int index=overlapLength[0].index;
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformation;
		transformation=pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4::Identity();
		transformation.block(0,0,3,3)=combinedResults[index].R;
		transformation.block(0,3,3,1)=combinedResults[index].T;
		cout<<"transformation matrix:"<<endl<<transformation<<endl;
		fout<<transformation<<endl;
		cout<<"result has been saved to "<<drive+targetFilePath+"\\transformation.reg"<<endl;
		RUN_TIME
	}
	fout.close();

	return 0;

}


#endif