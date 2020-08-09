#include "stdafx.h"
#include "pointCloud_processing.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/boundary.h>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>
#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>


namespace PCP
{
float generateNan()//产生NAN
{
	float i=0.0,j=0.0;
	return i/j;
}

template<class _Tp> string num2str(_Tp i)
{
	stringstream ss;
	ss<<i;
	return ss.str();
}
template<class _Tp> _Tp str2num(string str)
{
	_Tp num;
	stringstream ss(s);
	ss>>num;
	return num;
}

int DownSamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &srcPoints,pcl::PointCloud<pcl::PointXYZ>::Ptr &downSamplePoints,
						 float gridX,float gridY,float gridZ)
{
	if (srcPoints->empty()||gridX<=0||gridY<=0||gridZ<=0)
	{
		return -1;
	}
	//////////////////////////////////////////////////////////////////////////
	pcl::VoxelGrid<pcl::PointXYZ> grid; //VoxelGrid 把一个给定的点云，聚集在一个局部的3D网格上,并下采样和滤波点云数据
	grid.setLeafSize (gridX, gridY, gridZ); //设置体元网格的叶子大小
	grid.setInputCloud(srcPoints);
	grid.filter (*downSamplePoints); //下采样和滤波，并存储在src中
	return 0;
}


int ApplyRigidTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr &srcPoints,pcl::PointCloud<pcl::PointXYZ>::Ptr &outPoints,
							 pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4  &transformation,
							 bool isUsingICP, pcl::PointCloud<pcl::PointXYZ>::Ptr &destPoints,
							 pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4  *pOuttransformation,
							 float gridX,float gridY,float gridZ)
{
	if (srcPoints->empty()||gridX<=0||gridY<=0||gridZ<=0)
	{
		return -1;
	}
	if (isUsingICP&&destPoints->empty())
	{
		return -1;
	}
	//////////////////////////////////////////////////////////////////////////
	if (isUsingICP)
	{
		//进行刚体变换
		pcl::PointCloud<pcl::PointXYZ>::Ptr tempPoints(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*srcPoints,*tempPoints,transformation);

		//对输入点云进行下采样
		pcl::VoxelGrid<pcl::PointXYZ> grid; //VoxelGrid 把一个给定的点云，聚集在一个局部的3D网格上,并下采样和滤波点云数据
		pcl::PointCloud<pcl::PointXYZ>::Ptr down_source_points (new pcl::PointCloud<pcl::PointXYZ>); //创建输出/目标点云（指针）
		grid.setLeafSize (gridX, gridY, gridZ); //设置体元网格的叶子大小
		//下采样 源点云
		grid.setInputCloud (tempPoints); //设置输入点云
		grid.filter (*down_source_points); //下采样和滤波，并存储在src中
		//计算输入输出点云的法线信息
		pcl::NormalEstimation<pcl::PointXYZ,pcl::PointNormal> pclNormal;
		pclNormal.setInputCloud(down_source_points);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		pclNormal.setSearchMethod (tree);
		pclNormal.setRadiusSearch (gridX*2);
		//计算输入点云的法向
		pcl::PointCloud<pcl::PointNormal>::Ptr down_source_points_normals (new pcl::PointCloud<pcl::PointNormal>);
		pclNormal.compute (*down_source_points_normals);
		pcl::copyPointCloud(*down_source_points,*down_source_points_normals);
		//计算目标点云的法向
		pcl::PointCloud<pcl::PointNormal>::Ptr down_dest_points_normals (new pcl::PointCloud<pcl::PointNormal>);
		pclNormal.setInputCloud(destPoints);
		pclNormal.compute (*down_dest_points_normals);
		pcl::copyPointCloud(*destPoints,*down_dest_points_normals);
		
		//ICP 配准
		pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp; //创建ICP对象，用于ICP配准
		//采用kdTree进行加速搜索
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree1 (new pcl::search::KdTree<pcl::PointNormal>);
		tree1->setInputCloud(down_source_points_normals);
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud(down_dest_points_normals);
		icp.setSearchMethodSource(tree1);
		icp.setSearchMethodTarget(tree2);
		icp.setMaxCorrespondenceDistance(10);
		icp.setTransformationEpsilon(1e-6);
		icp.setEuclideanFitnessEpsilon(0.001);
		icp.setMaximumIterations(1000);
		icp.setInputCloud(down_source_points_normals); //设置输入点云
		icp.setInputTarget(down_dest_points_normals); //设置目标点云（输入点云进行仿射变换，得到目标点云）
		pcl::PointCloud<pcl::PointNormal> Final; //存储结果
		//进行配准，结果存储在Final中
		icp.align(Final); 
#ifdef _DEBUG
		std::cout << "has converged:" << icp.hasConverged() << " score: " <<
			icp.getFitnessScore() << std::endl;
#endif // _DEBUG
		pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal>::Matrix4 IcpTran=icp.getFinalTransformation();
		//再次进行进行刚体变换
		pcl::transformPointCloud(*tempPoints,*outPoints,IcpTran);
		//*pOuttransformation=IcpTran*transformation;
		if (pOuttransformation)
		{
			*pOuttransformation=IcpTran;
		}
	}
	else
	{
		//进行刚体变换
		pcl::transformPointCloud(*srcPoints,*outPoints,transformation);
	}

	return 0;
}


int ParseVGFile(const std::string &fileName,pcl::PointCloud<pcl::PointXYZ>::Ptr points,
				std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> >&plane,
				std::vector<std::vector<int>>&supportPlanePointsIndex)
{
	std::ifstream fin;
	fin.open(fileName);
	if (fin.fail())
	{
		return -1;
	}
	points->clear();
	plane.clear();
	supportPlanePointsIndex.clear();
	std::string param;
	while (!fin.eof())
	{
		if (!(fin>>param))
		{
			return -2;
		}
		if ("num_points:"==param)
		{
			int pointsNum;
			if (!(fin>>pointsNum))
			{
				return -2;
			}
			points->reserve(pointsNum);
			for (int i=0;i<pointsNum;i++)
			{
				float x,y,z;
				if (fin>>x>>y>>z)
				{
					points->push_back(pcl::PointXYZ(x,y,z));
				}
			}
		}
		else if ("num_groups:"==param)
		{
			int groupNum;
			if (!(fin>>groupNum))
			{
				return -2;
			}
			plane.resize(groupNum);
			supportPlanePointsIndex.resize(groupNum);
			for (int i=0;i<groupNum;i++)
			{
				if (!(fin>>param))
				{
					return -2;
				}
				if ("group_type:"!=param)
				{
					return -2;
				}
				int groupType;
				if (fin>>groupType)
				{
					int numParameters;
					switch (groupType)
					{
					case 0:
						if (!(fin>>param))
						{
							return -2;
						}
						if ("num_group_parameters:"!=param)
						{
							return -2;
						}
						fin>>numParameters;
						if (!(fin>>param))
						{
							return -2;
						}
						if ("group_parameters:"!=param)
						{
							return -2;
						}
						for (int j=0;j<numParameters;j++)
						{
							fin>>plane[i][j];
						}
						fin.ignore( numeric_limits<streamsize>::max(),'\n');
						fin.ignore( numeric_limits<streamsize>::max(),'\n');
						fin.ignore( numeric_limits<streamsize>::max(),'\n');
						if (!(fin>>param))
						{
							return -2;
						}
						if ("group_num_point:"!=param)
						{
							return -2;
						}
						int currentNum;
						fin>>currentNum;
						supportPlanePointsIndex[i].reserve(currentNum);
						for (int j=0;j<currentNum;j++)
						{
							int index;
							fin>>index;
							supportPlanePointsIndex[i].push_back(index);
						}
						fin.ignore( numeric_limits<streamsize>::max(),'\n');
						fin.ignore( numeric_limits<streamsize>::max(),'\n');
						break;
					case 1:
						break;
					case 2:
						break;
					case 3:
						break;
					case 4:
						break;
					default:
						return -2;

					}

				}
			}
		}

	}

	return 0;
}


int ComputeIntersectionPointOfThreePlanes(const Eigen::Vector4f &p1,
										  const Eigen::Vector4f &p2,
										  const Eigen::Vector4f &p3,
										  pcl::PointXYZ &point,
										  float *pCondNum)
{
	Eigen::Vector4f plane1=p1;
	Eigen::Vector4f plane2=p2;
	Eigen::Vector4f plane3=p3;
	//normalize
	plane1.normalize();
	plane2.normalize();
	plane3.normalize();
	// if the plane norms are in the same plane, then there is no intersection point;
	if (plane1==plane2||plane1==plane3||plane2==plane3)
	{
		return -1;
	}

 	if (plane1.cross3(plane2).dot(plane3)==0.0f)
 	{
 		return -1;
 	}
	Eigen::Matrix3f A;
	Eigen::Matrix<float,3,1> B;

	A(0,0)=plane1[0];
	A(0,1)=plane1[1];
	A(0,2)=plane1[2];
	A(1,0)=plane2[0];
	A(1,1)=plane2[1];
	A(1,2)=plane2[2];
	A(2,0)=plane3[0];
	A(2,1)=plane3[1];
	A(2,2)=plane3[2];
	B(0)=-plane1[3];
	B(1)=-plane2[3];
	B(2)=-plane3[3];
	Eigen::Vector3f result=A.inverse()*B;
	point.x=result(0);
	point.y=result(1);
	point.z=result(2);
	if (pCondNum)
	{
		*pCondNum=A.norm()*A.inverse().norm();
	}
	return 0;
}

int ComputeIntersectionLineOfTwoPlanes(const Eigen::Vector4f &plane1,
										const Eigen::Vector4f &plane2,
										Eigen::Vector3f &line)
{
	Eigen::Vector3f p1(plane1(0),plane1(1),plane1(2));
	Eigen::Vector3f p2(plane2(0),plane2(1),plane2(2));
	p1.normalize();
	p2.normalize();
	if (abs(p1.dot(p2))>0.95)
	{
		return -1;
	}
	line=p1.cross(p2);
	line.normalize();
	return 0;
}

int ComputeIntersectionLineOfTwoPlanes(const Eigen::Vector4f &plane1,
									   const Eigen::Vector4f &plane2,
									   Eigen::Vector3f &lineVec,
									   Eigen::Vector3f &linePoint)
{
	Eigen::Vector3f p1(plane1(0),plane1(1),plane1(2));
	Eigen::Vector3f p2(plane2(0),plane2(1),plane2(2));
	p1.normalize();
	p2.normalize();
	if (abs(p1.dot(p2))>0.95)
	{
		return -1;
	}
	lineVec=p1.cross(p2);
	lineVec.normalize();
	cv::Mat A(2,2,CV_64F,cv::Scalar(0));
	cv::Mat B(2,1,CV_64F,cv::Scalar(0));
	B.at<double>(0)=-plane1(3);
	B.at<double>(1)=-plane2(3);
	cv::Mat result;
	if (abs(plane1(0)*plane2(1)-plane2(0)*plane1(1))>1e-6)
	{
		A.at<double>(0,0)=plane1(0);
		A.at<double>(0,1)=plane1(1);
		A.at<double>(1,0)=plane2(0);
		A.at<double>(1,1)=plane2(1);
		result=A.inv()*B;
		linePoint(0)=result.at<double>(0);
		linePoint(1)=result.at<double>(1);
		linePoint(2)=0;
	}
	else if (abs(plane1(0)*plane2(2)-plane2(0)*plane1(2))>1e-6)
	{
		A.at<double>(0,0)=plane1(0);
		A.at<double>(0,1)=plane1(2);
		A.at<double>(1,0)=plane2(0);
		A.at<double>(1,1)=plane2(2);
		result=A.inv()*B;
		linePoint(0)=result.at<double>(0);
		linePoint(2)=result.at<double>(1);
		linePoint(1)=0;
		
	}
	else if (abs(plane1(1)*plane2(2)-plane2(1)*plane1(2))>1e-6)
	{
		A.at<double>(0,0)=plane1(1);
		A.at<double>(0,1)=plane1(2);
		A.at<double>(1,0)=plane2(1);
		A.at<double>(1,1)=plane2(2);
		result=A.inv()*B;
		linePoint(1)=result.at<double>(0);
		linePoint(2)=result.at<double>(1);
		linePoint(0)=0;
	}
	else
	{
		return -1;
	}
// 	cout<<plane1(0)*linePoint(0)+plane1(1)*linePoint(1)+plane1(2)*linePoint(2)+plane1(3)<<endl;
// 	cout<<plane2(0)*linePoint(0)+plane2(1)*linePoint(1)+plane2(2)*linePoint(2)+plane2(3)<<endl;

	return 0;
}



int ComputeBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr points,
					   Eigen::Vector3f &centerPoint,double &width,double &height,double &depth,
					   pcl::PointCloud<pcl::PointXYZ>*pCornerPoints,
					   Eigen::Matrix3f *pR, Eigen::Vector3f *pT)
{
	if (points->empty())
	{
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
	p2w.block<3,3>(0,0) = eigDx.transpose();
	p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
	pcl::PointCloud<pcl::PointXYZ> cPoints;
	pcl::transformPointCloud(*points, cPoints, p2w);

	pcl::PointXYZ min_pt, max_pt;
	pcl::getMinMax3D(cPoints, min_pt, max_pt);
	const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

	centerPoint = eigDx*mean_diag + centroid.head<3>();
	width=max_pt.x - min_pt.x;
	depth=max_pt.y - min_pt.y;
	height=max_pt.z - min_pt.z;

	//compute cornerPoints
	if (pCornerPoints)
	{
		pCornerPoints->clear();
		pcl::PointCloud<pcl::PointXYZ> corners;
		float x=min_pt.x;
		float y=min_pt.y;
		float z=min_pt.z;
		corners.push_back(min_pt);
		corners.push_back(pcl::PointXYZ(x,y+depth,z));
		corners.push_back(pcl::PointXYZ(x,y+depth,z+height));
		corners.push_back(pcl::PointXYZ(x,y,z+height));
		corners.push_back(pcl::PointXYZ(x+width,y,z+height));
		corners.push_back(pcl::PointXYZ(x+width,y+depth,z));
		corners.push_back(pcl::PointXYZ(x+width,y,z));
		corners.push_back(pcl::PointXYZ(x+width,y+depth,z+height));
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformation;
		transformation=pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4::Identity();
		transformation.block(0,0,3,3)=eigDx;
		transformation.block(0,3,3,1)=centroid.head<3>();
		pcl::transformPointCloud(corners, *pCornerPoints, transformation);
	}
	if (pR)
	{
		*pR=eigDx;
	}
	if (pT)
	{
		*pT=centroid.head<3>();
	}

	return 0;
}

int RansacEstimateRigidTransformation(pcl::PointCloud<pcl::PointXYZ>& source,pcl::PointCloud<pcl::PointXYZ>& target,
									  Eigen::Matrix4f &transformation, int maxIter,double lengthThreshold,
									  std::vector<int>&innerPointsIndex)
{
	if (source.empty()||target.empty()||source.size()!=target.size())
	{
		return -1;
	}
	//randomly select 3 points to compute rigid transformation
	int pointsNum=source.size();
	cv::Mat matrix(3,4,CV_32F,cv::Scalar(1));
	std::vector<std::vector<int>>innerPoints(maxIter);
	std::vector<LENGTHINDEX>innerPointsSize(maxIter);
	srand(UINT(time(0)));
	pcl::PointCloud<pcl::PointXYZ> sourceSamplePoints;
	pcl::PointCloud<pcl::PointXYZ> targetSamplePoints;
	sourceSamplePoints.resize(3);
	targetSamplePoints.resize(3);
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> TESVD; 
	for (int i=0;i<maxIter;i++)
	{
		for (int j=0;j<sourceSamplePoints.size();j++)
		{
			int index=rand()%pointsNum;
			sourceSamplePoints[j]=source[index];
			targetSamplePoints[j]=target[index];
		}
		//利用SVD方法求解变换矩阵  
 		TESVD.estimateRigidTransformation (sourceSamplePoints,targetSamplePoints,transformation);  
		//统计内点个数
		pcl::PointCloud<pcl::PointXYZ>tempPoints;
		pcl::transformPointCloud(source,tempPoints,transformation);
		for (size_t j=0;j<tempPoints.size();j++)
		{
			if (computeLengthOfTwo3DPoint(target[j],tempPoints[j])>lengthThreshold)
			{
				continue;
			}
			innerPoints[i].push_back(j);
		}
		LENGTHINDEX tempSize;
		tempSize.length=float(innerPoints[i].size());
		tempSize.index=i;
		innerPointsSize[i]=tempSize;
	}
	partial_sort(innerPointsSize.begin(),innerPointsSize.begin()+1,innerPointsSize.end(),myCompareGreater);//找到内点最多的
	int currentIndex=innerPointsSize[0].index;
	sourceSamplePoints.clear();
	sourceSamplePoints.reserve(innerPoints[currentIndex].size());
	targetSamplePoints.clear();
	targetSamplePoints.reserve(innerPoints[currentIndex].size());
	for (int i=0;i<innerPoints[currentIndex].size();i++)
	{
		sourceSamplePoints.push_back(source[innerPoints[currentIndex][i]]);
		targetSamplePoints.push_back(target[innerPoints[currentIndex][i]]);
	}
	TESVD.estimateRigidTransformation (sourceSamplePoints,targetSamplePoints,transformation);  
	innerPointsIndex=innerPoints[currentIndex];
	return 0;
}


//首先将直线方程化为对称式，得到其方向向量n1=（a1,b1,c1),n2=(a2,b2,c2).
//
//再将两向量叉乘得到其公垂向量N=（x,y,z）,在两直线上分别选取点A,B(任意)，得到向量AB， 
//求向量AB在向量N方向的投影即为两异面直线间的距离了（就是最短距离啦）。
//
//最短距离的求法：d=|向量N*向量AB|/|向量N|（上面是两向量的数量积，下面是取模）。
//
//设交点为C,D，带入公垂线N的对称式中，又因为C,D两点分别满足一开始的直线方程，
//所以得到关于C（或D）的两个连等方程，分别解出来就好了！
int ComputeNearstTwoPointsOfTwo3DLine(Eigen::Vector3f &line1Vec,Eigen::Vector3f &line1Point,
									  Eigen::Vector3f &line2Vec,Eigen::Vector3f &line2Point,
									  Eigen::Vector3f &point1,  Eigen::Vector3f &point2,
									  double &minLength)
{
	line1Vec.normalize();
	line2Vec.normalize();
	if (line1Vec==line2Vec)
	{
		return -1;
	}
	//////////////////////////////////////////////////////////////////////////
	Eigen::Vector3f directionVector=line1Vec.cross(line2Vec);
	directionVector.normalize();
	Eigen::Vector3f AB=line2Point-line1Point;
	//compute minLength
	minLength=abs(AB.dot(directionVector));
	//compute nearstPoint
	cv::Mat A(9,9,CV_32F,cv::Scalar(0));
	cv::Mat B(9,1,CV_32F,cv::Scalar(0));
	cv::Mat X;
	std::vector<float*>ptrA(9),ptrB(9);
	for (int i=0;i<9;i++)
	{
		ptrA[i]=A.ptr<float>(i);
		ptrB[i]=B.ptr<float>(i);
	}
	ptrA[0][0]=1;
	ptrA[0][3]=-line1Vec(0);
	ptrA[1][1]=1;
	ptrA[1][3]=-line1Vec(1);
	ptrA[2][2]=1;
	ptrA[2][3]=-line1Vec(2);
	ptrA[3][4]=1;
	ptrA[3][7]=-line2Vec(0);
	ptrA[4][5]=1;
	ptrA[4][7]=-line2Vec(1);
	ptrA[5][6]=1;
	ptrA[5][7]=-line2Vec(2);
	ptrA[6][0]=-1;
	ptrA[6][4]=1;
	ptrA[6][8]=-directionVector(0);
	ptrA[7][1]=-1;
	ptrA[7][5]=1;
	ptrA[7][8]=-directionVector(1);
	ptrA[8][2]=-1;
	ptrA[8][6]=1;
	ptrA[8][8]=-directionVector(2);
	ptrB[0][0]=line1Point(0);
	ptrB[1][0]=line1Point(1);
	ptrB[2][0]=line1Point(2);
	ptrB[3][0]=line2Point(0);
	ptrB[4][0]=line2Point(1);
	ptrB[5][0]=line2Point(2);

	cv::solve(A,B,X,cv::DECOMP_SVD);
	point1(0)=X.at<float>(0);
	point1(1)=X.at<float>(1);
	point1(2)=X.at<float>(2);
	point2(0)=X.at<float>(4);
	point2(1)=X.at<float>(5);
	point2(2)=X.at<float>(6);
	minLength=(point1-point2).norm();
	//test
// 	Eigen::Vector3f temp1=point1-line1Point;
// 	Eigen::Vector3f temp2=point2-line2Point;
// 	temp1.normalize();
// 	temp2.normalize();
// 	cout<<temp1<<" "<<line1Vec<<endl;
// 	cout<<temp2<<" "<<line2Vec<<endl;
// 	cout<<minLength<<endl;
	return 0;
}

int ComputeTransformationUsingTwoVecAndOnePoint(Eigen::Vector3f &sourceVec1,Eigen::Vector3f &sourceVec2,
												Eigen::Vector3f &destVec1,Eigen::Vector3f &destVec2,
												Eigen::Vector3f &sourcePoint,Eigen::Vector3f &targetPoint,
												Eigen::Matrix3f &R,Eigen::Vector3f &T)
{
// 	Eigen::Vector3f temp1=sourceVec1.cross(sourceVec2);
// 	Eigen::Vector3f temp2=destVec1.cross(destVec2);
// 	Eigen::MatrixXf mat=destVec1*sourceVec1.transpose()+destVec2*sourceVec2.transpose()+temp2*temp1.transpose();
// 	Eigen::JacobiSVD<Eigen::MatrixXf> svd(mat, Eigen::ComputeFullV | Eigen::ComputeFullU); // ComputeThinU | ComputeThinV  
// 	Eigen::MatrixXf A = svd.singularValues();  
// 	Eigen::MatrixXf U = svd.matrixU();  
// 	Eigen::MatrixXf V = svd.matrixV();
// 	R=V*U.transpose();
// 	T=targetPoint-R*sourcePoint;

	//test 
// 	cout<<"R: "<<R<<endl;
// 	cout<<"T: "<<T<<endl;
// 	cout<<"destVec1:"<<destVec1<<endl;
// 	cout<<R*sourceVec1<<endl;
// 	cout<<"destVec2:"<<destVec2<<endl;
// 	cout<<R*sourceVec2<<endl;
// 	cout<<targetPoint<<endl;
// 	cout<<R*sourcePoint+T<<endl;



	//利用SVD方法求解变换矩阵  
	pcl::PointCloud<pcl::PointXYZ> src,dst;
	Eigen::Vector3f temp1=sourceVec1.cross(sourceVec2);
	Eigen::Vector3f temp2=destVec1.cross(destVec2);
	src.push_back(pcl::PointXYZ(sourceVec1(0),sourceVec1(1),sourceVec1(2)));
	src.push_back(pcl::PointXYZ(sourceVec2(0),sourceVec2(1),sourceVec2(2)));
	src.push_back(pcl::PointXYZ(temp1(0),temp1(1),temp1(2)));
	dst.push_back(pcl::PointXYZ(destVec1(0),destVec1(1),destVec1(2)));
	dst.push_back(pcl::PointXYZ(destVec2(0),destVec2(1),destVec2(2)));
	dst.push_back(pcl::PointXYZ(temp2(0),temp2(1),temp2(2)));
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformation;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> TESVD;  
	TESVD.estimateRigidTransformation (src,dst,transformation);
	R=transformation.block(0,0,3,3);
	T=targetPoint-R*sourcePoint;

	//test 
//  	cout<<"R: "<<R<<endl;
//  	cout<<"T: "<<T<<endl;
//  	cout<<"destVec1:"<<destVec1<<endl;
//  	cout<<R*sourceVec1<<endl;
//  	cout<<"destVec2:"<<destVec2<<endl;
//  	cout<<R*sourceVec2<<endl;
//  	cout<<targetPoint<<endl;
//  	cout<<R*sourcePoint+T<<endl;
	return 0;
}


inline bool isTwoPlanesConsistent(Eigen::Vector4f &srcPlane,Eigen::Vector4f &tarPlane,Eigen::Matrix3f &R,Eigen::Vector3f &T,
								  double cosAngleThreshold, double lengthThreshold)
{
//	cout<<"isTwoPlanesConsistent"<<endl;
	Eigen::Vector4f plane1,plane2;
	plane1.block(0,0,3,1)=R*srcPlane.block(0,0,3,1);
	plane1(3)=-(-srcPlane(3)+(plane1.block(0,0,3,1).transpose()*T)(0));
	plane2=tarPlane;
	if (plane1(3)>0)
	{
		plane1=-plane1;
	}
	if (plane2(3)>0)
	{
		plane2=-plane2;
	}
//  	cout<<plane1.block(0,0,3,1).dot(plane2.block(0,0,3,1))<<endl;
//  	cout<<abs(plane1(3)-plane2(3))<<endl;
	if (plane1.block(0,0,3,1).dot(plane2.block(0,0,3,1))<cosAngleThreshold||abs(plane1(3)-plane2(3))>lengthThreshold)
	{
		return false;
	}
	return true;
}


bool isTwoPlanesConsistent1(Eigen::Vector4f &srcPlane,Eigen::Vector4f &tarPlane,
								   Eigen::Vector3f &srcPlaneBoundingBoxCenter,
								   Eigen::Vector3f &destPlaneBoundingBoxCenter,
								   float &srcPlaneBoundingBoxRadius,
								   float &destPlaneBoundingBoxRadius,
								   Eigen::Matrix3f &R,Eigen::Vector3f &T,
								  double cosAngleThreshold, double lengthThreshold)
{
	//	cout<<"isTwoPlanesConsistent"<<endl;
	Eigen::Vector4f plane1,plane2;
	plane1.block(0,0,3,1)=R*srcPlane.block(0,0,3,1);
	plane1(3)=-(-srcPlane(3)+(plane1.block(0,0,3,1).transpose()*T)(0));
	plane2=tarPlane;

	//check center to plane distance
	Eigen::Vector3f srcCenter2Dest=R*srcPlaneBoundingBoxCenter+T;
// 	cout<<abs(plane2.block(0,0,3,1).dot(srcCenter2Dest)+plane2(3))<<endl;
// 	cout<<abs(plane1.block(0,0,3,1).dot(destPlaneBoundingBoxCenter)+plane1(3))<<endl;
	double center2PlaneDistance=(abs(plane2.block(0,0,3,1).dot(srcCenter2Dest)+plane2(3))
		+abs(plane1.block(0,0,3,1).dot(destPlaneBoundingBoxCenter)+plane1(3)))/2;
	// cout<<plane1.block(0,0,3,1).dot(plane2.block(0,0,3,1))<<endl;
	// cout<<abs(plane1(3)-plane2(3))<<endl;
	if (center2PlaneDistance>lengthThreshold||plane1.block(0,0,3,1).dot(plane2.block(0,0,3,1))<cosAngleThreshold)
	{
		return false;
	}
	//check the distance
	double distance=(srcCenter2Dest-destPlaneBoundingBoxCenter).norm();
//	cout<<distance/(srcPlaneBoundingBoxRadius+destPlaneBoundingBoxRadius)<<endl;
	if (distance/(srcPlaneBoundingBoxRadius+destPlaneBoundingBoxRadius)>1)
	{
	//	cout<<"reject one "<<endl;
		return false;
	}
	return true;
}



//不进行错误检验了
bool isTwoIntersectionLionsConsistent(INTERSECTION_LINE &srcIntersectionLion, INTERSECTION_LINE &destIntersectionLion,
									  std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > &srcPlanes,
									  std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > &destPlanes,
									  Eigen::Matrix3f &R,Eigen::Vector3f &T,
									  double angleThreshold, double lengthThreshold)
{
	cout<<"isTwoIntersectionLionsConsistent:"<<endl;
	cout<<"angleThreshold:"<<angleThreshold<<endl;
	cout<<"lengthThreshold:"<<lengthThreshold<<endl;
	double cosAngleThreshold=cos(angleThreshold);
	//angle
	cout<<abs(srcIntersectionLion.planeAngle-destIntersectionLion.planeAngle)<<endl;
	if (abs(srcIntersectionLion.planeAngle-destIntersectionLion.planeAngle)>angleThreshold)
	{
		return false;
	}
	//intersection line
	Eigen::Vector3f &srcLineVec=srcIntersectionLion.lineVec;
	Eigen::Vector3f &destLineVec=destIntersectionLion.lineVec;
	Eigen::Vector3f src2DestVec=R*srcLineVec;
	Eigen::Vector3f src2DestPoint=R*srcIntersectionLion.linePoint+T;
	cout<<abs(src2DestVec.dot(destLineVec))<<endl;
	if (abs(src2DestVec.dot(destLineVec))<cosAngleThreshold)
	{
		return false;
	}
  	cout<<abs((src2DestPoint-destIntersectionLion.linePoint).dot(destLineVec)
  		/(src2DestPoint-destIntersectionLion.linePoint).norm())<<endl;
	if (abs((src2DestPoint-destIntersectionLion.linePoint).dot(destLineVec)
		/(src2DestPoint-destIntersectionLion.linePoint).norm())<cosAngleThreshold)
	{
		return false;
	}
	//planes
	int srcIndex1=srcIntersectionLion.supportPlanes[0];
	int srcIndex2=srcIntersectionLion.supportPlanes[1];
	int destIndex1=destIntersectionLion.supportPlanes[0];
	int destIndex2=destIntersectionLion.supportPlanes[1];
	if (isTwoPlanesConsistent(srcPlanes[srcIndex1],destPlanes[destIndex1],R,T,cosAngleThreshold,lengthThreshold))
	{
		if (isTwoPlanesConsistent(srcPlanes[srcIndex2],destPlanes[destIndex2],R,T,cosAngleThreshold,lengthThreshold))
		{
			return true;
		}
	}
	else if (isTwoPlanesConsistent(srcPlanes[srcIndex1],destPlanes[destIndex2],R,T,cosAngleThreshold,lengthThreshold))
	{
		if (isTwoPlanesConsistent(srcPlanes[srcIndex2],destPlanes[destIndex1],R,T,cosAngleThreshold,lengthThreshold))
		{
			return true;
		}
	}
	else
	{
		return false;
	}
	
	return false;
}

//不进行错误检验了
bool isTwoIntersectionLionsConsistent1(INTERSECTION_LINE &srcIntersectionLion, INTERSECTION_LINE &destIntersectionLion,
									  std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > &srcPlanes,
									  std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > &destPlanes,
									  std::vector<Eigen::Vector3f> &srcPlaneBoundingBoxCenter,
									  std::vector<Eigen::Vector3f> &destPlaneBoundingBoxCenter,
									  std::vector<float> &srcPlaneBoundingBoxRadius,
									  std::vector<float> &destPlaneBoundingBoxRadius,
									  Eigen::Matrix3f &R,Eigen::Vector3f &T,
									  double angleThreshold, double lengthThreshold)
{
// 	  	cout<<"isTwoIntersectionLionsConsistent:"<<endl;
// 	  	cout<<"angleThreshold:"<<angleThreshold<<endl;
// 	  	cout<<"lengthThreshold:"<<lengthThreshold<<endl;
	double cosAngleThreshold=cos(angleThreshold);
	//angle
	//cout<<abs(srcIntersectionLion.planeAngle-destIntersectionLion.planeAngle)<<endl;
	if (srcIntersectionLion.isIntersectionLine&&destIntersectionLion.isIntersectionLine)
	{
		if (abs(srcIntersectionLion.planeAngle-destIntersectionLion.planeAngle)>angleThreshold)
		{
			return false;
		}
	}
	//intersection line
	Eigen::Vector3f &srcLineVec=srcIntersectionLion.lineVec;
	Eigen::Vector3f &destLineVec=destIntersectionLion.lineVec;
	Eigen::Vector3f src2DestVec=R*srcLineVec;
	Eigen::Vector3f src2DestPoint=R*srcIntersectionLion.linePoint+T;
	//	cout<<abs(src2DestVec.dot(destLineVec))<<endl;
	if (abs(src2DestVec.dot(destLineVec))<cosAngleThreshold)
	{
		return false;
	}
// 	  	cout<<abs((src2DestPoint-destIntersectionLion.linePoint).dot(destLineVec)
// 	  		/(src2DestPoint-destIntersectionLion.linePoint).norm())<<endl;
	if (abs((src2DestPoint-destIntersectionLion.linePoint).dot(destLineVec)
		/(src2DestPoint-destIntersectionLion.linePoint).norm())<cosAngleThreshold)
	{
		//return false;
	}
	//planes
	if (srcIntersectionLion.isIntersectionLine&&destIntersectionLion.isIntersectionLine)
	{
		int srcIndex1=srcIntersectionLion.supportPlanes[0];
		int srcIndex2=srcIntersectionLion.supportPlanes[1];
		int destIndex1=destIntersectionLion.supportPlanes[0];
		int destIndex2=destIntersectionLion.supportPlanes[1];
		if (isTwoPlanesConsistent1(srcPlanes[srcIndex1],destPlanes[destIndex1],
			srcPlaneBoundingBoxCenter[srcIndex1],destPlaneBoundingBoxCenter[destIndex1],
			srcPlaneBoundingBoxRadius[srcIndex1],destPlaneBoundingBoxRadius[destIndex1],
			R,T,cosAngleThreshold,lengthThreshold))
		{
			if (isTwoPlanesConsistent1(srcPlanes[srcIndex2],destPlanes[destIndex2],
				srcPlaneBoundingBoxCenter[srcIndex2],destPlaneBoundingBoxCenter[destIndex2],
				srcPlaneBoundingBoxRadius[srcIndex2],destPlaneBoundingBoxRadius[destIndex2],
				R,T,cosAngleThreshold,lengthThreshold))
			{
				return true;
			}
		}
		else if (isTwoPlanesConsistent1(srcPlanes[srcIndex1],destPlanes[destIndex2],
			srcPlaneBoundingBoxCenter[srcIndex1],destPlaneBoundingBoxCenter[destIndex2],
			srcPlaneBoundingBoxRadius[srcIndex1],destPlaneBoundingBoxRadius[destIndex2],
			R,T,cosAngleThreshold,lengthThreshold))
		{
			if (isTwoPlanesConsistent1(srcPlanes[srcIndex2],destPlanes[destIndex1],
				srcPlaneBoundingBoxCenter[srcIndex2],destPlaneBoundingBoxCenter[destIndex1],
				srcPlaneBoundingBoxRadius[srcIndex2],destPlaneBoundingBoxRadius[destIndex1],
				R,T,cosAngleThreshold,lengthThreshold))
			{
				return true;
			}
		}
		else
		{
			return false;
		}
	}
	else
	{
		if (srcIntersectionLion.isIntersectionLine)
		{
			int srcIndex1=srcIntersectionLion.supportPlanes[0];
			int srcIndex2=srcIntersectionLion.supportPlanes[1];
			int destIndex1=destIntersectionLion.supportPlanes[0];
			if (isTwoPlanesConsistent1(srcPlanes[srcIndex1],destPlanes[destIndex1],
				srcPlaneBoundingBoxCenter[srcIndex1],destPlaneBoundingBoxCenter[destIndex1],
				srcPlaneBoundingBoxRadius[srcIndex1],destPlaneBoundingBoxRadius[destIndex1],
				R,T,cosAngleThreshold,lengthThreshold))
			{
				return true;
			}
			else if (isTwoPlanesConsistent1(srcPlanes[srcIndex2],destPlanes[destIndex1],
				srcPlaneBoundingBoxCenter[srcIndex2],destPlaneBoundingBoxCenter[destIndex1],
				srcPlaneBoundingBoxRadius[srcIndex2],destPlaneBoundingBoxRadius[destIndex1],
				R,T,cosAngleThreshold,lengthThreshold))
			{

				return true;
			}
			else
			{
				return false;
			}
		}
		else if(destIntersectionLion.isIntersectionLine)
		{
			int srcIndex1=srcIntersectionLion.supportPlanes[0];
			int destIndex1=destIntersectionLion.supportPlanes[0];
			int destIndex2=destIntersectionLion.supportPlanes[1];
			if (isTwoPlanesConsistent1(srcPlanes[srcIndex1],destPlanes[destIndex1],
				srcPlaneBoundingBoxCenter[srcIndex1],destPlaneBoundingBoxCenter[destIndex1],
				srcPlaneBoundingBoxRadius[srcIndex1],destPlaneBoundingBoxRadius[destIndex1],
				R,T,cosAngleThreshold,lengthThreshold))
			{
				return true;
			}
			else if (isTwoPlanesConsistent1(srcPlanes[srcIndex1],destPlanes[destIndex2],
				srcPlaneBoundingBoxCenter[srcIndex1],destPlaneBoundingBoxCenter[destIndex2],
				srcPlaneBoundingBoxRadius[srcIndex1],destPlaneBoundingBoxRadius[destIndex2],
				R,T,cosAngleThreshold,lengthThreshold))
			{

				return true;
			}
			else
			{
				return false;
			}
		}
		else
		{
			int srcIndex1=srcIntersectionLion.supportPlanes[0];
			int destIndex1=destIntersectionLion.supportPlanes[0];
			if (isTwoPlanesConsistent1(srcPlanes[srcIndex1],destPlanes[destIndex1],
				srcPlaneBoundingBoxCenter[srcIndex1],destPlaneBoundingBoxCenter[destIndex1],
				srcPlaneBoundingBoxRadius[srcIndex1],destPlaneBoundingBoxRadius[destIndex1],
				R,T,cosAngleThreshold,lengthThreshold))
			{
				return true;
			}
			else
			{
				return false;
			}
		}
	}


	return false;
}

int ComputeProjectionPointOf3DLine(const Eigen::Vector3f &lineVec, const Eigen::Vector3f &linePoint,
								   const pcl::PointXYZ &pointIn, pcl::PointXYZ &pointOut)
{
	if (lineVec[0]==0&&lineVec[1]==0&&lineVec[2]==0)
	{
		return -1;//直线参数错误
	}
	double m=lineVec[0];
	double n=lineVec[1];
	double p=lineVec[2];
	double x0=linePoint[0];
	double y0=linePoint[1];
	double z0=linePoint[2];
	double x1=pointIn.x;
	double y1=pointIn.y;
	double z1=pointIn.z;
	double t=-(m*(x0-x1)+n*(y0-y1)+p*(z0-z1))/(m*m+n*n+p*p);
	pointOut.x=float(m*t+x0);
	pointOut.y=float(n*t+y0);
	pointOut.z=float(p*t+z0);
	return 0;
}

int ComputeProjectionPointOf3DLine(const Eigen::Vector3f &lineVec, const Eigen::Vector3f &linePoint,
								   const pcl::PointXYZ&pointIn, Eigen::Vector3f &pointOut)
{
	if (lineVec[0]==0&&lineVec[1]==0&&lineVec[2]==0)
	{
		return -1;//直线参数错误
	}
	double m=lineVec[0];
	double n=lineVec[1];
	double p=lineVec[2];
	double x0=linePoint[0];
	double y0=linePoint[1];
	double z0=linePoint[2];
	double x1=pointIn.x;
	double y1=pointIn.y;
	double z1=pointIn.z;
	double t=-(m*(x0-x1)+n*(y0-y1)+p*(z0-z1))/(m*m+n*n+p*p);
	pointOut[0]=float(m*t+x0);
	pointOut[1]=float(n*t+y0);
	pointOut[2]=float(p*t+z0);
	return 0;
}


int ComputeMeanDistanceOfLine2Plane(INTERSECTION_LINE &line,const pcl::PointCloud<pcl::PointXYZ>&cornerPoints,
									pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree,double &meanDistance,float interval)
{
	if (cornerPoints.empty())
	{
		return -1;
	}
	//////////////////////////////////////////////////////////////////////////
	line.lineVec.normalize();
	std::vector<LENGTHINDEX> length(cornerPoints.size());
	std::vector<Eigen::Vector3f> projectPoints;
	projectPoints.resize(cornerPoints.size());
	Eigen::Vector3f basePoint;
	for (size_t i=0;i<cornerPoints.size();i++)
	{
		//cout<<"corner:"<<cornerPoints[i]<<endl;
		if (0!=ComputeProjectionPointOf3DLine(line.lineVec,line.linePoint,cornerPoints[i],projectPoints[i]))
		{
			return -1;
		}
		length[i].length=line.lineVec.dot(projectPoints[i]-projectPoints[0]);
		length[i].index=i;
	}

	sort(length.begin(),length.end(),myCompareLess);// 从小到大排序
	int count=0;
	double lengthSum=0;
	std::vector<int> neighbor;
	std::vector<float> neighborLength;
// 	cout<<length.front().length<<endl;
// 	cout<<length.back().length<<endl;
	for (float i=length.front().length;i<length.back().length;i+=interval)
	{
		count++;
		Eigen::Vector3f p=projectPoints[0]+i*line.lineVec;
		pcl::PointXYZ currentPoint(p(0),p(1),p(2));
		kdtree->nearestKSearch(currentPoint,1,neighbor,neighborLength);
		lengthSum+=neighborLength[0];
	}
	meanDistance=lengthSum/count;

	//test
// 	boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("test"));
// 	pcl::PointCloud<pcl::PointXYZ>::ConstPtr points=kdtree->getInputCloud();
// 	cout<<"points num:"<<points->size()<<endl;
// 	view->addPointCloud(points);
// 	for (size_t i=0;i<8;i++)
// 	{
// 		view->addSphere(cornerPoints[i],0.01,0,255,0,num2str(i));
// 		view->addSphere(pcl::PointXYZ(projectPoints[i][0],projectPoints[i][1],projectPoints[i][2]),0.01,255,0,0,num2str(i+8));
// 	}
// 	Eigen::Vector3f st=projectPoints[0]+line.lineVec*3;
// 	view->addLine(pcl::PointXYZ(projectPoints[0][0],projectPoints[0][1],projectPoints[0][2]),pcl::PointXYZ(st(0),st(1),st(2)));
// 	while (!view->wasStopped())
// 	{
// 		view->spinOnce(100);
// 
// 	}

	return 0;
}


int ResetNormalDirection(pcl::PointXYZ &viewPoint, pcl::PointCloud<pcl::PointXYZ>& points,
						 pcl::PointCloud<pcl::Normal>& normals)
{
	if (points.empty()||points.size()!=normals.size())
	{
		return -1;
	}
	size_t pointsNum=points.size();
	float vx=viewPoint.x;
	float vy=viewPoint.y;
	float vz=viewPoint.z;
	for (size_t i=0;i<pointsNum;i++)
	{
		pcl::PointXYZ &p=points[i];
		pcl::Normal   &n=normals[i];
		if ((vx-p.x)*n.normal_x+(vy-p.y)*n.normal_y+(vz-p.z)*n.normal_z<0)
		{
			n.normal_x=-n.normal_x;
			n.normal_y=-n.normal_y;
			n.normal_z=-n.normal_z;
		}
	}
	return 0;
}

int ComputeIntersectionPointOf23DLine(Eigen::Vector3f &lineVec1, Eigen::Vector3f &linePoint1,
								 Eigen::Vector3f &lineVec2, Eigen::Vector3f &linePoint2,
								 Eigen::Vector3f &intersectionPoint)
{
	if (abs(lineVec1.dot(lineVec2))>0.9999)
	{
		return -1;//parallel
	}
	cv::Mat A(6,5,CV_32F,cv::Scalar(0));
	cv::Mat B(6,1,CV_32F,cv::Scalar(0));
	cv::Mat X;
	std::vector<float*>ptrA(6),ptrB(6);
	for (int i=0;i<6;i++)
	{
		ptrA[i]=A.ptr<float>(i);
		ptrB[i]=B.ptr<float>(i);
	}
	ptrA[0][0]=1;
	ptrA[0][3]=-lineVec1(0);
	ptrA[1][1]=1;
	ptrA[1][3]=-lineVec1(1);
	ptrA[2][2]=1;
	ptrA[2][3]=-lineVec1(2);
	ptrA[3][0]=1;
	ptrA[3][4]=-lineVec2(0);
	ptrA[4][1]=1;
	ptrA[4][4]=-lineVec2(1);
	ptrA[5][2]=1;
	ptrA[5][4]=-lineVec2(2);
	ptrB[0][0]=linePoint1(0);
	ptrB[1][0]=linePoint1(1);
	ptrB[2][0]=linePoint1(2);
	ptrB[3][0]=linePoint2(0);
	ptrB[4][0]=linePoint2(1);
	ptrB[5][0]=linePoint2(2);

	cv::solve(A,B,X,cv::DECOMP_SVD);
	intersectionPoint(0)=X.at<float>(0);
	intersectionPoint(1)=X.at<float>(1);
	intersectionPoint(2)=X.at<float>(2);
	return 0;

}


int AreTwoPlanesPenetrable(Eigen::Vector4f &plane1,Eigen::Vector4f &plane2, 
						   std::vector<Eigen::Vector3f> &cornerPoints1,
						   std::vector<Eigen::Vector3f> &cornerPoints2,
						   pcl::search::KdTree<pcl::PointXYZ>::Ptr KdTree1,
						   pcl::search::KdTree<pcl::PointXYZ>::Ptr KdTree2,
						   bool &isPentrable,float searchRadius,int minPointsNum,float minDistance)
{

	isPentrable=false;
	if (cornerPoints1.empty()||cornerPoints2.empty())
	{
		return -1;
	}
	//////////////////////////////////////////////////////////////////////////
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr points1=KdTree1->getInputCloud(); 
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr points2=KdTree2->getInputCloud(); 
// 



	//compute the intersection line of two planes
	Eigen::Vector3f lineVec,linePoint;
	if (0!=ComputeIntersectionLineOfTwoPlanes(plane1,plane2,lineVec,linePoint))
	{
// 		cout<<"paral planes"<<endl;
// 		while (!view->wasStopped())
// 		{
// 			view->spinOnce(100);
// 		}
		return -1;
	}
	//check the intersection line with first plane
	size_t num1=cornerPoints1.size();
	std::vector<Eigen::Vector3f> intersectionPoints1;
	intersectionPoints1.reserve(2);
	for (size_t i=1;i<=num1;i++)
	{
		Eigen::Vector3f tempLine;
		tempLine=cornerPoints1[i%num1]-cornerPoints1[(i-1)%num1];
		tempLine.normalize();
		Eigen::Vector3f ip;
		if (0!=ComputeIntersectionPointOf23DLine(lineVec,linePoint,tempLine,cornerPoints1[i-1],ip))
		{
			continue;
		}

		if ((cornerPoints1[(i-1)%num1]-ip).dot(cornerPoints1[i%num1]-ip)>0)
		{
			continue;
		}
		
		intersectionPoints1.push_back(ip);
	}

//   	if (!intersectionPoints1.empty())
//   	{
//   		pcl::PointCloud<pcl::PointXYZ> pclPoints;
//   		ExchnageBetweentPCLPointXYZwithEigenVector3f(pclPoints,intersectionPoints1);
//   		view->addLine(pclPoints.front(),pclPoints.back(),1,0,0,"line1");
//   		view->addSphere(pclPoints.front(),0.03,1,0,0,"sphere1");
//   		view->addSphere(pclPoints.back(),0.03,1,0,0,"sphere2");
//   
//   	}

	
	size_t num2=cornerPoints2.size();
	std::vector<Eigen::Vector3f> intersectionPoints2;
	intersectionPoints2.reserve(2);
	for (size_t i=1;i<=num2;i++)
	{
		Eigen::Vector3f tempLine;

		tempLine=cornerPoints2[i%num2]-cornerPoints2[(i-1)%num2];

		tempLine.normalize();
		Eigen::Vector3f ip;
		if (0!=ComputeIntersectionPointOf23DLine(lineVec,linePoint,tempLine,cornerPoints2[i-1],ip))
		{
			continue;
		}
		if ((cornerPoints2[(i-1)%num2]-ip).dot(cornerPoints2[i%num2]-ip)>0)
		{
			continue;
		}
		intersectionPoints2.push_back(ip);
	}
//  	if (!intersectionPoints2.empty())
//  	{
//  		pcl::PointCloud<pcl::PointXYZ> pclPoints;
//  		ExchnageBetweentPCLPointXYZwithEigenVector3f(pclPoints,intersectionPoints2);
//  		view->addLine(pclPoints.front(),pclPoints.back(),0,1,0,"line2");
//  		view->addSphere(pclPoints.front(),0.03,0,1,0,"sphere3");
//  		view->addSphere(pclPoints.back(),0.03,0,1,0,"sphere4");
//  
//  	}

	if (intersectionPoints1.empty())
	{
		isPentrable=false;
		return 0;
	}
	else if (2!=intersectionPoints1.size())
	{
		return -1;
	}

	if (intersectionPoints2.empty())
	{
		isPentrable=false;
		return 0;
	}
	else if (2!=intersectionPoints2.size())
	{
		return -1;
	}

	//further check
	//check the intersection part
	Eigen::Vector3f direc=intersectionPoints1[1]-intersectionPoints1[0];
	direc.normalize();

	std::vector<Eigen::Vector3f> interPoints=intersectionPoints1;
	interPoints.push_back(intersectionPoints2[0]);
	interPoints.push_back(intersectionPoints2[1]);
	std::vector<LENGTHINDEX> lengthVector(interPoints.size());
	for (size_t i=0;i<interPoints.size();i++)
	{
		lengthVector[i].length=(interPoints[i]-interPoints[0]).dot(direc);
		lengthVector[i].index=i;
	}
	sort(lengthVector.begin(),lengthVector.end(),myCompareLess);// 从小到大排序
	if (0==(lengthVector[0].index/2-lengthVector[1].index/2))
	{
		//no intersection
		isPentrable=false;
		return 0;
	}
	Eigen::Vector3f &startPoint=interPoints[lengthVector[1].index];
	Eigen::Vector3f &endPoint=interPoints[lengthVector[2].index];

	Eigen::Vector3f searchPoint;
	std::vector<int> neighbor;
	std::vector<float> neighborLength;
	int negativeNum=0;
	int positiveNum=0;
	float length=(endPoint-startPoint).norm();
	std::vector<bool> checkIndex1(points1->size(),true);
	int count=0;
	for (float dist=0;dist<length;dist+=searchRadius)
	{
		searchPoint=startPoint+dist*direc;
		if (KdTree2->radiusSearch(pcl::PointXYZ(searchPoint(0),searchPoint(1),searchPoint(2)),
			searchRadius/2,neighbor,neighborLength,2)<2)
		{
			count++;
			continue;
		}
		KdTree1->radiusSearch(pcl::PointXYZ(searchPoint(0),searchPoint(1),searchPoint(2)),
			searchRadius,neighbor,neighborLength);
		for (size_t i=0;i<neighbor.size();i++)
		{
			if (checkIndex1[neighbor[i]])
			{
				checkIndex1[neighbor[i]]=false;
				const pcl::PointXYZ &p=points1->at(neighbor[i]);
				float tempDistance=plane2[0]*p.x+plane2[1]*p.y+plane2[2]*p.z+plane2[3];
				if (abs(tempDistance)>minDistance)
				{
					if (tempDistance>=0)
					{
						positiveNum++;
					}
					else
					{
						negativeNum++;
					}
				}
			}
		}
	}
//	cout<<"count:"<<count<<endl;
//  	cout<<"positiveNum:"<<positiveNum<<endl;
//  	cout<<"negativeNum:"<<negativeNum<<endl;
	if (positiveNum<minPointsNum||negativeNum<minPointsNum)
	{
		isPentrable=false;
		return 0;
	}
	if (double(MAX(positiveNum,negativeNum))/MIN(positiveNum,negativeNum+1)>5)
	{
		isPentrable=false;
		return 0;
	}

	negativeNum=0;
	positiveNum=0;
	std::vector<bool> checkIndex2(points2->size(),true);
	count=0;
	for (float dist=0;dist<length;dist+=searchRadius)
	{
		searchPoint=startPoint+dist*direc;
		if (KdTree1->radiusSearch(pcl::PointXYZ(searchPoint(0),searchPoint(1),searchPoint(2)),
			searchRadius/2,neighbor,neighborLength,2)<2)
		{
			count++;
			continue;
		}
		KdTree2->radiusSearch(pcl::PointXYZ(searchPoint(0),searchPoint(1),searchPoint(2)),
			searchRadius,neighbor,neighborLength);
		for (size_t i=0;i<neighbor.size();i++)
		{
			if (checkIndex2[neighbor[i]])
			{
				checkIndex2[neighbor[i]]=false;
				const pcl::PointXYZ &p=points2->at(neighbor[i]);
				float tempDistance=plane1[0]*p.x+plane1[1]*p.y+plane1[2]*p.z+plane1[3];
				if (abs(tempDistance)>minDistance)
				{
					if (tempDistance>=0)
					{
						positiveNum++;
					}
					else
					{
						negativeNum++;
					}
				}
			}
		}
	}
//	cout<<"count:"<<count<<endl;
// 	cout<<"positiveNum:"<<positiveNum<<endl;
// 	cout<<"negativeNum:"<<negativeNum<<endl;
	if (positiveNum<minPointsNum&&negativeNum<minPointsNum)
	{
		isPentrable=false;
		return 0;
	}
	if (double(MAX(positiveNum,negativeNum))/MIN(positiveNum,negativeNum+1)>5)
	{
		isPentrable=false;
		return 0;
	}

	isPentrable=true;

// 	boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("3DSC test"));
// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(points1,250,0,0);
// 	view->addPointCloud(points1,sources_cloud_color,"sources_cloud");
// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color (points2,0,250,0);
// 	view->addPointCloud(points2,target_cloud_color,"target_cloud");
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr corner1(new pcl::PointCloud<pcl::PointXYZ>),corner2(new pcl::PointCloud<pcl::PointXYZ>);
// 	ExchnageBetweentPCLPointXYZwithEigenVector3f(*corner1,cornerPoints1);
// 	ExchnageBetweentPCLPointXYZwithEigenVector3f(*corner2,cornerPoints2);
// 	view->addPointCloud(corner1,sources_cloud_color,"corner1");
// 	view->addPointCloud(corner2,target_cloud_color,"corner2");
// 	while (!view->wasStopped())
// 	{
// 		view->spinOnce(100);
// 	}
	return 0;
}

int ExchnageBetweentPCLPointXYZwithEigenVector3f(pcl::PointCloud<pcl::PointXYZ> &pclPoint,
												 std::vector<Eigen::Vector3f>& eigenPoints)
{
	if (pclPoint.empty())
	{
		size_t pointsNum=eigenPoints.size();
		pclPoint.resize(pointsNum);
		for (size_t i=0;i<pointsNum;i++)
		{
			memcpy(&pclPoint[i],&eigenPoints[i],sizeof(float)*3);
		}
	}
	else
	{
		size_t pointsNum=pclPoint.size();
		eigenPoints.resize(pointsNum);
		for (size_t i=0;i<pointsNum;i++)
		{
			memcpy(&eigenPoints[i],&pclPoint[i],sizeof(float)*3);
		}
	}
	return 0;
}

int ProjectPoints2Plane(const pcl::PointCloud<pcl::PointXYZ>&pointsIn,const Eigen::Vector4f &plane,
						std::vector<Eigen::Vector3f>&pointsOut,int *pBegin/*=NULL*/,int *pEnd/*=NULL*/)
{
	pcl::PointCloud<pcl::PointXYZ> pclPoints;
	if (0!=ProjectPoints2Plane(pointsIn,plane,pclPoints,pBegin,pEnd))
	{
		return -1;
	}
	return ExchnageBetweentPCLPointXYZwithEigenVector3f(pclPoints,pointsOut);
}

int ProjectPoints2Plane(const pcl::PointCloud<pcl::PointXYZ>&pointsIn,const Eigen::Vector4f &plane,
						pcl::PointCloud<pcl::PointXYZ>&pointsOut,
						int *pBegin/*=NULL*/,int *pEnd/*=NULL*/)
{
	if (pointsIn.empty())
	{
		return -1;
	}
	//////////////////////////////////////////////////////////////////////////
	float A=plane(0);
	float B=plane(1);
	float C=plane(2);
	float D=plane(3);
	size_t start=0,end=pointsIn.size();
	if (pBegin&&pEnd)
	{
		start=*pBegin;
		end=*pEnd;
		if (end>pointsIn.size())
		{
			end=pointsIn.size();
		}
	}
	pointsOut.resize(end-start);
	//#pragma omp parallel for
	for (int i=start;i<end;i++)
	{
		const pcl::PointXYZ &tempPoints=pointsIn[i];
		if (!cvIsInf(tempPoints.x+tempPoints.y+tempPoints.z)&&!cvIsNaN(tempPoints.x+tempPoints.y+tempPoints.z))
		{
			float k=-(A*tempPoints.x+B*tempPoints.y+C*tempPoints.z+D)/(A*A+B*B+C*C);
			pointsOut[i-start].x=tempPoints.x+k*A;
			pointsOut[i-start].y=tempPoints.y+k*B;
			pointsOut[i-start].z=tempPoints.z+k*C;
		}
		else
		{
			pointsOut[i-start].x=generateNan();
			pointsOut[i-start].y=pointsOut[i].x;
			pointsOut[i-start].z=pointsOut[i].x;
		}
	}
	//test
// 	boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("test"));
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr tempSource(new pcl::PointCloud<pcl::PointXYZ>);
// 	*tempSource=pointsIn;
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr tempProject(new pcl::PointCloud<pcl::PointXYZ>);
// 	*tempProject=pointsOut;
// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(tempSource,255,0,0);
// 	view->addPointCloud(tempSource,sources_cloud_color,cv::format("downsample"));
// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(tempProject,0,255,0);
// 	view->addPointCloud(tempProject,target_cloud_color,cv::format("source"));
// 	while (!view->wasStopped())
// 	{
// 		view->spinOnce(100);
// 	}
	return 0;
}

pcl::PointCloud<pcl::Normal>::Ptr ComputeNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr points,double normalEstimationRadius)
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


int DetectBoundaryPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr points, std::vector<int> &boundariePointsIndex,
						 float normalRadius/*=0.03*/, float boundarySearchRadius/*=0.05*/,bool showResult)
{
	if (points->empty())
	{
		return -1;
	}
	//////////////////////////////////////////////////////////////////////////
	//进行边缘检测
	pcl::PointCloud<pcl::Normal>::Ptr normals=ComputeNormal(points,normalRadius);
	pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr boundaryTree(new pcl::search::KdTree<pcl::PointXYZ>);
	est.setInputCloud (points);
	est.setInputNormals (normals);
	est.setRadiusSearch (boundarySearchRadius);   // radius
	est.setSearchMethod (boundaryTree);
	est.compute (boundaries);

	boundariePointsIndex.clear();
	for (size_t i=0;i<boundaries.points.size();i++)
	{
		if (boundaries.points[i].boundary_point)
		{
			boundariePointsIndex.push_back(i);
		}
	}
	if (showResult)
	{
		//test
		pcl::PointCloud<pcl::PointXYZ>::Ptr boundariePoints(new pcl::PointCloud<pcl::PointXYZ>);
		boundariePoints->reserve(boundariePointsIndex.size());
		for (size_t i=0;i<boundariePointsIndex.size();i++)
		{
			boundariePoints->push_back((*points)[boundariePointsIndex[i]]);
		}
		boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("3DSC test"));
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(points,250,0,0);
		view->addPointCloud(points,sources_cloud_color,"sources_cloud_v1");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> boundaryColor (boundariePoints,0,255,0);
		view->addPointCloud(boundariePoints,boundaryColor,"boundaryPoints");
		while (!view->wasStopped())
		{
			view->spinOnce(100);
		}
	}


	return 0;
}


int FilterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr points, std::vector<int> &boundariePointsIndex,pcl::PointXYZ &viewPoint,
				 std::vector<int>& outPoints,bool isRemoveOcclusionPoints )
{
	if (points->empty())
	{
		return -1;
	}
	outPoints.clear();
	//////////////////////////////////////////////////////////////////////////
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::KdTree<pcl::PointXYZ>::IndicesPtr index(new std::vector<int>);
	*index=boundariePointsIndex;
	kdTree->setInputCloud(points,index);
	float searchRadius=0.05;
	pcl::PointXYZ searchPoint;
	std::vector<int>neighbor;
	std::vector<float> neighborLength;
	outPoints.reserve(boundariePointsIndex.size());
	for (size_t i=0;i<boundariePointsIndex.size();i++)
	{
		int index=boundariePointsIndex[i];
		pcl::PointXYZ &start=(*points)[index];
		Eigen::Vector3f lineVec(viewPoint.x-start.x,viewPoint.y-start.y,viewPoint.z-start.z);
		float length=lineVec.norm();
		lineVec.normalize();
		bool isOcclusion=false;
		for (float dist=2*searchRadius;dist<length;dist+=searchRadius)
		{
			searchPoint.x=start.x+dist*lineVec(0);
			searchPoint.y=start.y+dist*lineVec(1);
			searchPoint.z=start.z+dist*lineVec(2);
			if (kdTree->radiusSearch(searchPoint,searchRadius,neighbor,neighborLength,1)>0)
			{
				isOcclusion=true;
				break;
			}
		}
		if (isRemoveOcclusionPoints)
		{
			if (!isOcclusion)
			{
				outPoints.push_back(index);
			}
		}
		else
		{
			if (isOcclusion)
			{
				outPoints.push_back(index);
			}
		}
	}

	//test
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr boundariePoints(new pcl::PointCloud<pcl::PointXYZ>);
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr boundariePoints1(new pcl::PointCloud<pcl::PointXYZ>);
// 	boundariePoints->reserve(boundariePointsIndex.size());
// 	boundariePoints1->reserve(outPoints.size());
// 	for (size_t i=0;i<boundariePointsIndex.size();i++)
// 	{
// 		boundariePoints->push_back((*points)[boundariePointsIndex[i]]);
// 	}
// 	for (size_t i=0;i<outPoints.size();i++)
// 	{
// 		boundariePoints1->push_back((*points)[outPoints[i]]);
// 	}
// 	boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("3DSC test"));
// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(points,250,0,0);
// 	view->addPointCloud(points,sources_cloud_color,"sources_cloud_v1");
// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> boundaryColor (boundariePoints,0,255,0);
// 	view->addPointCloud(boundariePoints,boundaryColor,"boundaryPoints");
// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> boundaryColor1 (boundariePoints1,0,0,255);
// 	view->addPointCloud(boundariePoints1,boundaryColor1,"boundaryPoints1");
// 	while (!view->wasStopped())
// 	{
// 		view->spinOnce(100);
// 	}

	return 0;
	
}

int load_bpn(const std::string& file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud,
			 pcl::PointCloud<pcl::Normal>*pointNormal/*=NULL*/) 
{
	std::ifstream input(file_name.c_str(), std::fstream::binary);
	if (input.fail())
	{
		std::cout<< "could open file\'" << file_name << "\'" << std::endl;
		return -1;
	}

	std::size_t components_per_point = 6;
	std::size_t bytes_per_point = sizeof(float) * components_per_point;

	input.seekg(0, input.end);
	std::streamoff length = input.tellg();
	input.seekg(0, input.beg);

	// num of points in the file
	std::size_t num = length / bytes_per_point;
	if (num <= 0)
		return -1;

	pointCloud->width = num;
	pointCloud->height = 1;
	pointCloud->resize (pointCloud->width * pointCloud->height);

	if (pointNormal)
	{
		pointNormal->resize(pointCloud->width * pointCloud->height);
	}

	float* data = new float[num * components_per_point];
	input.read((char*)data, num * bytes_per_point);	// read the entire block
	if (input.fail()) {
		std::cout<< input.gcount() << " bytes of the block could not be read" << std::endl;
		delete[] data;
		return -1;
	}

	//ProgressLogger progress(num);
	if (pointNormal)
	{
#pragma omp parallel for
		for (int i = 0; i < num; ++i) 
		{
			float* ptr = data + i * 6;
			pointCloud->at(i) = pcl::PointXYZ(ptr[0],ptr[1],ptr[2]);
			pointNormal->at(i)=pcl::Normal(ptr[3],ptr[4],ptr[5]);
		}
	}
	else
	{
#pragma omp parallel for
		for (int i = 0; i < num; ++i) 
		{
			float* ptr = data + i * 6;
			pointCloud->at(i) = pcl::PointXYZ(ptr[0],ptr[1],ptr[2]);
		}
	}


	delete[] data;
	return 0;
}

void save_bpn(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZ>& pointCloud, pcl::PointCloud<pcl::Normal>*pPointNormal)
{
	// open file
	std::ofstream output(file_name.c_str(), std::fstream::binary);
	if (output.fail()) {
		std::cout << "could not open file\'" << file_name << "\'" << std::endl;
		return;
	}

	// TODO: save large blocks
	//ProgressLogger progress(pset->num_point());
	if (pPointNormal)
	{
		for (std::size_t i = 0; i < pointCloud.size(); ++i) 
		{
			output.write((char*)&pointCloud[i], 12);
			output.write((char*)&(*pPointNormal)[i],12);
		}
	}
	else
	{
		float normal[3]={0,0,0};
		for (std::size_t i = 0; i < pointCloud.size(); ++i) 
		{
			output.write((char*)&pointCloud[i], 12);
			output.write((char*)&normal[0],12);
		}
	}

}

int load_xyz(const std::string &fileName,pcl::PointCloud<pcl::PointXYZ>::Ptr pointCLoud)
{
	std::ifstream fin;
	fin.open(fileName);
	if (fin.fail())
	{
		std::cout << "could not open file\'" << fileName<< "\'" << std::endl;
		return -1;
	}
	int totalLineNum=0;
	while (!fin.eof())
	{
		fin.ignore( numeric_limits<streamsize>::max(),'\n');
		totalLineNum++;
	}
	fin.close();
	pointCLoud->clear();
	pointCLoud->reserve(totalLineNum);
	float x,y,z;
	fin.open(fileName);
	if (fin.fail())
	{
		std::cout << "could not open file\'" << fileName<< "\'" << std::endl;
		return -1;
	}
	int currentLineNo=0;
	int acc=int(totalLineNum/100.0);
	int tempAcc=0; 
	std::stringstream str;
	while (!fin.eof())
	{
		fin >> std::ws;  // eat up any leading white spa
		if (fin.peek()!='#')
		{
			fin >> std::ws;  // eat up any leading white spa
			if (fin>>x>>y>>z)
			{
				pointCLoud->push_back(pcl::PointXYZ(x,y,z));
				currentLineNo++;
				tempAcc++;
				if (tempAcc>=acc)
				{
					cout<<"\r";
					cout<<"compplete: "<<int(double(currentLineNo)/totalLineNum*100)<<"%";
					tempAcc=0;
				}
			}
		}
		fin.ignore(numeric_limits<streamsize>::max(),'\n');
	}
	cout<<"\r";
	cout<<"compplete: 100%, total "<<pointCLoud->size()<<" points"<<endl;
	return 0;
}

int load_plane(const std::string &fileName,std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> >&planes)
{
	std::ifstream fin;
	fin.open(fileName);
	if (fin.fail())
	{
		std::cout << "could not open file\'" << fileName<< "\'" << std::endl;
		return -1;
	}
	planes.clear();
	int totalLineNum=0;
	while (!fin.eof())
	{
		fin.ignore( numeric_limits<streamsize>::max(),'\n');
		totalLineNum++;
	}
	fin.close();
	planes.reserve(totalLineNum);
	Eigen::Vector4f planeParam;
	fin.open(fileName);
	while (!fin.eof())
	{
		fin >> std::ws;  // eat up any leading white spa
		if (fin.peek()!='#')
		{
			fin >> std::ws;  // eat up any leading white spa
			if (fin>>planeParam[0]>>planeParam[1]>>planeParam[2]>>planeParam[3])
			{
				planes.push_back(planeParam);
			}
			fin.ignore(numeric_limits<streamsize>::max(),'\n');
		}
	}
	return 0;
}

int load_support_points_index(const std::string &fileName,std::vector<std::vector<int>>&supportPointsIndex)
{
	std::ifstream fin;
	fin.open(fileName);
	if (fin.fail())
	{
		std::cout << "could not open file\'" << fileName<< "\'" << std::endl;
		return -1;
	}
	supportPointsIndex.clear();
	while (!fin.eof())
	{
		fin >> std::ws;  // eat up any leading white spa
		if (fin.peek()!='#')
		{
			fin >> std::ws;  // eat up any leading white spa
			int pointsNum;
			if (fin>>pointsNum)
			{
				std::vector<int>currentPointsIndex(pointsNum);
				for (int i=0;i<pointsNum;i++)
				{
					fin>>currentPointsIndex[i];
				}
				supportPointsIndex.push_back(currentPointsIndex);
			}
			fin.ignore(numeric_limits<streamsize>::max(),'\n');
		}
	}
	return 0;
}



int Fit3DLine(std::vector<cv::Point3f>&points, cv::Vec6f &param,float *pAccuracy/*=NULL*/)
{
	if (points.size()<3)
	{
		return -1;
	}
	//////////////////////////////////////////////////////////////////////////
	cv::fitLine(points,param,CV_DIST_L2,0,0.01,0.01);
	//判断直线拟合结果
	if (param[0]==0&&param[1]==0&&param[2]==0)
	{
		return -1;
	}
	if (pAccuracy)//如果需要计算拟合精度
	{
		//计算点到直线的距离
		int pointsNum=points.size();
		double sum=0;
		for (int i=0;i<pointsNum;i++)
		{
			sum+=computeLengthOfPoint23DLine(param,points[i]);
		}
		*pAccuracy=float(sum/pointsNum);
	}
	return 0;

}

int RansacExtract3Dline(pcl::PointCloud<pcl::PointXYZ>&points,std::vector<int>&boudaryPointsIndex, 
						std::vector<std::vector<int>>& extractLines,
						int maxLinesNum, double inlierThreshold,int minLinePointsNum,int eachLineTestNum,bool showResult)
{
	if (points.empty()||boudaryPointsIndex.empty()||boudaryPointsIndex.size()<minLinePointsNum)
	{
		return -1;
	}
	//////////////////////////////////////////////////////////////////////////
	srand(UINT(time(0)));
	int boundaryPointsNum=boudaryPointsIndex.size();
	extractLines.clear();
	extractLines.reserve(maxLinesNum);
	std::vector<bool> haveBeenUsed(boundaryPointsNum,false);
	int allAvaliablePointsNum=boundaryPointsNum;
	for (int i=0;i<maxLinesNum*2;i++)
	{
		std::vector<std::vector<int>> inliers;
		inliers.reserve(eachLineTestNum);
		std::vector<LENGTHINDEX> lengthIndex;
		lengthIndex.reserve(eachLineTestNum);
		int j;
		for (j=0;j<eachLineTestNum;j++)
		{
			int i1=rand()%boundaryPointsNum;
			int count;
			for (count=0;count<boundaryPointsNum;count++)
			{
				if (haveBeenUsed[i1])
				{
					i1=(++i1)%boundaryPointsNum;
					continue;
				}
				break;
			}
			if (count==boundaryPointsNum)
			{
				break;//all boundary points have been used
			}
			int i2=rand()%boundaryPointsNum;
			for (count=0;count<boundaryPointsNum;count++)
			{
				if (haveBeenUsed[i2]||i2==i1)
				{
					i2=(++i2)%boundaryPointsNum;
					continue;
				}
				break;
			}
			if (count==boundaryPointsNum)
			{
				break;//all boundary points have been used
			}
			pcl::PointXYZ& start=points[boudaryPointsIndex[i1]];
			pcl::PointXYZ& end=points[boudaryPointsIndex[i2]];
			Eigen::Vector3f currentLineVec(start.x-end.x,start.y-end.y,start.z-end.z);
			// compute the length of each point to this line
			std::vector<int> currentInlier;
			currentInlier.push_back(i1);
			currentInlier.push_back(i2);
			for (int k=0;k<boundaryPointsNum;k++)
			{
				if (haveBeenUsed[k])
				{
					continue;
				}
				if (computeLengthOfPoint23DLine(currentLineVec,start,points[boudaryPointsIndex[k]])<inlierThreshold)
				{
					currentInlier.push_back(k);
				}
				
			}
			inliers.push_back(currentInlier);
			LENGTHINDEX tempLength;
			tempLength.index=j;
			tempLength.length=currentInlier.size();
			lengthIndex.push_back(tempLength);
		}
		if (j<eachLineTestNum)
		{
			//cout<<"no avalible points"<<endl;
			break;//no avalible points
		}
		partial_sort(lengthIndex.begin(),lengthIndex.begin()+1,lengthIndex.end(),myCompareGreater);//找到内点最多的
		if (lengthIndex[0].length<minLinePointsNum)
		{
			continue;
		}
		int index=lengthIndex[0].index;
		std::vector<int> currentExtractLine(inliers[index].size());
		for (size_t k=0;k<inliers[index].size();k++)
		{
			haveBeenUsed[inliers[index][k]]=true;
			currentExtractLine[k]=boudaryPointsIndex[inliers[index][k]];
		}
		extractLines.push_back(currentExtractLine);
		allAvaliablePointsNum-=lengthIndex[0].length;
		if (allAvaliablePointsNum<minLinePointsNum)
		{
			break;
		}
	}

	if (showResult)
	{
		//test
		boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("test"));
		pcl::PointCloud<pcl::PointXYZ>::Ptr boundariePoints(new pcl::PointCloud<pcl::PointXYZ>);
		boundariePoints->reserve(boudaryPointsIndex.size());
		for (size_t i=0;i<boudaryPointsIndex.size();i++)
		{
			boundariePoints->push_back((points)[boudaryPointsIndex[i]]);
		}
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(boundariePoints,255,255,255);
		view->addPointCloud(boundariePoints,sources_cloud_color,"sources_cloud_v1");
		cout<<"totalLinesNum:"<<extractLines.size()<<endl;
		for (size_t i=0;i<extractLines.size();i++)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr lines(new pcl::PointCloud<pcl::PointXYZ>);
			lines->reserve(extractLines[i].size());
			cout<<"line"<<i<<"   "<<extractLines[i].size()<<endl;
			for (size_t j=0;j<extractLines[i].size();j++)
			{
				lines->push_back((points)[extractLines[i][j]]);
			}
			double r=rand()%255;
			double g=rand()%255;
			double b=rand()%255;
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> linesColor(lines,r,g,b);
			view->addPointCloud(lines,linesColor,cv::format("lines%d",i));
			view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,4,cv::format("lines%d",i));
			view->addLine(lines->front(),lines->back(),r,g,b,cv::format("liness%d",i));
		}
		while (!view->wasStopped())
		{
			view->spinOnce(100);
		}
	}

	return 0;
}

bool CheckWhetherPointCLoudsInTheInlierOfRoomFrame(
							std::vector<std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>>>&roomFrame,
							pcl::PointCloud<pcl::PointXYZ>&testPoints,double distanceThreshold,int maxOutlierNum)
{
	if (roomFrame.empty()||testPoints.empty())
	{
		return true;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr outLierPoints(new pcl::PointCloud<pcl::PointXYZ>);

	//////////////////////////////////////////////////////////////////////////
	size_t pointsNum=testPoints.size();
	size_t boxNum=roomFrame.size();
	int count=0;
	for (size_t i=0;i<pointsNum;i++)
	{
		pcl::PointXYZ &p=testPoints[i];
		int outOfBoxNum=0;
		for (size_t j=0;j<boxNum;j++)
		{
			size_t frameNum=roomFrame[j].size();
			for (size_t k=0;k<frameNum;k++)
			{
				double length=roomFrame[j][k](0)*p.x+roomFrame[j][k](1)*p.y+roomFrame[j][k](2)*p.z+roomFrame[j][k](3);
				if (length>=-distanceThreshold)
				{
					continue;
				}
				else
				{
					//	cout<<length<<endl;
					outOfBoxNum++;
					break;
				}
			}
		}
		if (outOfBoxNum==boxNum)
		{
			//outLierPoints->push_back(p);
 			if ((++count)>maxOutlierNum)
 			{
 				return false;
 			}
		}
	}
	//test
	//pcl::PointCloud<pcl::PointXYZ>::Ptr tempPoints(new pcl::PointCloud<pcl::PointXYZ>);
	//*tempPoints=testPoints;
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("test"));
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_cloud_color(tempPoints,255,255,255);
	//view->addPointCloud(tempPoints,src_cloud_color,"src_cloud_v1");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outLier_cloud_color(outLierPoints,255,0,0);
	//view->addPointCloud(outLierPoints,outLier_cloud_color,"coarse_region_cloud_v1");
	//cout<<outLierPoints->size()<<endl;
	//while (!view->wasStopped())
	//{
	//	view->spinOnce(100);
	//}
	return true;
}

//不进行错误检查
bool CheckWhetherPointInTheInlierOfRoomFrame(
	std::vector<std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>>>&roomFrame,
	pcl::PointXYZ &testPoints,double distanceThreshold)
{
	Eigen::Vector3f test(testPoints.x,testPoints.y,testPoints.z);
	return CheckWhetherPointInTheInlierOfRoomFrame(roomFrame,test,distanceThreshold);
}


//不进行错误检查
bool CheckWhetherPointInTheInlierOfRoomFrame(
	std::vector<std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>>>&roomFrame,
	Eigen::Vector3f &testPoints,double distanceThreshold)
{
	size_t boxNum=roomFrame.size();
	int outOfBoxNum=0;
	for (size_t j=0;j<boxNum;j++)
	{
		size_t frameNum=roomFrame[j].size();
		for (size_t k=0;k<frameNum;k++)
		{
			double length=roomFrame[j][k](0)*testPoints(0)+roomFrame[j][k](1)*testPoints(1)
				+roomFrame[j][k](2)*testPoints(2)+roomFrame[j][k](3);
			if (length>=-distanceThreshold)
			{
				continue;
			}
			else
			{
				//	cout<<length<<endl;
				outOfBoxNum++;
				break;
			}
		}
	}
	if (outOfBoxNum==boxNum)
	{
		return false;
	}

	return true;
}

int ComputeOverlap(pcl::search::KdTree<pcl::PointXYZ>::Ptr queryTree,
				   pcl::search::KdTree<pcl::PointXYZ>::Ptr destTree,
				   Eigen::Vector3f &queryCenter,
				   float queryRadius,
				   float inlierDistance, float &overLapRation,bool showResult)
{
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr queryPoints=queryTree->getInputCloud();
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr destPoints=destTree->getInputCloud();
	std::vector<int> neighbor;
	std::vector<float> neighborLength;
	//compute the coarse overlap region
	if (destTree->radiusSearch(pcl::PointXYZ(queryCenter(0),queryCenter(1),queryCenter(2)),queryRadius,
		neighbor,neighborLength)<=0)
	{
		overLapRation=0;
		return 0;
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr neighborPoints(new pcl::PointCloud<pcl::PointXYZ>);
	size_t neighborNum=neighbor.size();
	neighborPoints->resize(neighborNum);
	pcl::PointCloud<pcl::PointXYZ> &ne=*neighborPoints;
	const::pcl::PointCloud<pcl::PointXYZ> &de=*destPoints;
	for (size_t i=0;i<neighborNum;i++)
	{
		memcpy(&ne[i],&de[neighbor[i]],sizeof(float)*3);
	}
	pcl::search::KdTree<pcl::PointXYZ>::Ptr neighborKdTree(new pcl::search::KdTree<pcl::PointXYZ>);
	neighborKdTree->setInputCloud(neighborPoints);
	size_t queryPointsNum=queryPoints->size();
	int count=0;
	for (size_t i=0;i<queryPointsNum;i++)
	{
		if (neighborKdTree->radiusSearch(queryPoints->at(i),inlierDistance,neighbor,neighborLength,1)>0)
		{
			count++;
		}
	}
	overLapRation=double(count)/MIN(queryPointsNum,destPoints->size());

	if (showResult)
	{
		//test
 		boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("test"));
 		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> dest_cloud_color(destPoints,255,255,255);
 		view->addPointCloud(destPoints,dest_cloud_color,"dest_cloud_v1");
 		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> coarse_region_cloud_color(neighborPoints,255,0,0);
 		view->addPointCloud(neighborPoints,coarse_region_cloud_color,"coarse_region_cloud_v1");
 		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> query_cloud_color(queryPoints,0,255,0);
 		view->addPointCloud(queryPoints,query_cloud_color,"query_cloud_v1");
 		cout<<overLapRation<<endl;
 		while (!view->wasStopped())
 		{
 		 	view->spinOnce(100);
 		}
	}


	return 0;
}

bool CheckWheterTwoPointCloudsHaveOverLap(pcl::search::KdTree<pcl::PointXYZ>::Ptr queryTree,
										 pcl::search::KdTree<pcl::PointXYZ>::Ptr destTree,
										 float inlierDistance)
{
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr queryPoints=queryTree->getInputCloud();
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr destPoints=destTree->getInputCloud();
	std::vector<int> neighbor;
	std::vector<float> neighborLength;
	if (queryPoints->size()<destPoints->size())
	{
		size_t queryPointsNum=queryPoints->size();
		for (size_t i=0;i<queryPointsNum;i++)
		{
			if (destTree->radiusSearch(queryPoints->at(i),inlierDistance,neighbor,neighborLength,1)>0)
			{
				return true;
			}
		}
	}
	else
	{
		size_t queryPointsNum=destPoints->size();
		for (size_t i=0;i<queryPointsNum;i++)
		{
			if (queryTree->radiusSearch(destPoints->at(i),inlierDistance,neighbor,neighborLength,1)>0)
			{
				return true;
			}
		}
	}



	//test
// 	boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("test"));
// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> dest_cloud_color(destPoints,255,255,255);
// 	view->addPointCloud(destPoints,dest_cloud_color,"dest_cloud_v1");
// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> query_cloud_color(queryPoints,0,255,0);
// 	view->addPointCloud(queryPoints,query_cloud_color,"query_cloud_v1");
// 
// 	while (!view->wasStopped())
// 	{
// 	 	view->spinOnce(100);
// 	}

	return false;
}


bool CheckWheterTwoPointCloudsHaveOverLap(pcl::search::KdTree<pcl::PointXYZ>::Ptr queryTree,
										  pcl::search::KdTree<pcl::PointXYZ>::Ptr destTree,
										  Eigen::Vector3f &queryCenter,
										  Eigen::Vector3f &destCenter,
										  float queryRadius,
										  float destRadius,
										  float inlierDistance)
{
	if ((queryCenter-destCenter).norm()-queryRadius-destRadius>inlierDistance)
	{
		return false;
	}
	return CheckWheterTwoPointCloudsHaveOverLap(queryTree,destTree,inlierDistance);
}

int ComputeDescriptorVectorForPairLines(PCP::INTERSECTION_LINE &line1,
										PCP::INTERSECTION_LINE &line2,
										Eigen::Vector3f &line1sp1,
										Eigen::Vector3f &line1sp2,
										Eigen::Vector3f &line2sp1,
										Eigen::Vector3f &line2sp2,
										Eigen::VectorXf &descriptor,
										Eigen::Vector3f &newLine1,
										Eigen::Vector3f &newLine2,
										METHODINDEX methodIndex)
{
	//determine the direction if line
	//line1
	float angle1=abs(line1.lineVec.dot(line2sp1));
	float angle2=abs(line1.lineVec.dot(line2sp2));
	Eigen::Vector3f newline2sp1,newline2sp2;
	if (angle1<=angle2)
	{
		newline2sp1=line2sp1;
		newline2sp2=line2sp2;
	}
	else
	{
		newline2sp1=line2sp2;
		newline2sp2=line2sp1;
	}
	newLine2=newline2sp1.cross(newline2sp2);
	//line2
	Eigen::Vector3f newline1sp1,newline1sp2;
	angle1=abs(line2.lineVec.dot(line1sp1));
	angle2=abs(line2.lineVec.dot(line1sp2));
	if (angle1<=angle2)
	{
		newline1sp1=line1sp1;
		newline1sp2=line1sp2;
	}
	else
	{
		newline1sp1=line1sp2;
		newline1sp2=line1sp1;
	}
	newLine1=newline1sp1.cross(newline1sp2);
	if (method22==methodIndex)
	{
		// construct descriptor vector
		//(*p)[0]=lineLength[i][j].length;//distance of two lines
		descriptor[1]=newLine1.dot(newLine2);//angle of two lines
		descriptor[2]=newline1sp1.dot(newline1sp2);//angle of support planes of line1
		descriptor[3]=newline2sp1.dot(newline2sp2);//angle of support planes of line2
		descriptor[4]=newLine1.dot(newline2sp1);//angle of line1 with support planes of line2
		descriptor[5]=newLine1.dot(newline2sp2);//angle of line1 with support planes of line2
		descriptor[6]=newLine2.dot(newline1sp1);//angle of line2 with support planes of line1
		descriptor[7]=newLine2.dot(newline1sp2);//angle of line2 with support planes of line1
	}
	else if (method21==methodIndex)
	{
		// construct descriptor vector
		//(*p)[0]=lineLength[i][j].length;//distance of two lines
		descriptor[1]=newLine1.dot(newLine2);//angle of two lines
		descriptor[2]=newline1sp1.dot(newline1sp2);//angle of support planes of line1
		descriptor[3]=newLine1.dot(line2sp1);//angle of line1 with support planes of line2
		descriptor[4]=newLine2.dot(newline1sp1);//angle of line2 with support planes of line1
		descriptor[5]=newLine2.dot(newline1sp2);//angle of line2 with support planes of line1
	}
	else if (method12==methodIndex)
	{
		// construct descriptor vector
		//(*p)[0]=lineLength[i][j].length;//distance of two lines
		descriptor[1]=newLine1.dot(newLine2);//angle of two lines
		descriptor[2]=newline2sp1.dot(newline2sp2);//angle of support planes of line2
		descriptor[3]=newLine1.dot(newline2sp1);//angle of line1 with support planes of line2
		descriptor[4]=newLine1.dot(newline2sp2);//angle of line1 with support planes of line2
		descriptor[5]=newLine2.dot(line1sp1);//angle of line2 with support planes of line1
	}
	else if (method11==methodIndex)
	{
		// construct descriptor vector
		//(*p)[0]=lineLength[i][j].length;//distance of two lines
		descriptor[1]=newLine1.dot(newLine2);//angle of two lines
		descriptor[2]=newLine1.dot(line2sp1);//angle of line1 with support planes of line2
		descriptor[3]=newLine2.dot(line1sp1);//angle of line2 with support planes of line1
	}
	return 0;
}


//kdtrees8 22
//kdtrees6 12 22-12 21 22-21
//kdtrees4 11 12-11 21-11 22-11
//linesInformation 22 12 22-12 21 22-21 11 12-11 21-11 22-11
int ConstructPairLinesKdTree(std::vector<PCP::INTERSECTION_LINE> &lines,
							 std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>> &planes,
							 std::vector<KdTreeSearchNDim<Eigen::VectorXf,8>> &kdtrees8,
							 std::vector<KdTreeSearchNDim<Eigen::VectorXf,6>> &kdtrees6,
							 std::vector<KdTreeSearchNDim<Eigen::VectorXf,4>> &kdtrees4,
							 std::vector<std::vector<PCP::PAIRLINE>> &linesInformation,
							 float scalar)
{
	if (lines.empty()||planes.empty())
	{
		return -1;
	}
	size_t linesNum=lines.size();
	size_t planeNum=planes.size();
	std::vector<Eigen::Vector3f> suppotPlanes(planeNum);
	for (size_t i=0;i<planeNum;i++)
	{
		suppotPlanes[i]=planes[i].block(0,0,3,1);
	}
	kdtrees8.resize(1);
	kdtrees6.resize(4);
	kdtrees4.resize(4);
	KdTreeSearchNDim<Eigen::VectorXf,8> &kdtree22=kdtrees8[0]; kdtree22.begin();
	KdTreeSearchNDim<Eigen::VectorXf,6> &kdtree12=kdtrees6[0]; kdtree12.begin();
	KdTreeSearchNDim<Eigen::VectorXf,6> &kdtree2212=kdtrees6[1]; kdtree2212.begin();
	KdTreeSearchNDim<Eigen::VectorXf,6> &kdtree21=kdtrees6[2]; kdtree21.begin();
	KdTreeSearchNDim<Eigen::VectorXf,6> &kdtree2221=kdtrees6[3]; kdtree2221.begin();
	KdTreeSearchNDim<Eigen::VectorXf,4> &kdtree11=kdtrees4[0]; kdtree11.begin();
	KdTreeSearchNDim<Eigen::VectorXf,4> &kdtree1211=kdtrees4[1]; kdtree1211.begin();
	KdTreeSearchNDim<Eigen::VectorXf,4> &kdtree2111=kdtrees4[2]; kdtree2111.begin();
	KdTreeSearchNDim<Eigen::VectorXf,4> &kdtree2211=kdtrees4[3]; kdtree2211.begin();
	std::vector<Eigen::VectorXf*> vec22;
	std::vector<Eigen::VectorXf*> vec12;
	std::vector<Eigen::VectorXf*> vec2212;
	std::vector<Eigen::VectorXf*> vec21;
	std::vector<Eigen::VectorXf*> vec2221;
	std::vector<Eigen::VectorXf*> vec11;
	std::vector<Eigen::VectorXf*> vec1211;
	std::vector<Eigen::VectorXf*> vec2111;
	std::vector<Eigen::VectorXf*> vec2211;
	linesInformation.resize(9);
	std::vector<PCP::PAIRLINE> &lf22=linesInformation[0];
	std::vector<PCP::PAIRLINE> &lf12=linesInformation[1];
	std::vector<PCP::PAIRLINE> &lf2212=linesInformation[2];
	std::vector<PCP::PAIRLINE> &lf21=linesInformation[3];
	std::vector<PCP::PAIRLINE> &lf2221=linesInformation[4];
	std::vector<PCP::PAIRLINE> &lf11=linesInformation[5];
	std::vector<PCP::PAIRLINE> &lf1211=linesInformation[6];
	std::vector<PCP::PAIRLINE> &lf2111=linesInformation[7];
	std::vector<PCP::PAIRLINE> &lf2211=linesInformation[8];

	struct NEARST_POINTS_TWOLINE 
	{
		Eigen::Vector3f points1;
		Eigen::Vector3f points2;
		double length;
	};
	std::vector<std::vector<NEARST_POINTS_TWOLINE>> lineLength(linesNum);
	float minLength=0.1/scalar;
	float angleThresh=cos(10.0/180*M_PI);
	for (size_t i=0;i<linesNum;i++)
	{
		//cout<<i<<endl;
		lineLength[i].resize(linesNum);
		INTERSECTION_LINE &line1=lines[i];
		for (size_t j=0;j<linesNum;j++)
		{
			if (i==j)
			{
				continue;
			}
			INTERSECTION_LINE &line2=lines[j];
			//compute length of each pair of lines
			if (i>j)
			{
				lineLength[i][j].points1=lineLength[j][i].points2;
				lineLength[i][j].points2=lineLength[j][i].points1;
				lineLength[i][j].length=lineLength[j][i].length;
			}
			else if (i<j)
			{
				if (0!=PCP::ComputeNearstTwoPointsOfTwo3DLine(line1.lineVec,line1.linePoint,
					line2.lineVec,line2.linePoint,lineLength[i][j].points1,
					lineLength[i][j].points2,lineLength[i][j].length))
				{
					lineLength[i][j].length=-1;
				}
				lineLength[i][j].length=lineLength[i][j].length/scalar;
			}
			//discard some lines
			if (abs(line1.lineVec.dot(line2.lineVec))>angleThresh)
			{
				continue;
			}
			PAIRLINE pairLine;
			pairLine.linePoints1=lineLength[i][j].points1;
			pairLine.linePoints2=lineLength[i][j].points2;
			pairLine.originalIndex1=i;
			pairLine.originalIndex2=j;
			//construct kdTree
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
														pairLine.lineVec1,pairLine.lineVec2,method22);
					
					(*p22)[0]=lineLength[i][j].length;
					pairLine.descriptor=*p22;
					lf22.push_back(pairLine);
					vec22.push_back(p22);
					kdtree22.add_point(p22);
					

					//22-21
					Eigen::VectorXf *p2221_1=new Eigen::VectorXf(6);
					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],suppotPlanes[l1sp2],
						suppotPlanes[l2sp1],line2.lineVec.cross(suppotPlanes[l2sp1]),*p2221_1,
						pairLine.lineVec1,pairLine.lineVec2,method21);
					(*p2221_1)[0]=lineLength[i][j].length;
					pairLine.descriptor=*p2221_1;
					lf2221.push_back(pairLine);

					Eigen::VectorXf *p2221_2=new Eigen::VectorXf(6);
					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],suppotPlanes[l1sp2],
						suppotPlanes[l2sp2],line2.lineVec.cross(suppotPlanes[l2sp2]),*p2221_2,
						pairLine.lineVec1,pairLine.lineVec2,method21);
					(*p2221_2)[0]=lineLength[i][j].length;
					pairLine.descriptor=*p2221_2;
					lf2221.push_back(pairLine);

					Eigen::VectorXf *p2221_3=new Eigen::VectorXf(6);
					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],suppotPlanes[l1sp2],
						suppotPlanes[l2sp1],line2.lineVec.cross(-suppotPlanes[l2sp1]),*p2221_3,
						pairLine.lineVec1,pairLine.lineVec2,method21);
					(*p2221_3)[0]=lineLength[i][j].length;
					pairLine.descriptor=*p2221_3;
					lf2221.push_back(pairLine);

					Eigen::VectorXf *p2221_4=new Eigen::VectorXf(6);
					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],suppotPlanes[l1sp2],
						suppotPlanes[l2sp2],line2.lineVec.cross(-suppotPlanes[l2sp2]),*p2221_4,
						pairLine.lineVec1,pairLine.lineVec2,method21);
					(*p2221_4)[0]=lineLength[i][j].length;
					pairLine.descriptor=*p2221_4;
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
					Eigen::VectorXf *p2212_1=new Eigen::VectorXf(6);
					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],line1.lineVec.cross(suppotPlanes[l1sp1]),
						suppotPlanes[l2sp1],suppotPlanes[l2sp2],*p2212_1,
						pairLine.lineVec1,pairLine.lineVec2,method12);
					(*p2212_1)[0]=lineLength[i][j].length;
					pairLine.descriptor=*p2212_1;
					lf2212.push_back(pairLine);

					Eigen::VectorXf *p2212_2=new Eigen::VectorXf(6);
					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp2],line1.lineVec.cross(suppotPlanes[l1sp2]),
						suppotPlanes[l2sp1],suppotPlanes[l2sp2],*p2212_2,
						pairLine.lineVec1,pairLine.lineVec2,method12);
					(*p2212_2)[0]=lineLength[i][j].length;
					pairLine.descriptor=*p2212_2;
					lf2212.push_back(pairLine);

					Eigen::VectorXf *p2212_3=new Eigen::VectorXf(6);
					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],line1.lineVec.cross(-suppotPlanes[l1sp1]),
						suppotPlanes[l2sp1],suppotPlanes[l2sp2],*p2212_3,
						pairLine.lineVec1,pairLine.lineVec2,method12);
					(*p2212_3)[0]=lineLength[i][j].length;
					pairLine.descriptor=*p2212_3;
					lf2212.push_back(pairLine);

					Eigen::VectorXf *p2212_4=new Eigen::VectorXf(6);
					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp2],line1.lineVec.cross(-suppotPlanes[l1sp2]),
						suppotPlanes[l2sp1],suppotPlanes[l2sp2],*p2212_4,
						pairLine.lineVec1,pairLine.lineVec2,method12);
					(*p2212_4)[0]=lineLength[i][j].length;
					pairLine.descriptor=*p2212_4;
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
				}
				else//21
				{
					int l1sp1=line1.supportPlanes[0];
					int l1sp2=line1.supportPlanes[1];
					int l2sp1=line2.supportPlanes[0];
					//21
					Eigen::VectorXf *p21=new Eigen::VectorXf(6);
					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],suppotPlanes[l1sp2],
						suppotPlanes[l2sp1],line2.lineVec.cross(suppotPlanes[l2sp1]),*p21,
						pairLine.lineVec1,pairLine.lineVec2,method21);
					(*p21)[0]=lineLength[i][j].length;
					pairLine.descriptor=*p21;
					lf21.push_back(pairLine);

					Eigen::VectorXf *p21_1=new Eigen::VectorXf(6);
					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],suppotPlanes[l1sp2],
						suppotPlanes[l2sp1],line2.lineVec.cross(-suppotPlanes[l2sp1]),*p21_1,
						pairLine.lineVec1,pairLine.lineVec2,method21);
					(*p21_1)[0]=lineLength[i][j].length;
					pairLine.descriptor=*p21_1;
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
						pairLine.lineVec1,pairLine.lineVec2,method12);
					(*p12)[0]=lineLength[i][j].length;
					pairLine.descriptor=*p12;
					lf12.push_back(pairLine);

					Eigen::VectorXf *p12_1=new Eigen::VectorXf(6);
					ComputeDescriptorVectorForPairLines(line1,line2,suppotPlanes[l1sp1],line1.lineVec.cross(-suppotPlanes[l1sp1]),
						suppotPlanes[l2sp1],suppotPlanes[l2sp2],*p12_1,
						pairLine.lineVec1,pairLine.lineVec2,method12);
					(*p12_1)[0]=lineLength[i][j].length;
					pairLine.descriptor=*p12_1;
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

				}
				else//11
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
	}
	kdtree11.end();kdtree12.end();kdtree1211.end();kdtree21.end();kdtree2111.end();
	kdtree22.end();kdtree2212.end();kdtree2221.end();kdtree2211.end();

	std::vector<std::vector<Eigen::VectorXf*>*>vec;
	vec.push_back(&vec11);
	vec.push_back(&vec12);
	vec.push_back(&vec1211);
	vec.push_back(&vec21);
	vec.push_back(&vec2111);
	vec.push_back(&vec22);
	vec.push_back(&vec2211);
	vec.push_back(&vec2212);
	vec.push_back(&vec2221);
	int count=0;
	for (size_t i=0;i<vec.size();i++)
	{
		for (size_t j=0;j<(*vec[i]).size();j++)
		{
			delete (*vec[i])[j];
			count++;
		}
	}
	cout<<"total descriptors:"<<count<<endl;
	return 0;
}

float g_angleThreshold=0.0076;//5 degree
bool EnforceSimilarity (const pcl::PointXYZINormal& point_a, const pcl::PointXYZINormal& point_b, float squared_distance)
{
	Eigen::VectorXf temp(3);
	temp[0]=point_a.normal_x-point_b.normal_x;
	temp[1]=point_a.normal_y-point_b.normal_y;
	temp[2]=point_a.normal_z-point_b.normal_z;
	if (temp.squaredNorm()<g_angleThreshold)
	{
		return true;
	}
	return false;
}

int ClusterTransformation(std::vector<Eigen::Matrix3f>&Rs,
						  std::vector<Eigen::Vector3f>&Ts,
						  float distanceThreshold,
						  float angleThreshold,
						  pcl::IndicesClusters &clusters)
{
	if (Rs.empty()||Rs.size()!=Ts.size())
	{
		return -1;
	}
	//////////////////////////////////////////////////////////////////////////
	size_t transNum=Rs.size();
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr trans(new pcl::PointCloud<pcl::PointXYZINormal>);
	trans->resize(transNum);
	for (size_t i=0;i<transNum;i++)
	{
		(*trans)[i].x=Ts[i][0];
		(*trans)[i].y=Ts[i][1];
		(*trans)[i].z=Ts[i][2];
		Eigen::Transform<float, 3, Eigen::Affine> R(Rs[i]);
		pcl::getEulerAngles<float>(R,(*trans)[i].normal_x,(*trans)[i].normal_y,(*trans)[i].normal_z);
	}

	pcl::ConditionalEuclideanClustering<pcl::PointXYZINormal> cec (true);
	cec.setInputCloud (trans);
	cec.setConditionFunction (&EnforceSimilarity);
	// Points within this distance from one another are going to need to validate the enforceIntensitySimilarity function to be part of the same cluster:
	cec.setClusterTolerance (distanceThreshold);
	g_angleThreshold=angleThreshold;
	// Size constraints for the clusters:
	cec.setMinClusterSize (1);
	cec.setMaxClusterSize (transNum);
	// The resulting clusters (an array of pointindices):
	cec.segment (clusters);
	return 0;
}

//upright为z轴正向
int DetectFloorsFromLaserScan(std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>> &planes,
				std::vector<int> &floorIndex)
{
	floorIndex.clear();
	//
	Eigen::Vector3f upRight(0,0,1);
	float thres=cos(5.0/180*M_PI);
	std::vector<LENGTHINDEX> length;
	for (size_t i=0;i<planes.size();i++)
	{
		if (upRight.dot(planes[i].block(0,0,3,1))>thres)
		{
			LENGTHINDEX tempLength;
			tempLength.index=i;
			tempLength.length=-planes[i](3);
			length.push_back(tempLength);
		}
	}
	sort(length.begin(),length.end(),myCompareLess);
	floorIndex.push_back(length.front().index);
	for (size_t i=1;i<length.size();i++)
	{
		if (abs(length[0].length-length[i].length)>0.02)
		{
			break;
		}
		floorIndex.push_back(length[i].index);
	}
	return 0;
}

int FitPlane(pcl::PointCloud<pcl::PointXYZ>&points,Eigen::Vector4f &plane)
{
	if (points.size()<3)
	{
		return -1;
	}
	//////////////////////////////////////////////////////////////////////////
	int pointsNum=points.size();
	cv::Mat matrix(pointsNum,4,CV_32F,cv::Scalar(1));
	int count=0;
	for (int i=0;i<pointsNum;i++)
	{
		if (cvIsNaN(points[i].x+points[i].y+points[i].z)||cvIsInf(points[i].x+points[i].y+points[i].z))
		{
			continue;
		}
		float *p=matrix.ptr<float>(count++);
		p[0]=points[i].x;
		p[1]=points[i].y;
		p[2]=points[i].z;
	}
	cv::Mat planeParam;
	cv::SVD::solveZ(matrix.rowRange(0,count),planeParam);
	for (size_t i=0;i<4;i++)
	{
		plane[i]=planeParam.at<float>(i);
	}
	double norm=plane.block(0,0,3,1).norm();
	plane=plane/norm;
	return 0;
}

//对一个空间平面上的点进行坐标变换，使平面上的点的坐标的z值为0
//point 平面上一点，坐标变换后该点为原点
//vec 转换的z轴向量,新的坐标系的z轴和这个向量重合
//R    输出的变换矩阵
//变换完之后,新的点的坐标是 xNew=R*xOld
int CoordinateTransformation(pcl::PointXYZ &point,Eigen::Vector3f &vecZ,Eigen::Matrix4f &R)
{
	//////////////////////////////////////////////////////////////////////////
	//法向量
	float a=vecZ(0);
	float b=vecZ(1);
	float c=vecZ(2);
	//
	Eigen::Matrix4f T=Eigen::Matrix4f::Identity();
	T(3,0)=-point.x;
	T(3,1)=-point.y;
	T(3,2)=-point.z;
	Eigen::Matrix4f RxA=Eigen::Matrix4f::Identity();
	float sinA=b/sqrt(b*b+c*c);
	float cosA=c/sqrt(b*b+c*c);
	RxA(1,1)=cosA;
	RxA(1,2)=sinA;
	RxA(2,1)=-sinA;
	RxA(2,2)=cosA;
	float sinB=-a/sqrt(a*a+b*b+c*c);
	float cosB=sqrt(b*b+c*c)/sqrt(a*a+b*b+c*c);
	Eigen::Matrix4f RyB=Eigen::Matrix4f::Identity();
	RyB(0,0)=cosB;
	RyB(0,2)=-sinB;
	RyB(2,0)=sinB;
	RyB(2,2)=cosB;
	R=T*RxA*RyB;
	R.transposeInPlace();//以v为Z轴的坐标系的点=R*P1

	return 0;
}


int ComputeTotalAreaOfTwoCoplanarPlanes(const pcl::PointCloud<pcl::PointXYZ>& points1,
									  const pcl::PointCloud<pcl::PointXYZ>& points2,
									  const Eigen::Vector4f &mappedPlane,
									  float& totalArea,
									  float *pGridSize
									  )
{
	//将拟合点投影到平面
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempPoints(new pcl::PointCloud<pcl::PointXYZ>);
	*tempPoints=points1;
	*tempPoints+=points2;
	std::vector<Eigen::Vector3f> projectPoints;
	if (0!=ProjectPoints2Plane(*tempPoints,mappedPlane,projectPoints))
	{
		return -1;
	}
	//将投影到平面的点转换到平面坐标系
	int pointsNum=projectPoints.size();
	Eigen::Matrix4f trans;
	if (0!=CoordinateTransformation(pcl::PointXYZ(projectPoints[0][0],projectPoints[0][1],projectPoints[0][2]),
									Eigen::Vector3f(mappedPlane(0),mappedPlane(1),mappedPlane(2)),trans))
	{
		return -3;
	}
	Eigen::Matrix3f R=trans.block(0,0,3,3);
	Eigen::Vector3f T=trans.block(0,3,3,1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointsTrans(new pcl::PointCloud<pcl::PointXYZ>);
	pointsTrans->resize(pointsNum);
	for (int i=0;i<pointsNum;i++)
	{
		Eigen::Vector3f temp=R*projectPoints[i]+T;
		memcpy(&(*pointsTrans)[i],&temp,sizeof(float)*3);
	}
	float gridSize=0.05;
	if (pGridSize)
	{
		gridSize=*pGridSize;
	}
	pcl::VoxelGrid<pcl::PointXYZ> grid; //VoxelGrid 把一个给定的点云，聚集在一个局部的3D网格上,并下采样和滤波点云数据
	grid.setLeafSize (gridSize, gridSize, gridSize); //设置体元网格的叶子大小
	grid.setInputCloud(pointsTrans);
	pcl::PointCloud<pcl::PointXYZ>::Ptr downSamplePoints(new pcl::PointCloud<pcl::PointXYZ>);
	grid.filter (*downSamplePoints); //下采样和滤波，并存储在src中
	totalArea=downSamplePoints->size()*gridSize*gridSize;

	//test
//  	cout<<downSamplePoints->size()<<" totalArea: "<<downSamplePoints->size()*gridSize*gridSize<<endl;
//  	boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("test"));
//  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(pointsTrans,255,0,0);
//  	view->addPointCloud(pointsTrans,sources_cloud_color,cv::format("downsample"));
//  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(downSamplePoints,0,255,0);
//  	view->addPointCloud(downSamplePoints,target_cloud_color,cv::format("source"));
// 
//  	while (!view->wasStopped())
//  	{
//  		view->spinOnce(100);
//  
//  	}

	return 0;
}

int ProjectPoints2PlaneCoordinate( pcl::PointCloud<pcl::PointXYZ>& pointsIn,
								  std::vector<cv::Point2f> &pointsOut,
								   Eigen::Vector4f &plane)
{
	if (pointsIn.empty())
	{
		return -1;
	}
	//////////////////////////////////////////////////////////////////////////
	//将拟合点投影到平面
	std::vector<Eigen::Vector3f> projectPoints;
	if (0!=ProjectPoints2Plane(pointsIn,plane,projectPoints))
	{
		return -1;
	}
	//将投影到平面的点转换到平面坐标系
	int pointsNum=projectPoints.size();
	Eigen::Matrix4f trans;
	if (0!=CoordinateTransformation(pcl::PointXYZ(projectPoints[0][0],projectPoints[0][1],projectPoints[0][2]),
		Eigen::Vector3f(plane(0),plane(1),plane(2)),trans))
	{
		return -3;
	}
	Eigen::Matrix3f R=trans.block(0,0,3,3);
	Eigen::Vector3f T=trans.block(0,3,3,1);
	pointsOut.resize(pointsNum);
	for (int i=0;i<pointsNum;i++)
	{
		Eigen::Vector3f temp=R*projectPoints[i]+T;
		memcpy(&(pointsOut)[i],&temp,sizeof(float)*2);
	}
	return 0;
}

int ComputeInformationOfTwoCoplanarPlanes(const pcl::PointCloud<pcl::PointXYZ>& points1,
										const pcl::PointCloud<pcl::PointXYZ>& points2,
										const Eigen::Vector4f &mappedPlane,
										float* pIntersectArea,
										float* pTotalArea,
										float* pPoints1Area,
										float* pPoints2Area,
										float *pGridSizeX,
										float *pGridSizeY
										)
{
	//将拟合点投影到平面
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempPoints(new pcl::PointCloud<pcl::PointXYZ>);
	*tempPoints=points1;
	*tempPoints+=points2;
	std::vector<Eigen::Vector3f> projectPoints;
	if (0!=ProjectPoints2Plane(*tempPoints,mappedPlane,projectPoints))
	{
		return -1;
	}
	int points1Num=points1.size();
	int points2Num=points2.size();
	std::vector<int>projectPointsIndex(projectPoints.size());
	for (size_t i=0;i<projectPoints.size();i++)
	{
		if (i<points1Num)
		{
			projectPointsIndex[i]=1;
		}
		else
		{
			projectPointsIndex[i]=2;
		}
	}
	//将投影到平面的点转换到平面坐标系
	int pointsNum=projectPoints.size();
	Eigen::Matrix4f trans;
	if (0!=CoordinateTransformation(pcl::PointXYZ(projectPoints[0][0],projectPoints[0][1],projectPoints[0][2]),
		Eigen::Vector3f(mappedPlane(0),mappedPlane(1),mappedPlane(2)),trans))
	{
		return -3;
	}
	Eigen::Matrix3f R=trans.block(0,0,3,3);
	Eigen::Vector3f T=trans.block(0,3,3,1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointsTrans(new pcl::PointCloud<pcl::PointXYZ>);
	pointsTrans->resize(pointsNum);
//	#pragma omp parallel for
	for (int i=0;i<pointsNum;i++)
	{
		Eigen::Vector3f temp=R*projectPoints[i]+T;
		memcpy(&(*pointsTrans)[i],&temp,sizeof(float)*3);
	}
	//创建网格
	pcl::PointXYZ minPoint,maxPoint;
	pcl::getMinMax3D(*pointsTrans,minPoint,maxPoint);
	float xGridSize=0.05;
	float yGridSize=0.05;
	if (pGridSizeX)
	{
		xGridSize=*pGridSizeX;
	}
	if (pGridSizeY)
	{
		yGridSize=*pGridSizeY;
	}

	int xDivNum=ceil((maxPoint.x-minPoint.x)/xGridSize)+1;
	int yDivNum=ceil((maxPoint.y-minPoint.y)/yGridSize)+1;
	std::vector<std::vector<std::pair<bool,bool>>> lightMap(yDivNum);
	for (int i=0;i<yDivNum;i++)
	{
		lightMap[i].resize(xDivNum);
		for (int j=0;j<xDivNum;j++)
		{
			lightMap[i][j]=std::pair<bool,bool>(false,false);
		}
	}
	//omp_set_num_threads(1);
//	#pragma omp parallel for
	for (int i=0;i<pointsNum;i++)
	{
		int x=floor(((*pointsTrans)[i].x-minPoint.x)/xGridSize);
		int y=floor(((*pointsTrans)[i].y-minPoint.y)/yGridSize);
		if (i<points1Num)
		{
			lightMap[y][x].first=true;
		}
		else
		{
			lightMap[y][x].second=true;
		}
	}
	int points1Count=0,points2Count=0,intersectCount=0;
//	#pragma omp parallel for
	for (int i=0;i<yDivNum;i++)
	{
		for (int j=0;j<xDivNum;j++)
		{
			if (lightMap[i][j].first&&lightMap[i][j].second)
			{
				//#pragma omp atomic
				intersectCount++;
				//#pragma omp atomic
				points1Count++;
				//#pragma omp atomic
				points2Count++;
			}
			else if (lightMap[i][j].first)
			{
				//#pragma omp atomic
				points1Count++;
			}
			else if (lightMap[i][j].second)
			{
				//#pragma omp atomic
				points2Count++;
			}
		}
	}
	float gridArea=xGridSize*yGridSize;
	if (pIntersectArea)
	{
		*pIntersectArea=intersectCount*gridArea;
	}
	if (pTotalArea)
	{
		*pTotalArea=(points1Count+points2Count-intersectCount)*gridArea;
	}
	if (pPoints1Area)
	{
		*pPoints1Area=points1Count*gridArea;
	}
	if (pPoints2Area)
	{
		*pPoints2Area=points2Count*gridArea;
	}
	//test
//  	cout<<"points1Count:"<<points1Count<<" area: "<<points1Count*gridArea<<endl;
//  	cout<<"points2Count:"<<points2Count<<" area: "<<points2Count*gridArea<<endl;
//  	cout<<"totalCount:"<<points1Count+points2Count-intersectCount<<" area: "<<(points1Count+points2Count-intersectCount)*gridArea<<endl;
//  	cout<<"intersectCount:"<<intersectCount<<" area: "<<intersectCount*gridArea<<endl;
//  	boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("test"));
//  	pcl::PointCloud<pcl::PointXYZ>::Ptr t1(new pcl::PointCloud<pcl::PointXYZ>);
//  	pcl::PointCloud<pcl::PointXYZ>::Ptr t2(new pcl::PointCloud<pcl::PointXYZ>);
//  	*t1=points1;
//  	*t2=points2;
//  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color1(t1,255,0,0);
//  	view->addPointCloud(t1,sources_cloud_color1,cv::format("source1"));
//  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color2(t2,0,255,0);
//  	view->addPointCloud(t2,sources_cloud_color2,cv::format("source2"));
//  	//pcl::PointCloud<pcl::PointXYZ>::Ptr tempProjetPoints(new pcl::PointCloud<pcl::PointXYZ>);
//  	//ExchnageBetweentPCLPointXYZwithEigenVector3f(*tempProjetPoints,projectPoints);
//  	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> project_cloud_color(tempProjetPoints,0,0,255);
//  	//view->addPointCloud(tempProjetPoints,project_cloud_color,cv::format("projectPoints"));
//  	pcl::ModelCoefficients modelPlane;
//  	modelPlane.values.push_back(mappedPlane[0]);
//  	modelPlane.values.push_back(mappedPlane[1]);
//  	modelPlane.values.push_back(mappedPlane[2]);
//  	modelPlane.values.push_back(mappedPlane[3]);
//  	view->addPlane(modelPlane,points1[0].x,points1[0].y,points1[0].z);
//  	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(pointsTrans,0,255,0);
//  	view->addPointCloud(pointsTrans,target_cloud_color,cv::format("source"));
//  	while (!view->wasStopped())
//  	{
//  		view->spinOnce(100);
//  	}
//  	cv::Mat tempMat(yDivNum,xDivNum,CV_8UC3,cv::Scalar(0,0,0));
//  	for (int i=0;i<yDivNum;i++)
//  	{
//  		for (int j=0;j<xDivNum;j++)
//  		{
//  			if (lightMap[i][j].first&&lightMap[i][j].second)
//  			{
//  				tempMat.at<cv::Vec3b>(i,j)=cv::Vec3b(0,255,0);
//  			}
//  			else if (lightMap[i][j].first)
//  			{
//  				tempMat.at<cv::Vec3b>(i,j)=cv::Vec3b(255,255,0);
//  			}
//  			else if (lightMap[i][j].second)
//  			{
//  				tempMat.at<cv::Vec3b>(i,j)=cv::Vec3b(0,255,255);
//  			}
//  		}
//  	}
	return 0;
}

int CombinePlanes(std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>> &planes,
				std::vector<std::vector<int>>&combinedPlanes,
				float distanceTh,float angleTh)
{
	if (planes.empty())
	{
		return -1;
	}
	combinedPlanes.clear();
	std::vector<bool> isCombined(planes.size(),false);
	for (size_t i=0;i<planes.size();i++)
	{
		if (isCombined[i])
		{
			continue;
		}
		std::vector<int> currentCombined;
		currentCombined.push_back(i);
		Eigen::Vector4f plane1=planes[i];
		for (size_t j=i+1;j<planes.size();j++)
		{
			if (isCombined[j])
			{
				continue;
			}
			Eigen::Vector4f plane2=planes[j];
			//check angle
			float dotSum=plane1.block(0,0,3,1).dot(plane2.block(0,0,3,1));
			if (abs(dotSum)>cos(M_PI_2-angleTh))
			{
				if (dotSum<0)
				{
					plane2=-plane2;
				}
				//check distance
				if ((plane1[3]-plane2[3])>distanceTh)
				{
					continue;
				}
				//combine
				currentCombined.push_back(j);
				isCombined[j]=true;
			}
			isCombined[i]=true;
			combinedPlanes.push_back(currentCombined);
		}
	}
	return 0;
}




// int RansacExtractPlanes(pcl::PointCloud<pcl::PointXYZ>& points,
// 						std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>>&planes,
// 						std::vector<std::vector<int>>&supportPlanePointsIndex,
// 						float lengthThresh,
// 						float normalThresh,
// 						int minPointsNum)
// {
// 	if (points.empty())
// 	{
// 		return -1;
// 	}
// 	//////////////////////////////////////////////////////////////////////////
// 	pcl::PointXYZ minPoint,maxPoint;
// 	pcl::getMinMax3D(points,minPoint,maxPoint);
// 
// 	PointCloud pc;
// 	// fill or load point cloud from file
// 	pc.resize(points.size());
// 	for (size_t i=0;i<points.size();i++)
// 	{
// 		pc[i]=Point(Vec3f(points[i].x,points[i].y,points[i].z));
// 	}
// 	// don't forget to set the bbox in pc
// 	pc.setBBox(Vec3f(minPoint.x,minPoint.y,minPoint.z),Vec3f(maxPoint.x,maxPoint.y,maxPoint.z));
// 	pc.calcNormals(0.05);
// 	RansacShapeDetector::Options ransacOptions;
// 
// 
// 	ransacOptions.m_epsilon = 0.001f * pc.getScale(); // set distance threshold to .01f of bounding box width
// 	// NOTE: Internally the distance threshold is taken as 3 * ransacOptions.m_epsilon!!!
// 	ransacOptions.m_bitmapEpsilon = 0.02f * pc.getScale(); // set bitmap resolution to .02f of bounding box width
// 	// NOTE: This threshold is NOT multiplied internally!
// 	ransacOptions.m_normalThresh = 0.9f; // this is the cos of the maximal normal deviation
// 	ransacOptions.m_minSupport = 300; // this is the minimal numer of points required for a primitive
// 	ransacOptions.m_probability = 0.001f; // this is the "probability" with which a primitive is overlooked
// 
// 
// // 	ransacOptions.m_epsilon = lengthThresh; // set distance threshold to .01f of bounding box width
// // 	// NOTE: Internally the distance threshold is taken as 3 * ransacOptions.m_epsilon!!!
// // 	ransacOptions.m_bitmapEpsilon = 0.02f * pc.getScale(); // set bitmap resolution to .02f of bounding box width
// // 	// NOTE: This threshold is NOT multiplied internally!
// // 	ransacOptions.m_normalThresh = normalThresh; // this is the cos of the maximal normal deviation
// // 	ransacOptions.m_minSupport = minPointsNum; // this is the minimal numer of points required for a primitive
// // 	ransacOptions.m_probability = 0.001f; // this is the "probability" with which a primitive is overlooked
// 
// 	RansacShapeDetector detector(ransacOptions); // the detector object
// 
// 	// set which primitives are to be detected by adding the respective constructors
// 	detector.Add(new PlanePrimitiveShapeConstructor());
// 
// 	MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes; // stores the detected shapes
// 	// returns number of unassigned points
// 	// the array shapes is filled with pointers to the detected shapes
// 	// the second element per shapes gives the number of points assigned to that primitive (the support)
// 	// the points belonging to the first shape (shapes[0]) have been sorted to the end of pc,
// 	// i.e. into the range [ pc.size() - shapes[0].second, pc.size() )
// 	// the points of shape i are found in the range
// 	// [ pc.size() - \sum_{j=0..i} shapes[j].second, pc.size() - \sum_{j=0..i-1} shapes[j].second )
// 	size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes); // run detection
// 	for (size_t i=0;i<pc.size();i++)
// 	{
// 		//std::memcpy(&points[i],&pc[i],sizeof(float)*3);
// 		//points[i]=pcl::PointXYZ(pc[i][0],pc[i][1],pc[i][2]);
// 		cout<<i<<"   "<<(points)[i]<<"     "<<pc[i][0]<<endl;
// 	}
// 	char tt;
// 	cin>>tt;
// 	int index=pc.size()-1;
// 	planes.resize(shapes.size());
// 	supportPlanePointsIndex.resize(shapes.size());
// #define TEST
// #ifdef TEST
// 	boost::shared_ptr<pcl::visualization::PCLVisualizer> view1 (new pcl::visualization::PCLVisualizer("3DSC test"));
// #endif // TEST
// 
// 	cout<<shapes.size()<<endl;
// 	for (size_t i=0;i<shapes.size();i++)
// 	{
// 		supportPlanePointsIndex.clear();
// 		if (0==shapes[i].first->Identifier())//plane
// 		{
// 			pcl::PointCloud<pcl::PointXYZ>::Ptr  currentPoints(new pcl::PointCloud<pcl::PointXYZ>);
// 			currentPoints->resize(shapes[i].second);
// 			cout<<currentPoints->size()<<endl;
// 			for (size_t j=0;j<currentPoints->size();j++,index--)
// 			{
// 				(*currentPoints)[j]=points[index];
// 				supportPlanePointsIndex[i].push_back(index);
// 				cout<<index<<"  ";
// 				cout<<(*currentPoints)[j]<<"      "<<points[index]<<endl;
// 			}
// 			cv::Mat planeParam;
// 			FitPlane(*currentPoints,planes[i]);
// #ifdef TEST
// 		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(currentPoints,rand()%255,rand()%255,rand()%255);
// 		view1->addPointCloud(currentPoints,sources_cloud_color,cv::format("sources_cloud_%d",i));
// #endif // TEST
// 
// 		}
// 		else
// 		{
// 			index-=shapes[i].second;
// 		}
// 	}
// #ifdef TEST
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr  currentPoints(new pcl::PointCloud<pcl::PointXYZ>);
// 	currentPoints->resize(remaining);
// 	for (size_t j=0;j<currentPoints->size();j++,index--)
// 	{
// 		(*currentPoints)[j]=points[index];
// 	}
// 	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(currentPoints,255,255,255);
// 	view1->addPointCloud(currentPoints,sources_cloud_color,cv::format("remaing_points"));
// 	while (!view1->wasStopped())
// 	{
// 		view1->spinOnce(100);
// 	}
// #endif // TEST
// 
// 	return 0;
// }

}//namespace PCP