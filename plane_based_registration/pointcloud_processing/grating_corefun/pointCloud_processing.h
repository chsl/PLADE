#ifndef POINTCLOUD_PROCESSING_ALGO
#define POINTCLOUD_PROCESSING_ALGO

#ifdef POINTCLOUD_PROCESSING_EXPORTS
#define POINTCLOUD_PROCESSING_API __declspec(dllexport)
#else
#define POINTCLOUD_PROCESSING_API __declspec(dllimport)
#endif


//一些不好处理的警告
#pragma warning( disable: 4251 ) //类模板导出问题
#pragma warning( disable: 4723 ) //在generateNaN中使用的用于产生NAN数引起的
#pragma warning( disable: 4800 )

#include <iostream>
#include <math.h>
#include <map>
#include <list>
#include <cmath>
#include <vector>
#include <algorithm> 
#include <string>
#include <fstream>
#include <tchar.h>
#include <Eigen/Dense>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h> //pcd文件输入/输出
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h> //ICP(iterative closest point)配准
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv2/core/core_c.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cxcore.hpp>
#include <opencv2/core/core.hpp>

#include <ANN.h>



using namespace std;

namespace PCP
{

//一些数学常量的定义
 #define M_E        2.71828182845904523536
 #define M_LOG2E    1.44269504088896340736
 #define M_LOG10E   0.434294481903251827651
 #define M_LN2      0.693147180559945309417
 #define M_LN10     2.30258509299404568402
 #define M_PI       3.14159265358979323846
 #define M_PI_2     1.57079632679489661923
 #define M_PI_4     0.785398163397448309616
 #define M_1_PI     0.318309886183790671538
 #define M_2_PI     0.636619772367581343076
 #define M_2_SQRTPI 1.12837916709551257390
 #define M_SQRT2    1.41421356237309504880
 #define M_SQRT1_2  0.707106781186547524401
 #define M_2PI      6.283185307179586


inline double startTime()
{
	return (double)cv::getTickCount();
}

inline double stopTime(double t)
{
	return ((double)cv::getTickCount() - t)/cv::getTickFrequency()*1000;
}

//代码运行时间的测试
 #ifdef SHOW_RUNTIME_DIGLOG//若想使用对话框弹出运行时间，请在项目属性中定义此宏
 #define SHOW_RUN_TIME_MEHOD 1//使用对话框显示运行时间
 #else
 #define SHOW_RUN_TIME_MEHOD 2//
 #endif //
 
 #define INIT_RUN_TIME  bool show_run_time=true;double run_time =PCP::startTime();
 #define NO_SHOW_RUN_TIME show_run_time=false;
 #define RUN_TIME       if(show_run_time) {cout<<__LINE__<<"th line, run time(ms):"<<PCP::stopTime(run_time)<<endl;}

//函数返回值说明
//   -1                    传入参数错误
//    0                    正常返回

struct INTERSECTION_POINT  
{
	pcl::PointXYZ intersectionPoint;
	std::vector<int> planeIndex;
	std::vector<Eigen::Vector3f> intersectionLines;
};

struct LENGTHINDEX
{
	float length;
	int index;
};

struct INTERSECTION_LINE
{
	Eigen::Vector3f lineVec;
	Eigen::Vector3f linePoint;
	std::vector<int> supportPlanes;
	std::vector<double> confidence;
	std::vector<double> distance;
	double          planeAngle;
	bool      isIntersectionLine;
};

enum METHODINDEX
{
	method11,
	method12,
	method21,
	method22
};

struct PAIRLINE
{
	Eigen::Vector3f lineVec1;
	Eigen::Vector3f lineVec2;
	Eigen::Vector3f linePoints1;
	Eigen::Vector3f linePoints2;
	int originalIndex1;
	int originalIndex2;
	Eigen::VectorXf descriptor;
};

//比较函数
inline bool myCompare(const LENGTHINDEX&l1,const LENGTHINDEX &l2)
{
	if (l1.length<l2.length)
	{
		return true;
	}
	return false;
}
inline bool myCompareLess(const LENGTHINDEX&l1,const LENGTHINDEX &l2)
{
	if (l1.length<l2.length)
	{
		return true;
	}
	return false;
}
inline bool myCompareGreater(const LENGTHINDEX&l1,const LENGTHINDEX &l2)
{
	if (l1.length>l2.length)
	{
		return true;
	}
	return false;
}


POINTCLOUD_PROCESSING_API int DownSamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &srcPoints,pcl::PointCloud<pcl::PointXYZ>::Ptr &downSamplePoints,
													 float gridX,float gridY,float gridZ);

POINTCLOUD_PROCESSING_API int ApplyRigidTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr &srcPoints,pcl::PointCloud<pcl::PointXYZ>::Ptr &outPoints,
								pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4  &transformation,
								bool isUsingICP, pcl::PointCloud<pcl::PointXYZ>::Ptr &destPoints,
								pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4  *pOuttransformation,
								float gridX,float gridY,float gridZ);

POINTCLOUD_PROCESSING_API int ParseVGFile(const std::string &fileName,pcl::PointCloud<pcl::PointXYZ>::Ptr points,
								std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> >&plane,
								std::vector<std::vector<int>>&supportPlanePointsIndex);

POINTCLOUD_PROCESSING_API int ComputeIntersectionPointOfThreePlanes(const Eigen::Vector4f &p1,
																	const Eigen::Vector4f &p2,
																	const Eigen::Vector4f &p3,
																	pcl::PointXYZ &point,
																	float *pCondNum=NULL);

POINTCLOUD_PROCESSING_API int ComputeIntersectionLineOfTwoPlanes(const Eigen::Vector4f &plane1,
																 const Eigen::Vector4f &plane2,
																 Eigen::Vector3f &line);

POINTCLOUD_PROCESSING_API int ComputeIntersectionLineOfTwoPlanes(const Eigen::Vector4f &plane1,
																 const Eigen::Vector4f &plane2,
																 Eigen::Vector3f &lineVec,
																 Eigen::Vector3f &linePoint);

POINTCLOUD_PROCESSING_API int ComputeBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr points,
												 Eigen::Vector3f &centerPoint,double &width,double &height,double &depth,
												 pcl::PointCloud<pcl::PointXYZ>*pCornerPoints=NULL,
												 Eigen::Matrix3f *pR=NULL, Eigen::Vector3f *pT=NULL);

POINTCLOUD_PROCESSING_API int RansacEstimateRigidTransformation(pcl::PointCloud<pcl::PointXYZ>& source,pcl::PointCloud<pcl::PointXYZ>& target,
									  Eigen::Matrix4f &transformation, int maxIter,double lengthThreshold,
									   std::vector<int>&innerPointsIndex);

POINTCLOUD_PROCESSING_API int ComputeNearstTwoPointsOfTwo3DLine(Eigen::Vector3f &line1Vec,Eigen::Vector3f &line1Point,
									  Eigen::Vector3f &line2Vec,Eigen::Vector3f &line2Point,
									  Eigen::Vector3f &point1,  Eigen::Vector3f &point2,
									  double &minLength);

POINTCLOUD_PROCESSING_API int ComputeTransformationUsingTwoVecAndOnePoint(Eigen::Vector3f &sourceVec1,Eigen::Vector3f &sourceVec2,
												Eigen::Vector3f &destVec1,Eigen::Vector3f &destVec2,
												Eigen::Vector3f &sourcePoint,Eigen::Vector3f &targetPoint,
												Eigen::Matrix3f &R,Eigen::Vector3f &T);

POINTCLOUD_PROCESSING_API bool isTwoPlanesConsistent(Eigen::Vector4f &srcPlane,Eigen::Vector4f &tarPlane,Eigen::Matrix3f &R,Eigen::Vector3f &T,
													double cosAngleThreshold, double lengthThreshold);

POINTCLOUD_PROCESSING_API bool isTwoPlanesConsistent1(Eigen::Vector4f &srcPlane,Eigen::Vector4f &tarPlane,
													  Eigen::Vector3f &srcPlaneBoundingBoxCenter,
													  Eigen::Vector3f &destPlaneBoundingBoxCenter,
													  float &srcPlaneBoundingBoxRadius,
													  float &destPlaneBoundingBoxRadius,
													  Eigen::Matrix3f &R,Eigen::Vector3f &T,
													  double cosAngleThreshold, double lengthThreshold);

POINTCLOUD_PROCESSING_API bool isTwoIntersectionLionsConsistent(INTERSECTION_LINE &srcIntersectionLion, INTERSECTION_LINE &destIntersectionLion,
									  std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > &srcPlanes,
									  std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > &destPlanes,
									  Eigen::Matrix3f &R,Eigen::Vector3f &T,
									  double angleThreshold, double lengthThreshold);


POINTCLOUD_PROCESSING_API bool isTwoIntersectionLionsConsistent1(INTERSECTION_LINE &srcIntersectionLion, INTERSECTION_LINE &destIntersectionLion,
																 std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > &srcPlanes,
																 std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > &destPlanes,
																 std::vector<Eigen::Vector3f> &srcPlaneBoundingBoxCenter,
																 std::vector<Eigen::Vector3f> &destPlaneBoundingBoxCenter,
																 std::vector<float> &srcPlaneBoundingBoxRadius,
																 std::vector<float> &destPlaneBoundingBoxRadius,
																 Eigen::Matrix3f &R,Eigen::Vector3f &T,
																 double angleThreshold, double lengthThreshold);

POINTCLOUD_PROCESSING_API int ComputeMeanDistanceOfLine2Plane(INTERSECTION_LINE &line,const pcl::PointCloud<pcl::PointXYZ>&cornerPoints,
									pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree,double &meanDistance,float interval);

POINTCLOUD_PROCESSING_API int ComputeProjectionPointOf3DLine(const Eigen::Vector3f &lineVec, const Eigen::Vector3f &linePoint,
								   const pcl::PointXYZ &pointIn, pcl::PointXYZ &pointOut);

POINTCLOUD_PROCESSING_API int ComputeProjectionPointOf3DLine(const Eigen::Vector3f &lineVec, const Eigen::Vector3f &linePoint,
								   const pcl::PointXYZ&pointIn, Eigen::Vector3f &pointOut);

POINTCLOUD_PROCESSING_API int ResetNormalDirection(pcl::PointXYZ &viewPoint, pcl::PointCloud<pcl::PointXYZ>& points,
								 pcl::PointCloud<pcl::Normal>& normals);

POINTCLOUD_PROCESSING_API int AreTwoPlanesPenetrable(Eigen::Vector4f &plane1,Eigen::Vector4f &plane2, 
													 std::vector<Eigen::Vector3f> &cornerPoints1,
													 std::vector<Eigen::Vector3f> &cornerPoints2,
													 pcl::search::KdTree<pcl::PointXYZ>::Ptr KdTree1,
													 pcl::search::KdTree<pcl::PointXYZ>::Ptr KdTree2,
													 bool &isPentrable,float searchRadius=0.1,int minPointsNum=10,
													 float minDistance=0.03);

POINTCLOUD_PROCESSING_API int ProjectPoints2Plane(const pcl::PointCloud<pcl::PointXYZ>&pointsIn,const Eigen::Vector4f &plane, 
												  pcl::PointCloud<pcl::PointXYZ>&pointsOut,
												  int *pBegin=NULL,int *pEnd=NULL);

POINTCLOUD_PROCESSING_API int ProjectPoints2Plane(const pcl::PointCloud<pcl::PointXYZ>&pointsIn,const Eigen::Vector4f &plane,
											std::vector<Eigen::Vector3f>&pointsOut,int *pBegin=NULL,int *pEnd=NULL);


POINTCLOUD_PROCESSING_API int ProjectPoints2PlaneCoordinate( pcl::PointCloud<pcl::PointXYZ>& pointsIn,
															  std::vector<cv::Point2f> &pointsOut,
															Eigen::Vector4f &plane);

POINTCLOUD_PROCESSING_API int ExchnageBetweentPCLPointXYZwithEigenVector3f(pcl::PointCloud<pcl::PointXYZ> &pclPoint,
												 std::vector<Eigen::Vector3f>& eigenPoints);

POINTCLOUD_PROCESSING_API int DetectBoundaryPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr points, std::vector<int> &boundariePointsIndex,
											 float normalRadius=0.03, float boundarySearchRadius=0.05,bool showResult=false );

POINTCLOUD_PROCESSING_API int FilterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr points, std::vector<int> &boundariePointsIndex,pcl::PointXYZ &viewPoint,
											 std::vector<int>& outPoints,bool isRemoveOcclusionPoints );


POINTCLOUD_PROCESSING_API int load_bpn(const std::string& file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud,
											 pcl::PointCloud<pcl::Normal>*pointNormal=NULL); 

POINTCLOUD_PROCESSING_API void save_bpn(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZ>& pointCloud,pcl::PointCloud<pcl::Normal>*pPointNormal=NULL);

POINTCLOUD_PROCESSING_API int load_xyz(const std::string &fileName,pcl::PointCloud<pcl::PointXYZ>::Ptr pointCLoud);

POINTCLOUD_PROCESSING_API int load_plane(const std::string &fileName,std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> >&planes);

POINTCLOUD_PROCESSING_API int load_support_points_index(const std::string &fileName,std::vector<std::vector<int>>&supportPointsIndex);

POINTCLOUD_PROCESSING_API int Fit3DLine(std::vector<cv::Point3f>&points, cv::Vec6f &param,float *pAccuracy=NULL);

POINTCLOUD_PROCESSING_API int RansacExtract3Dline(pcl::PointCloud<pcl::PointXYZ>&points,std::vector<int>&boudaryPointsIndex, 
												  std::vector<std::vector<int>>& extractLines,
												  int maxLinesNum, double inlierThreshold=0.02,int minLinePointsNum=30,
												  int eachLineTestNum=100,bool showResult=false);

POINTCLOUD_PROCESSING_API bool CheckWhetherPointCLoudsInTheInlierOfRoomFrame(
										std::vector<std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>>>&roomFrame,
										pcl::PointCloud<pcl::PointXYZ>&testPoints,double distanceThreshold,int maxOutlierNum);

POINTCLOUD_PROCESSING_API bool CheckWhetherPointInTheInlierOfRoomFrame(
									std::vector<std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>>>&roomFrame,
									pcl::PointXYZ &testPoints,double distanceThreshold);

POINTCLOUD_PROCESSING_API bool CheckWhetherPointInTheInlierOfRoomFrame(
									std::vector<std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>>>&roomFrame,
									Eigen::Vector3f &testPoints,double distanceThreshold);

POINTCLOUD_PROCESSING_API int ComputeOverlap(pcl::search::KdTree<pcl::PointXYZ>::Ptr queryTree,
									   pcl::search::KdTree<pcl::PointXYZ>::Ptr destTree,
									   Eigen::Vector3f &queryCenter,
									   float queryRadius,
									   float inlierDistance, float &overLapRation,bool showResult=false);

POINTCLOUD_PROCESSING_API bool CheckWheterTwoPointCloudsHaveOverLap(pcl::search::KdTree<pcl::PointXYZ>::Ptr queryTree,
										  pcl::search::KdTree<pcl::PointXYZ>::Ptr destTree,
										  float inlierDistance);

POINTCLOUD_PROCESSING_API bool CheckWheterTwoPointCloudsHaveOverLap(pcl::search::KdTree<pcl::PointXYZ>::Ptr queryTree,
										  pcl::search::KdTree<pcl::PointXYZ>::Ptr destTree,
										  Eigen::Vector3f &queryCenter,
										  Eigen::Vector3f &destCenter,
										  float queryRadius,
										  float destRadius,
										  float inlierDistance);

POINTCLOUD_PROCESSING_API int ComputeDescriptorVectorForPairLines(PCP::INTERSECTION_LINE &line1,
																  PCP::INTERSECTION_LINE &line2,
																  Eigen::Vector3f &line1sp1,
																  Eigen::Vector3f &line1sp2,
																  Eigen::Vector3f &line2sp1,
																  Eigen::Vector3f &line2sp2,
																  Eigen::VectorXf &descriptor,
																  Eigen::Vector3f &newLine1,
																  Eigen::Vector3f &newLine2,
																  METHODINDEX methodIndex);

POINTCLOUD_PROCESSING_API int ConstructPairLinesKdTree(std::vector<PCP::INTERSECTION_LINE> &lines,
													   std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>> &planes,
													   std::vector<KdTreeSearchNDim<Eigen::VectorXf,8>> &kdtrees8,
													   std::vector<KdTreeSearchNDim<Eigen::VectorXf,6>> &kdtrees6,
													   std::vector<KdTreeSearchNDim<Eigen::VectorXf,4>> &kdtrees4,
													   std::vector<std::vector<PCP::PAIRLINE>> &linesInformation,
													   float scalar);

POINTCLOUD_PROCESSING_API int ClusterTransformation(std::vector<Eigen::Matrix3f>&Rs,
													std::vector<Eigen::Vector3f>&Ts,
													float distanceThreshold,
													float angleThreshold,
													pcl::IndicesClusters &clusters);

POINTCLOUD_PROCESSING_API int DetectFloorsFromLaserScan(std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>> &planes,
								std::vector<int> &floorIndex);

POINTCLOUD_PROCESSING_API int RansacExtractPlanes(pcl::PointCloud<pcl::PointXYZ>& points,
												  std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>>&planes,
												  std::vector<std::vector<int>>&supportPlanePointsIndex,
												  float lengthThresh,
												  float normalThresh,
												  int minPointsNum);

POINTCLOUD_PROCESSING_API int ComputeTotalAreaOfTwoCoplanarPlanes(const pcl::PointCloud<pcl::PointXYZ>& points1,
										const pcl::PointCloud<pcl::PointXYZ>& points2,
										const Eigen::Vector4f &mappedPlane,
										float& totalArea,
										float *pGridSize=NULL
										);

POINTCLOUD_PROCESSING_API int ComputeInformationOfTwoCoplanarPlanes(const pcl::PointCloud<pcl::PointXYZ>& points1,
										  const pcl::PointCloud<pcl::PointXYZ>& points2,
										  const Eigen::Vector4f &mappedPlane,
										  float* pIntersectArea=NULL,
										  float* pTotalArea=NULL,
										  float* pPoints1Area=NULL,
										  float* pPoints2Area=NULL,
										  float *pGridSizeX=NULL,
										  float *pGridSizeY=NULL
										  );

POINTCLOUD_PROCESSING_API int CombinePlanes(std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>> &planes,
										std::vector<std::vector<int>>&combinedPlanes,
										float distanceTh,float angleTh);


template<typename _T1, typename _T2>
inline double computeLengthOfTwo3DPoint(_T1 &point1,_T2 &point2)
{
	double x=point1.x-point2.x;
	double y=point1.y-point2.y;
	double z=point1.z-point2.z;
	return sqrt(x*x+y*y+z*z);
}

#pragma warning( disable: 4244 )//去除一些类型转换的警告
//计算两条直线的交点
//类型必须是vec3...
template<typename _T1, typename _T2>
int ComputeIntersectionPoint(_T1 &lineCoefficient1, _T1 &lineCoefficient2,_T2 &intersectionPoint)
{
	double  i1Scalar=lineCoefficient1[0]*lineCoefficient2[1]-lineCoefficient2[0]*lineCoefficient1[1];
	if (i1Scalar)//若为0 则交于无限远点
	{
		intersectionPoint.x=((lineCoefficient1[1]*lineCoefficient2[2]-lineCoefficient2[1]*lineCoefficient1[2])/double(i1Scalar));
		intersectionPoint.y=((lineCoefficient2[0]*lineCoefficient1[2]-lineCoefficient1[0]*lineCoefficient2[2])/double(i1Scalar));
	}
	else
	{
		return -1;
	}
	return 0;
}

//计算两点之间的距离
//_T1与_T2必须是cv::Point中的类型
template<typename _T1, typename _T2>
inline double computeLengthOfTwoPoint(_T1 &point1,_T2 &point2)
{
	cv::Point2d tempPoint;
	tempPoint.x=point1.x-point2.x;
	tempPoint.y=point1.y-point2.y;
	return sqrt(tempPoint.x*tempPoint.x+tempPoint.y*tempPoint.y);
}


template<typename _T1, typename _T2>
inline double computeLengthOfTwo3DVec3f(_T1 &point1,_T2 &point2)
{
	double x=point1[0]-point2[0];
	double y=point1[1]-point2[1];
	double z=point1[2]-point2[2];
	return sqrt(x*x+y*y+z*z);
}

template<typename _T1, typename _T2, typename _T3 >
inline void computeCenterPointOfTwoPoint(_T1 &point1,_T2 &point2, _T3 &centerPoint)
{
	centerPoint.x=double(point1.x+point2.x)/2;
	centerPoint.y=double(point1.y+point2.y)/2;
}

template<typename _Tout, typename _T1, typename _T2>
inline _Tout computeLineThroughTwoPoints(_T1 &point1,_T2 &point2)
{
	_Tout Line;//中点连线
	Line[0]=point1.y-point2.y;
	Line[1]=point2.x-point1.x;
	Line[2]=point1.x*point2.y-point2.x*point1.y;
	return Line;
}
template<typename _Tout, typename _T1, typename _T2>
inline _Tout computeLineThroughTwo3DPoints(_T1 &point1,_T2 &point2)
{
	_Tout Line;//中点连线
	Line[0]=point1.x-point2.x;
	Line[1]=point1.y-point2.y;
	Line[2]=point1.z-point2.z;
	Line[3]=point1.x;
	Line[4]=point1.y;
	Line[5]=point1.z;
	return Line;
}

template<typename _T1, typename _T2>
inline double computeLengthOfPoint2Line(_T1 &line, _T2 &point)
{
	if (line[0]==0&&line[1]==0)
	{
		return -1;//直线参数错误
	}
	return abs(line[0]*point.x+line[1]*point.y+line[2])/sqrt(line[0]*line[0]+line[1]*line[1]);
}

template<typename _T1, typename _T2>
inline double computeLengthOfPoint23DLine(_T1 &line, _T2 &point)
{
	if (line[0]==0&&line[1]==0&&line[2]==0)
	{
		return -1;//直线参数错误
	}
	double m=line[0];
	double n=line[1];
	double p=line[2];
	double x0=line[3];
	double y0=line[4];
	double z0=line[5];
	double x1=point.x;
	double y1=point.y;
	double z1=point.z;
	double t=(m*(x0-x1)+n*(y0-y1)+p*(z0-z1))/(m*m+n*n+p*p);
	double xc=m*t+x1;
	double yc=n*t+y1;
	double zc=p*t+z1;
	return sqrt((x0-xc)*(x0-xc)+(y0-yc)*(y0-yc)+(z0-zc)*(z0-zc));
}

inline double computeLengthOfPoint23DLine(Eigen::Vector3f &lineVec,pcl::PointXYZ& linePoint, pcl::PointXYZ &testPoint)
{
	if (lineVec[0]==0&&lineVec[1]==0&&lineVec[2]==0)
	{
		return -1;//直线参数错误
	}
	double m=lineVec[0];
	double n=lineVec[1];
	double p=lineVec[2];
	double x0=linePoint.x;
	double y0=linePoint.y;
	double z0=linePoint.z;
	double x1=testPoint.x;
	double y1=testPoint.y;
	double z1=testPoint.z;
	double t=(m*(x0-x1)+n*(y0-y1)+p*(z0-z1))/(m*m+n*n+p*p);
	double xc=m*t+x1;
	double yc=n*t+y1;
	double zc=p*t+z1;
	return sqrt((x0-xc)*(x0-xc)+(y0-yc)*(y0-yc)+(z0-zc)*(z0-zc));
}

//lineVec 需要归一化
inline double computeLengthOfPoint23DLine1(Eigen::Vector3f &lineVec,pcl::PointXYZ& linePoint, pcl::PointXYZ &testPoint)
{
	Eigen::Vector3f test(testPoint.x,testPoint.y,testPoint.z);
	Eigen::Vector3f lp(linePoint.x,linePoint.y,linePoint.z);
	Eigen::Vector3f vec=test-lp;
	double veclength=vec.norm();
	double projectLength=vec.dot(lineVec);
	return sqrt(veclength*veclength-projectLength*projectLength);
}

//求两条直线之间所夹的那个锐角的大小，范围为0到pi/2；
template<typename _T1, typename _T2>
int ComputeAngleOfTwoLine(_T1 &line1,_T2 &line2,double &angle)
{
	//求得方向向量
	double vx1,vy1,vx2,vy2;
	if (line1[1])
	{
		vx1=1;
		vy1=-line1[0]/line1[1];
	}
	else
	{
		vx1=0;
		vy1=1;
	}
	if (line2[1])
	{
		vx2=1;
		vy2=-line2[0]/line2[1];
	}
	else
	{
		vx2=0;
		vy2=1;
	}
	if (cvIsNaN(vx1*vy1*vx2*vy2))
	{
		return -1;
	}
	angle=acos((vx1*vx2+vy1*vy2)/(sqrt(vx1*vx1+vy1*vy1)*sqrt(vx2*vx2+vy2*vy2)));
	if (angle >M_PI_2)
	{
		angle=M_PI-angle;
	}
	return 0;
}

//求由point1指向point2的向量,其中
template<typename _T1, typename _T2, typename _T3>
int ComputeVectorThroughTwoPoints(_T1 &point1,_T2 &point2, _T3 &vec)
{
	vec[0]=point2.x-point1.x;
	vec[1]=point2.y-point1.y;
	return 0;
}

//求由point1指向point2的向量,其中
template<typename _T1, typename _T2, typename _T3>
int ComputeVectorThroughTwo3DPoints(_T1 &point1,_T2 &point2, _T3 &vec)
{
	vec[0]=point2.x-point1.x;
	vec[1]=point2.y-point1.y;
	vec[2]=point2.z-point1.z;
	return 0;
}

//计算两向量之间的夹角
template<typename _T1, typename _T2>
int ComputeAngleOfTwoVector(_T1 &vec1,_T2 &vec2,double &angle)
{
	double temp=sqrt(vec1[0]*vec1[0]+vec1[1]*vec1[1])*sqrt(vec2[0]*vec2[0]+vec2[1]*vec2[1]);
	if (!temp)//temp==0
	{
		return -1;
	}
	angle=acos((vec1[0]*vec2[0]+vec1[1]*vec2[1])/temp);
	return 0;
}

//计算两向量之间最小的夹角
template<typename _T1, typename _T2>
int ComputeMinAngleOfTwoVector(_T1 &vec1,_T2 &vec2,double &angle)
{
	double temp=sqrt(vec1[0]*vec1[0]+vec1[1]*vec1[1])*sqrt(vec2[0]*vec2[0]+vec2[1]*vec2[1]);
	if (!temp)//temp==0
	{
		return -1;
	}
	angle=acos((vec1[0]*vec2[0]+vec1[1]*vec2[1])/temp);
	if (angle>M_PI_2)
	{
		angle=M_PI-angle;
	}
	return 0;
}

//计算两向量之间的夹角
template<typename _T1, typename _T2>
int ComputeAngleOfTwo3DVector(_T1 &vec1,_T2 &vec2,double &angle)
{
	double temp=sqrt(vec1[0]*vec1[0]+vec1[1]*vec1[1]+vec1[2]*vec1[2])*sqrt(vec2[0]*vec2[0]+vec2[1]*vec2[1]+vec2[2]*vec2[2]);
	if (!temp)//temp==0
	{
		return -1;
	}
	angle=acos((vec1[0]*vec2[0]+vec1[1]*vec2[1]+vec1[2]*vec2[2])/temp);
	return 0;
}

//计算两向量之间的夹角
template<typename _T1, typename _T2>
int ComputeMinAngleOfTwo3DVector(_T1 &vec1,_T2 &vec2,double &angle)
{
	double temp=sqrt(vec1[0]*vec1[0]+vec1[1]*vec1[1]+vec1[2]*vec1[2])*sqrt(vec2[0]*vec2[0]+vec2[1]*vec2[1]+vec2[2]*vec2[2]);
	if (!temp)//temp==0
	{
		return -1;
	}
	angle=acos((vec1[0]*vec2[0]+vec1[1]*vec2[1]+vec1[2]*vec2[2])/temp);
	if (angle>M_PI_2)
	{
		angle=M_PI-angle;
	}
	return 0;
}

}
#endif // !POINTCLOUD_PROCESSING