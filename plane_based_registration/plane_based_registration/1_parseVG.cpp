#include "stdafx.h"
#define COMPILE
#ifdef COMPILE
#include "common.h"

int main (int argc, char **argv)
{
	if (argc != 2){
		cout<<"usage: **.exe point_cloud.bpn"<<endl;
		return -1;
	}
	//Parameters
	std::string targetFile;
	targetFile = argv[1];
	cout<<"point_cloud: "<<targetFile<<endl;
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr  part21(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr  part211(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr  normal(new pcl::PointCloud<pcl::Normal>);
	std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>> part21Plane; 
	std::vector<std::vector<int>> part21Index;
	PCP::ParseVGFile(drive+targetFilePath+targetFileName+".vg",part21,part21Plane,part21Index);
	PCP::load_bpn(targetFile,part211,&(*normal));

	size_t mainPlanesNum=part21Plane.size();
	std::vector<pcl::ModelCoefficients> modelMainPlaneCoff(mainPlanesNum);
	for (size_t i=0;i<mainPlanesNum;i++)
	{
		for (int j=0;j<4;j++)
		{
			modelMainPlaneCoff[i].values.push_back(part21Plane[i][j]);
		}
	}
	cout<<"planesNum:"<<part21Plane.size()<<endl;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("points"));
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(part21,255,255,255);
	view->addPointCloud(part21,sources_cloud_color,"sources_cloud_v1");
	for (size_t k=0;k<part21Plane.size();k++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr  planePoint(new pcl::PointCloud<pcl::PointXYZ>);
		planePoint->reserve(part21Index[k].size());
		for (int j = 0; j < part21Index[k].size(); j++)
		{
			planePoint->push_back(part21->at(part21Index[k][j]));
		}
		int r=rand()%255;
		int g=rand()%255;
		int b=rand()%255;
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(planePoint,r,g,b);
		view->addPointCloud(planePoint,target_cloud_color,cv::format("planePoints%d",k));
		view->addPlane(modelMainPlaneCoff[k],(*planePoint)[0].x,(*planePoint)[0].y,(*planePoint)[0].z,cv::format("plane%d",k));
		Eigen::Vector3f start((*planePoint)[0].x,(*planePoint)[0].y,(*planePoint)[0].z);
		Eigen::Vector3f end=start+part21Plane[k].block(0,0,3,1)*1;
		view->addArrow(pcl::PointXYZ(start(0),start(1),start(2)),pcl::PointXYZ(end(0),end(1),end(2)),r/255.0,g/255.0,b/255.0,cv::format("line%d",k));
		view->addText3D(cv::format("%d",k),pcl::PointXYZ(start(0),start(1),start(2)),0.2);

	}

	while (!view->wasStopped())
	{
		view->spinOnce(100);
	}

	cout<<"need flip plane norm? y or n:";
	char answer = 'n';
	cin>>answer;
	if (answer == 'y'){
		while (true)
		{
			int i;
			cout<<"input which one to flip(-1 to exit):";
			cin>>i;
			if (i<0)
			{
				break;
			}
			part21Plane[i]=-part21Plane[i];
			for (int j=0;j<4;j++)
			{
				modelMainPlaneCoff[i].values[j]=part21Plane[i][j];
			}
			cout<<"flip successfully"<<endl;
			cout<<"please check again..."<<endl;
			boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("3DSC test"));
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(part21,255,255,255);
			view->addPointCloud(part21,sources_cloud_color,"sources_cloud_v1");
			pcl::PointCloud<pcl::PointXYZ>::Ptr  planePoint(new pcl::PointCloud<pcl::PointXYZ>);
			planePoint->reserve(part21Index[i].size());
			for (int j = 0; j < part21Index[i].size(); j++)
			{
				planePoint->push_back(part21->at(part21Index[i][j]));
			}
			int r=rand()%255;
			int g=rand()%255;
			int b=rand()%255;
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(planePoint,r,g,b);
			view->addPointCloud(planePoint,target_cloud_color,cv::format("planePoints%d",i));
			view->addPlane(modelMainPlaneCoff[i],(*planePoint)[0].x,(*planePoint)[0].y,(*planePoint)[0].z,cv::format("plane%d",i));
			Eigen::Vector3f start((*planePoint)[0].x,(*planePoint)[0].y,(*planePoint)[0].z);
			Eigen::Vector3f end=start+part21Plane[i].block(0,0,3,1)*1;
			view->addArrow(pcl::PointXYZ(start(0),start(1),start(2)),pcl::PointXYZ(end(0),end(1),end(2)),r/255.0,g/255.0,b/255.0,cv::format("line%d",i));
			view->addText3D(cv::format("%d",i),pcl::PointXYZ(start(0),start(1),start(2)),0.2);
			while (!view->wasStopped())
			{
				view->spinOnce(100);
			}
		}
	}
	else
	{
		ofstream fout;
		fout.open(drive+targetFilePath+"\\"+targetFileName+"_plane.txt");
		for (size_t i=0;i<part21Plane.size();i++)
		{
			fout<<part21Plane[i](0)<<" "<<part21Plane[i](1)<<" "<<part21Plane[i](2)<<" "<<part21Plane[i](3)<<endl;
		}
		fout.close();
		fout.open(drive+targetFilePath+"\\"+targetFileName+"_planeIndex.txt");
		for (size_t i=0;i<part21Index.size();i++)
		{
			fout<<part21Index[i].size()<<" ";
			for (size_t j=0;j<part21Index[i].size();j++)
			{
				fout<<part21Index[i][j]<<" ";
			}
			fout<<endl;
		}
		fout.close();
	}
	
	return 0;
}
#endif