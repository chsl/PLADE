#include "stdafx.h"
//#define COMPILE
#ifdef COMPILE
#include "common.h"
//#define DISPLAY1
//#define DISPLAY2

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

int load_bpn(const std::string& file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud,pcl::PointCloud<pcl::Normal>*pointNormal=NULL) 
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

void save_bpn(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZ>& pointCloud)
{
	// open file
	std::ofstream output(file_name.c_str(), std::fstream::binary);
	if (output.fail()) {
		std::cout << "could not open file\'" << file_name << "\'" << std::endl;
		return;
	}

	// TODO: save large blocks
	//ProgressLogger progress(pset->num_point());
	float normal[3]={0,0,0};
	for (std::size_t i = 0; i < pointCloud.size(); ++i) 
	{
		output.write((char*)&pointCloud[i], 12);
		output.write((char*)normal,12);
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
			fin>>pointsNum;
			std::vector<int>currentPointsIndex(pointsNum);
			for (int i=0;i<pointsNum;i++)
			{
				fin>>currentPointsIndex[i];
			}
			supportPointsIndex.push_back(currentPointsIndex);
			fin.ignore(numeric_limits<streamsize>::max(),'\n');
		}
	}
	return 0;
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


int main (int argc, char **argv)
{
	std::string fileDire;
	cout<<"input the file director:";
	cin>>fileDire;
	while (true)
	{
		cout<<"input part num:";
		int partNum;
		cin>>partNum;
		pcl::PointCloud<pcl::PointXYZ>::Ptr  points(new pcl::PointCloud<pcl::PointXYZ>);
		std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f>> plane; 
		std::vector<std::vector<int>> supportPlaneIndex;
		PCP::ParseVGFile(fileDire+cv::format("part%d.vg",partNum),points,plane,supportPlaneIndex);
		cout<<"points:"<<points->size()<<endl;
		size_t mainPlanesNum=plane.size();
		std::vector<pcl::ModelCoefficients> modelMainPlaneCoff(mainPlanesNum);
		for (size_t i=0;i<mainPlanesNum;i++)
		{
			for (int j=0;j<4;j++)
			{
				modelMainPlaneCoff[i].values.push_back(plane[i][j]);
			}
		}
		cout<<"planesNum:"<<plane.size()<<endl;

		for (size_t i=0;i<plane.size();i+=4)
		{
			cout<<i<<endl;
			boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("3DSC test"));
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(points,255,255,255);
			view->addPointCloud(points,sources_cloud_color,"sources_cloud_v1");
			for (size_t k=i;k<MIN(i+4,plane.size());k++)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr  planePoint(new pcl::PointCloud<pcl::PointXYZ>);
				planePoint->reserve(supportPlaneIndex[k].size());
				for (int j = 0; j < supportPlaneIndex[k].size(); j++)
				{
					planePoint->push_back(points->at(supportPlaneIndex[k][j]));
				}
				int r=rand()%255;
				int g=rand()%255;
				int b=rand()%255;
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(planePoint,r,g,b);
				view->addPointCloud(planePoint,target_cloud_color,cv::format("planePoints%d",k));
				view->addPlane(modelMainPlaneCoff[k],(*planePoint)[0].x,(*planePoint)[0].y,(*planePoint)[0].z,cv::format("plane%d",k));
				Eigen::Vector3f start((*planePoint)[0].x,(*planePoint)[0].y,(*planePoint)[0].z);
				Eigen::Vector3f end=start+plane[k].block(0,0,3,1)*1;
				view->addArrow(pcl::PointXYZ(start(0),start(1),start(2)),pcl::PointXYZ(end(0),end(1),end(2)),r/255.0,g/255.0,b/255.0,cv::format("line%d",k));
				view->addText3D(cv::format("%d",k),pcl::PointXYZ(start(0),start(1),start(2)),0.2);

			}
			
			while (!view->wasStopped())
			{
				view->spinOnce(100);
			}

			char answer;
			cout<<"flip? y or n: ";
			cin>>answer;
			if (answer=='y')
			{
				while (true)
				{
					int i;
					cout<<"input which one to flip:";
					cin>>i;
					if (i<0)
					{
						break;
					}
					plane[i]=-plane[i];
					for (int j=0;j<4;j++)
					{
						modelMainPlaneCoff[i].values[j]=plane[i][j];
					}
					cout<<"flip successfully"<<endl;
					cout<<"please check again..."<<endl;
					boost::shared_ptr<pcl::visualization::PCLVisualizer> view (new pcl::visualization::PCLVisualizer("3DSC test"));
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(points,255,255,255);
					view->addPointCloud(points,sources_cloud_color,"sources_cloud_v1");
					pcl::PointCloud<pcl::PointXYZ>::Ptr  planePoint(new pcl::PointCloud<pcl::PointXYZ>);
					planePoint->reserve(supportPlaneIndex[i].size());
					for (int j = 0; j < supportPlaneIndex[i].size(); j++)
					{
						planePoint->push_back(points->at(supportPlaneIndex[i][j]));
					}
					int r=rand()%255;
					int g=rand()%255;
					int b=rand()%255;
					pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(planePoint,r,g,b);
					view->addPointCloud(planePoint,target_cloud_color,cv::format("planePoints%d",i));
					view->addPlane(modelMainPlaneCoff[i],(*planePoint)[0].x,(*planePoint)[0].y,(*planePoint)[0].z,cv::format("plane%d",i));
					Eigen::Vector3f start((*planePoint)[0].x,(*planePoint)[0].y,(*planePoint)[0].z);
					Eigen::Vector3f end=start+plane[i].block(0,0,3,1)*1;
					view->addArrow(pcl::PointXYZ(start(0),start(1),start(2)),pcl::PointXYZ(end(0),end(1),end(2)),r/255.0,g/255.0,b/255.0,cv::format("line%d",i));
					view->addText3D(cv::format("%d",i),pcl::PointXYZ(start(0),start(1),start(2)),0.2);
					while (!view->wasStopped())
					{
						view->spinOnce(100);
					}
				}
			}
		}
		
		ofstream fout;
		fout.open(fileDire+cv::format("partPlane%d.txt",partNum));
		for (size_t i=0;i<plane.size();i++)
		{
			fout<<plane[i](0)<<" "<<plane[i](1)<<" "<<plane[i](2)<<" "<<plane[i](3)<<endl;
		}
		fout.close();
		fout.open(fileDire+cv::format("planeIndex%d.txt",partNum));
		for (size_t i=0;i<supportPlaneIndex.size();i++)
		{
			fout<<supportPlaneIndex[i].size()<<" ";
			for (size_t j=0;j<supportPlaneIndex[i].size();j++)
			{
				fout<<supportPlaneIndex[i][j]<<" ";
			}
			fout<<endl;
		}
		fout.close();
		cout<<"files have been saved successfully"<<endl;
	}
	
	return 0;
}
#endif