# PLADE
## PLADE: A Plane-based Descriptor for Point Cloud Registration with Small Overlap

## Instructions for use:  
1. Download the code, open it in visual studio, the entry file is "PLADE.sln"
1. Dependencies:  
-     1、[Opencv](https://opencv.org/)
-     2、[boost](https://www.boost.org/)
-     3、[PCL](https://pointclouds.org/)  
    Please modify "opencv2410.props", "pcl1.8.0_debug.props" and "pcl1.8.0_release.props" after updating the third-party library
3. The project consists of two parts:
-     1、pointcloud_processing: mainly includeing some point cloud processing algorithms
-     2、plane_based_registration: code related to point cloud registration
4. Specific instructions for use:
    1. Download the point cloud processing software [Mapple] (https://3d.bk.tudelft.nl/liangliang/software.html) published by Professor Nan Liangliang
    2. Use this software to preprocess point cloud data, including:  
        (1) Downsampling, as shown in the figure below. Generally, for indoor point clouds, the point spacing is recommended to be set to 0.005-0.01mm, and for outdoor point cloud data, the point spacing is recommended to be set to 0.01-0.05mm.
![image](https://github.com/chsl/PLADE/blob/master/plane_based_registration/image/downsample1.png)
![image](https://github.com/chsl/PLADE/blob/master/plane_based_registration/image/downsample2.png)  
        (2) If the input point data does not include normal information, you need to manually calculate the normal, and proceed as follows
![image](https://github.com/chsl/PLADE/blob/master/plane_based_registration/image/normals.png)  
        (3) Perform plane extraction operations
![image](https://github.com/chsl/PLADE/blob/master/plane_based_registration/image/plane1.png)
![image](https://github.com/chsl/PLADE/blob/master/plane_based_registration/image/plane2.png)
The default UI will not display the extracted planes, so you need to switch the display mode as shown in the figure below. The points of the same color indicate that they belong to the same plane.
![image](https://github.com/chsl/PLADE/blob/master/plane_based_registration/image/plane3.png)  
        (4) Save the extracted planes in vg format, and save a copy of data in bpn format with the same file name at the same time  
        (5) Compile the parse_VG file in the plane_based_registration project, and process the data saved in the previous step and view the normal direction of the plane. The algorithm implementation requires that the two corresponding planes in the two point cloud data to be registered have the same normal direction. If they are not the same, you need to follow the prompts to flip them. After the operation is completed, the corresponding file will be generated for the next operation  
        (6) Compile the registration file in the plane_based_registration project, and follow the prompts to perform the registration operation  
# PLADE  
##  PLADE: A Plane-based Descriptor for Point Cloud Registration with Small Overlap  

## 使用说明：  
1. 下载整个代码仓，在visual studio中打开，入口文件为 "PLADE.sln"  
1. 依赖的第三方库：  
-     1、[Opencv](https://opencv.org/)  
-     2、[boost](https://www.boost.org/)  
-     3、[PCL](https://pointclouds.org/)  
    更新第三方库后请修改 "opencv2410.props"、"pcl1.8.0_debug.props" 和 "pcl1.8.0_release.props"  
3. 工程包括两部分：  
-     1、pointcloud_processing：主要包括一些点云处理的算法  
-     2、plane_based_registration：点云注册相关的代码  
4. 具体使用说明：  
    1、下载南亮亮教授公开的点云处理软件[Mapple ](https://3d.bk.tudelft.nl/liangliang/software.html)  
    2、使用该软件对点云数据进行预处理，包括：  
        (1) 下采样，按照如下图所示操作, 一般情况下，对于室内点云，点间距建议下采样到0.005-0.01mm，对于室外点云数据，点间距建议下采样到0.01-0.05mm
![image](https://github.com/chsl/PLADE/blob/master/plane_based_registration/image/downsample1.png)
![image](https://github.com/chsl/PLADE/blob/master/plane_based_registration/image/downsample2.png)  
        (2) 如果输入的点数据不包括法线信息，需要手动计算法线，按照如下进行操作
![image](https://github.com/chsl/PLADE/blob/master/plane_based_registration/image/normals.png)  
        (3) 进行平面的提取操作
![image](https://github.com/chsl/PLADE/blob/master/plane_based_registration/image/plane1.png)
![image](https://github.com/chsl/PLADE/blob/master/plane_based_registration/image/plane2.png)
提取完平面之后，默认界面是不会显示的，需要按照如下图所示切换下显示模式，同一种颜色的点表示属于同一个平面
![image](https://github.com/chsl/PLADE/blob/master/plane_based_registration/image/plane3.png)  
        (4) 保存提取的平面数据为vg格式，并同时以相同的文件名保存一份为bpn格式的数据  
        (5) 编译plane_based_registration 项目中的parse_VG文件，对上一步中保存的数据进行处理并查看平面的法向，算法实现中要求两个待配准的 点云数据中相对应的两个平面法向相同，如果不相同，需要按照提示进行下翻转操作，操作完成后，会生成相应的文件用于下一步操作  
        (6) 编译plane_based_registration 项目中的registration文件，按提示进行配准操作  