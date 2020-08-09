# PLADE  
##  PLADE: A Plane-based Descriptor for Point Cloud Registration with Small Overlap  

## 使用说明：  
1. 下载整个代码仓，在visual studio中打开，入口文件为 "PLADE.sln"  
1. 依赖的第三方库：  
-     1、Opencv  
-     2、boost  
-     3、PCL  
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
        