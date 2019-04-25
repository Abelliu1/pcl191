/************************************************************************/
/* test1: test pcl1.9.1---成功。具体的属性列表已经保存
*/
/************************************************************************/
/*
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include<fstream>  
#include <string>  
#include <vector> 

using namespace std;

int main()
{
	typedef struct tagPOINT_3D
	{
		double x;  //mm world coordinate x  
		double y;  //mm world coordinate y  
		double z;  //mm world coordinate z  
		double r;
	}POINT_WORLD;


	/////加载txt数据  
	int number_Txt;
	FILE *fp_txt;
	tagPOINT_3D TxtPoint;
	vector<tagPOINT_3D> m_vTxtPoints;
	fp_txt = fopen("za.txt", "r");
	if (fp_txt)
	{
		while (fscanf(fp_txt, "%lf %lf %lf", &TxtPoint.x, &TxtPoint.y, &TxtPoint.z) != EOF)
		{
			m_vTxtPoints.push_back(TxtPoint);
		}
	}
	else
		cout << "txt数据加载失败！" << endl;
	number_Txt = m_vTxtPoints.size();
	//pcl::PointCloud<pcl::PointXYZ> cloud;
	//这里使用“PointXYZ”是因为我后面给的点云信息是包含的三维坐标，同时还有点云信息包含的rgb颜色信息的或者还有包含rgba颜色和强度信息。
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the cloud data  
	cloud->width = number_Txt;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = m_vTxtPoints[i].x;
		cloud->points[i].y = m_vTxtPoints[i].y;
		cloud->points[i].z = m_vTxtPoints[i].z;
	}
	pcl::io::savePCDFileASCII("txt2pcd_bunny1.pcd", *cloud);
	std::cerr << "Saved " << cloud->points.size() << " data points to txt2pcd.pcd." << std::endl;

	//for (size_t i = 0; i < cloud.points.size(); ++i)
	//  std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

	//PCL Visualizer
	// Viewer  
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.addPointCloud(cloud);
	viewer.setBackgroundColor(0, 0, 0);

	viewer.spin();
	system("pause");
	return 0;

}*/


/************************************************************************/
/* test2: rops estimate
*/
/************************************************************************/

#include <pcl/point_cloud.h>
#include <pcl/features/rops_estimation.h>
#include <pcl/io/pcd_io.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
pcl::PointIndicesPtr indices;
std::vector<pcl::Vertices> triangles;

int main()
{
	cloud = (new pcl::PointCloud<pcl::PointXYZ>())->makeShared();
	pcl::io::loadPCDFile("rops_cloud.pcd",*cloud);

	indices = boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices());
	std::ifstream indices_file;
	indices_file.open("rops_indices.txt", std::ifstream::in);
    for (std::string line;std::getline(indices_file,line);)
    {
		std::istringstream in(line);
		unsigned int index = 0;
		in >> index;
		indices->indices.push_back(index - 1);
    }
	indices_file.close();

	std::ifstream trianges_file;
	trianges_file.open("rops_triangles.txt", std::ifstream::in);
	for (std::string line;std::getline(trianges_file,line);)
	{
		pcl::Vertices triangle;
		std::istringstream in(line);
		unsigned int vertex = 0;
		in >> vertex;
		triangle.vertices.push_back(vertex - 1);
		in >> vertex;
		triangle.vertices.push_back(vertex - 1);
		in >> vertex;
		triangle.vertices.push_back(vertex - 1);
        
		triangles.push_back(triangle);
	}


	float support_raduis = 0.0285f;
	unsigned int number_of_partition_bins = 5;
	unsigned int number_of_rotaions = 3;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method(new pcl::search::KdTree<pcl::PointXYZ>);
	search_method->setInputCloud(cloud);

	pcl::ROPSEstimation<pcl::PointXYZ, pcl::Histogram<135>> feature_estimator;
	feature_estimator.setSearchMethod(search_method);
	feature_estimator.setSearchSurface(cloud);
	feature_estimator.setInputCloud(cloud);
	feature_estimator.setIndices(indices);
	feature_estimator.setTriangles(triangles);
	feature_estimator.setRadiusSearch(support_raduis);
	feature_estimator.setNumberOfPartitionBins(number_of_partition_bins);
	feature_estimator.setNumberOfRotations(number_of_rotaions);
	feature_estimator.setSupportRadius(support_raduis);

	pcl::PointCloud<pcl::Histogram<135>>::Ptr hisograms(new pcl::PointCloud<pcl::Histogram<135>>());
	support_raduis = -support_raduis;
	feature_estimator.setSupportRadius(support_raduis);
	support_raduis = feature_estimator.getSupportRadius();
	std::cout << "support raduis: " << support_raduis << std::endl;

	number_of_partition_bins = 0;
	feature_estimator.setNumberOfPartitionBins(number_of_partition_bins);
	number_of_partition_bins = feature_estimator.getNumberOfPartitionBins();
	std::cout << "number of partition bins: " << number_of_partition_bins<<std::endl;

	number_of_rotaions = 0;
	feature_estimator.setNumberOfRotations(number_of_rotaions);
	number_of_rotaions = feature_estimator.getNumberOfRotations();
	std::cout << "number of rotations: " << number_of_rotaions<<std::endl;

	std::vector<pcl::Vertices> empty_trianlges;
	feature_estimator.setTriangles(empty_trianlges);
	feature_estimator.compute(*hisograms);
	std::cout << "histograms points size: " << hisograms->points.size() << std::endl;

	system("pause");
	return 0;
}