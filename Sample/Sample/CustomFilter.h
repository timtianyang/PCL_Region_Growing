#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
template <typename T>
class CustomFilter
{
public:
	CustomFilter(typename pcl::PointCloud<T>::Ptr input, double xmin, double xmax, double ymin, double ymax
		, double zmin, double zmax, double leafSize);
	typename pcl::PointCloud<T>::Ptr getOutput();
	pcl::IndicesPtr getIndices();
	//void run();
private:
	//output cloud
	typename pcl::PointCloud<T>::Ptr filtered_cloud;
	//keep track of input
	typename pcl::PointCloud<T>::Ptr *input_cloud;

	pcl::IndicesPtr indices;

};

template <typename T>
CustomFilter<T>::CustomFilter(typename pcl::PointCloud<T>::Ptr input, double xmin, double xmax, double ymin, double ymax
	, double zmin, double zmax,	double leafSize	){

	input_cloud = &input;
	std::cout << xmin << xmax << ymin << ymax << std::endl;
	std::cout << "before filtering " << (*input_cloud)->points.size() << std::endl;
	filtered_cloud = typename pcl::PointCloud<T>::Ptr(new typename pcl::PointCloud<T>);



	indices= pcl::IndicesPtr(new std::vector <int>);

	//filtered_cloud = pcl::PointCloud<T>::Ptr(new pcl::PointCloud<T>);
	pcl::PointCloud<T>::Ptr temp_cloud(new pcl::PointCloud<T>);
	pcl::PointCloud<T>::Ptr temp_cloud2(new pcl::PointCloud<T>);

	pcl::PassThrough<T> pass;
	pass.setInputCloud((*input_cloud));//x direction
	pass.setFilterFieldName("x");
	pass.setFilterLimits(xmin, xmax);
	pass.filter(*filtered_cloud);
	//std::cout << "after X pass through " << filtered_cloud->points.size() << std::endl;
	pass.setInputCloud(filtered_cloud);//y direction
	pass.setFilterFieldName("y");
	pass.setFilterLimits(ymin, ymax);
	pass.filter(*temp_cloud);
	//std::cout << "after Y pass through " << temp_cloud->points.size() << std::endl;
	
	pass.setInputCloud(temp_cloud);//z direction
	pass.setFilterFieldName("z");
	pass.setFilterLimits(zmin, zmax);
	pass.filter(*temp_cloud2);
	std::cout << "after pass through " << temp_cloud2->points.size() << std::endl;
	

	
	/*
	pass.setInputCloud(temp_cloud);//z direction
	pass.setFilterFieldName("z");
	pass.setFilterLimits(zmin, zmax);
	pass.filter(*filtered_cloud);
	std::cout << "after Z pass through " << temp_cloud2->points.size() << std::endl;
	*/


	pcl::VoxelGrid<T> vg;
	vg.setInputCloud(temp_cloud2);
	//Test to find appropriate leaf size to speed the algorithm but no lost key feature
	vg.setLeafSize(leafSize, leafSize, leafSize );
	vg.filter(*filtered_cloud);
	std::cout << "afterVoxelGrid " << filtered_cloud->points.size() << std::endl;
	pass.setInputCloud(filtered_cloud);
	pass.filter(*indices);
	std::cout << "indices done " << std::endl;
	
}
template <typename T>
typename pcl::PointCloud<T>::Ptr CustomFilter<T>::getOutput(){
	return filtered_cloud;
}
template <typename T>
pcl::IndicesPtr CustomFilter<T>::getIndices(){
	return indices;
}