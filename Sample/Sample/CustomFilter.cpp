#include "stdafx.h"
#include "CustomFilter.h"



/*
CustomFilter::CustomFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr *input, double xmin, double xmax, double ymin, double ymax){
	input_cloud_XYZRGB = input;
	std::cout << xmin << xmax << ymin << ymax <<std::endl;
	std::cout << "before filtering " << (*input)->points.size() << std::endl;
	filtered_cloud_XYZRGB = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_XYZRGB(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud((*input_cloud_XYZRGB));//x direction
	pass.setFilterFieldName("x");
	pass.setFilterLimits(xmin, xmax);
	pass.filter(*temp_cloud_XYZRGB);

	pass.setInputCloud(temp_cloud_XYZRGB);//y direction
	pass.setFilterFieldName("y");
	pass.setFilterLimits(ymin, ymax);
	pass.filter(*filtered_cloud_XYZRGB);

	std::cout << "after filtering " << filtered_cloud_XYZRGB->points.size();
}
CustomFilter::CustomFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr *input, double xmin, double xmax, double ymin, double ymax){
	input_cloud_XYZI = input;
	std::cout << xmin << xmax << ymin << ymax << std::endl;
	std::cout << "before filtering " << (*input)->points.size() << std::endl;
	filtered_cloud_XYZI = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_XYZI(new pcl::PointCloud<pcl::PointXYZI>);

	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud((*input_cloud_XYZI));//x direction
	pass.setFilterFieldName("x");
	pass.setFilterLimits(xmin, xmax);
	pass.filter(*temp_cloud_XYZI);

	pass.setInputCloud(temp_cloud_XYZI);//y direction
	pass.setFilterFieldName("y");
	pass.setFilterLimits(ymin, ymax);
	pass.filter(*filtered_cloud_XYZI);

	std::cout << "after filtering " << filtered_cloud_XYZI->points.size();
}
*/


/*
pcl::PointCloud<pcl::PointXYZI>::Ptr *CustomFilter::getOutputXYZI(){
	return &filtered_cloud_XYZI;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr *CustomFilter::getOutputXYZRGB(){
	return &filtered_cloud_XYZRGB;
	}*/