#pragma once
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
class RGBRegGrowing
{
public:
	RGBRegGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::IndicesPtr indices,
		double clusterMin, double clusterMax, double PointColorThr, double RegionColorThr, double distanceThr);
	~RGBRegGrowing();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColoredCloud();

	std::vector <pcl::PointIndices>& getClusters();
private:
	//output cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud;
	//output clusters

	std::vector <pcl::PointIndices> clusters;
};

