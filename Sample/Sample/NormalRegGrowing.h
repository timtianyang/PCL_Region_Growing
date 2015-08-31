#pragma once
#include <pcl/segmentation/region_growing.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <vector>
#include <pcl/features/normal_3d_omp.h>
#include <omp.h>
#include <pcl/visualization/cloud_viewer.h>
class NormalRegGrowing
{
public:
	NormalRegGrowing(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::IndicesPtr indices,
		double clusterMin, double clusterMax, double smoothThr, double curThr);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColoredCloud();

	std::vector <pcl::PointIndices>& getClusters();


private:
	//output cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud;
	//output clusters
	void compute_surface_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr &points, float normal_radius,
		pcl::PointCloud<pcl::Normal>::Ptr &normals_out);
	std::vector <pcl::PointIndices> clusters;
};


