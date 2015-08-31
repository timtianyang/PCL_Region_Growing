#include "stdafx.h"
#include "RGBRegGrowing.h"


RGBRegGrowing::RGBRegGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::IndicesPtr indices,
	double clusterMin, double clusterMax, double PointColorThr, double RegionColorThr, double distanceThr)
{

	pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> >(new pcl::search::KdTree<pcl::PointXYZRGB>);
	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;

	reg.setInputCloud(input);
	reg.setIndices(indices);
	reg.setSearchMethod(tree);

	reg.setMinClusterSize(clusterMin);
	reg.setMaxClusterSize(clusterMax);


	reg.setDistanceThreshold(distanceThr);
	reg.setNumberOfRegionNeighbours(8);
	reg.setPointColorThreshold(PointColorThr);
	reg.setRegionColorThreshold(RegionColorThr);


	std::cout << "extracting clusters..." << std::endl;
	reg.extract(clusters);
	if (clusters.size() == 0) {
		std::cout << "recognized 0 objects! Please exit and adjust parameters...";
	/*	pcl::visualization::PCLVisualizer viewer("find obj");
		viewer.addPointCloud(reg.getColoredCloud());
		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
*/
		while (1);
	}
	for (int i = 0; i < clusters.size(); i++)
	{
		std::cout << "NO. " << (i + 1) << " object has " << clusters[i].indices.size() << " points" << std::endl;
	}
	std::cout << "Number of objects is  " << clusters.size() << std::endl;
	filtered_cloud = reg.getColoredCloud();

	if (filtered_cloud.get() != NULL){ std::cout << "after filtering " << filtered_cloud->points.size() << std::endl; }
	else{ std::cout << "RGB region growing failed..\n"; }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBRegGrowing::getColoredCloud(){
	return filtered_cloud;
}
std::vector <pcl::PointIndices>& RGBRegGrowing::getClusters(){

	return clusters;
}
RGBRegGrowing::~RGBRegGrowing()
{
}
