#include "stdafx.h"
#include "NormalRegGrowing.h"


NormalRegGrowing::NormalRegGrowing(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::IndicesPtr indices,
	double clusterMin, double clusterMax, double smoothThr, double curThr)
{

	pcl::search::Search <pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	compute_surface_normals(input, 30, normals);
	reg.setInputCloud(input);
	//reg.setIndices(indices);
	reg.setSearchMethod(tree);
	reg.setInputNormals(normals);
	reg.setMinClusterSize(clusterMin);
	reg.setMaxClusterSize(clusterMax);

	//Adjust this paramter to seperate touching boxes
	reg.setSmoothnessThreshold(smoothThr / 180.0 * M_PI);
	reg.setCurvatureThreshold(curThr);

	reg.setNumberOfNeighbours(16);

	std::cout << "extracting clusters..." << std::endl;
	reg.extract(clusters);
	if (clusters.size() == 0) {
		std::cout << "recognized 0 objects! Please exit and adjust parameters...";
		pcl::visualization::PCLVisualizer viewer("find obj");
		viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(input, normals,10,10);
		viewer.setBackgroundColor(0, 0, 0);
		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}

		while (1);
	}
	for (int i = 0; i < clusters.size(); i++)
	{
		std::cout << "NO. " << (i + 1) << " object has " << clusters[i].indices.size() << " points" << std::endl;
	}
	std::cout << "Number of objects is  " << clusters.size() << std::endl;
	filtered_cloud = reg.getColoredCloud();

	if (filtered_cloud.get() != NULL){ std::cout << "after filtering " << filtered_cloud->points.size() << std::endl; }
	else{ std::cout << "Normal region growing failed..\n"; }
}

//Calculate normal
void NormalRegGrowing::compute_surface_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr &points, float normal_radius,
	pcl::PointCloud<pcl::Normal>::Ptr &normals_out)
{
	//create the normal estimation object
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est(omp_get_max_threads());
	// Use a FLANN-based KdTree to perform neighborhood searches
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	// Specify the size of the local neighborhood to use when computing the surface normals
	norm_est.setKSearch(normal_radius);
	// Set the input points
	norm_est.setInputCloud(points);
	// Estimate the surface normals and store the result in "normals_out"
	norm_est.compute(*normals_out);
}




pcl::PointCloud<pcl::PointXYZRGB>::Ptr NormalRegGrowing::getColoredCloud(){
	return filtered_cloud;
}
std::vector <pcl::PointIndices>& NormalRegGrowing::getClusters(){

	return clusters;
}


