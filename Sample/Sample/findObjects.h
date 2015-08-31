#pragma once
#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <math.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include "ctrData.h"
#include <pcl/visualization/cloud_viewer.h>

template <typename T>
class FindObjects
{
public:
	FindObjects(std::vector <pcl::PointIndices> &c, typename pcl::PointCloud<T>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud);
	void find();
	void show();
	sMsgSend grab();
private:
	std::vector <pcl::PointIndices> clusters;

	std::list<sBox<T>> lstBox;
	typename pcl::PointCloud<T>::Ptr inputCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud;

	void MinMaxOnPCADirections(
		const typename pcl::PointCloud<T> &src, std::vector<Eigen::Vector4f>& points, double &dx, double &dy, double &dz);

	void GetMinMax3D(const typename pcl::PointCloud<T> &src, Eigen::Vector4f& pMin, Eigen::Vector4f& pMax, int ndx[6]);

	static bool FindObjects::compare_centre(const sBox<T>& first, const sBox<T>& second)
	{
		return first.pCentre.z > second.pCentre.z;
	}
	// comparsion,whole height.
	static bool FindObjects::compare_height(const sBox<T>& first, const sBox<T>& second)
	{
		return first.pTop.z > second.pTop.z;
	}
	//Comparison ,whole area
	static bool FindObjects::compare_area(const sBox<T>& first, const sBox<T>& second)
	{
		return first.area < second.area;
	}
};

template <typename T>
FindObjects<T>::FindObjects(std::vector <pcl::PointIndices> &c, typename pcl::PointCloud<T>::Ptr inCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud)
{
	clusters = c;
	inputCloud = inCloud;
	coloredCloud = colorCloud;
}

template <typename T>
void FindObjects<T>::find(){

	//---------------> Add oriented bounding box (OBB)	<----------------------------------------------
	for (int i = 0; i < clusters.size(); i++)
	{
		// Extract the  inliers from the input cloud
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		//Temporary ptr
		typename pcl::PointCloud<T>::Ptr cloud_cluster_temp(new typename pcl::PointCloud<T>);
		*inliers = clusters[i];
		typename pcl::ExtractIndices<T> extract;
		extract.setInputCloud(inputCloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		// Get the outliers
		extract.filter(*cloud_cluster_temp);
		//Get OBB feature
		pcl::MomentOfInertiaEstimation <T> feature_extractor;
		feature_extractor.setInputCloud(cloud_cluster_temp);
		feature_extractor.compute();
		T min_point_OBB;
		T max_point_OBB;
		T position_OBB;
		Eigen::Matrix3f rotational_matrix_OBB;
		feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

		Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);

		Eigen::Quaternionf quat(rotational_matrix_OBB);

		//stringstream robotTarget;
		std::string boxType = "Unknown Object";
		sBox<T> Box;
		Box.area = (max_point_OBB.x - min_point_OBB.x)*(max_point_OBB.y - min_point_OBB.y);
		Box.max_point_OBB = max_point_OBB;
		Box.min_point_OBB = min_point_OBB;
		pcl::getMinMax3D(*cloud_cluster_temp, min_point_OBB, max_point_OBB);
		Box.pTop = max_point_OBB;
		Box.pBottom = min_point_OBB;
		Box.orient = quat;
		Box.pCentre = position_OBB;
		Box.boxType = -1;


		//if (Box.area >= 0.15 || Box.area <= 0.07){ continue; }

		lstBox.push_back(Box);

	}
	lstBox.sort(compare_centre);



}
template <typename T>
void FindObjects<T>::show(){
	boost::shared_ptr<pcl::visualization::PCLVisualizer>  viewer(new pcl::visualization::PCLVisualizer("objects"));;
	std::list<sBox<T>>::iterator it; int i = 1;

	for (it = lstBox.begin(); it != lstBox.end(); it++)
	{
		Eigen::Vector3f position((*it).pCentre.x, (*it).pCentre.y, (*it).pCentre.z);
		viewer->addCube(position, (*it).orient, (*it).max_point_OBB.x - (*it).min_point_OBB.x, (*it).max_point_OBB.y - (*it).min_point_OBB.y, (*it).max_point_OBB.z - (*it).min_point_OBB.z, "cube" + std::to_string(i));
		viewer->addSphere((*it).pCentre, 0.01, 255, 0, 0, "s " + std::to_string(i));
		viewer->addText3D("  O " + std::to_string(i)+
			"  (" + std::to_string((*it).pCentre.x) +
			", " + std::to_string((*it).pCentre.y) +
			", " + std::to_string((*it).pCentre.z) +

			")(" + std::to_string((*it).orient.w()) +
			"," + std::to_string((*it).orient.x()) +
			", " + std::to_string((*it).orient.y()) +
			", " + std::to_string((*it).orient.z()) + ")"

			, pcl::PointXYZ((*it).pCentre.x, (*it).pCentre.y, (*it).pCentre.z), 0.01, 0, 200, 0, "t" + std::to_string(i));
		cout << "object " << i << " Area: " << (*it).area << endl;
		i++;//increment Id
	}



	viewer->addPointCloud(coloredCloud);


	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}
	

}
// comparison, centre height.
template <typename T>
void FindObjects<T>::MinMaxOnPCADirections(
	const typename pcl::PointCloud<T> &src, std::vector<Eigen::Vector4f>& points, double &dx, double &dy, double &dz)
{
	// calculate PCA
	pcl::PCA<T> pca;
	pca.setInputCloud(src.makeShared());
	Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
	Eigen::Vector3f u = eigenVectors.col(0);
	Eigen::Vector3f v = eigenVectors.col(1);
	Eigen::Vector3f n = eigenVectors.col(2);

	pcl::PointCloud<T> projected;
	pca.project(src, projected);

	int ndx[6] = { 0 };
	Eigen::Vector4f minPt(0, 0, 0, 1), maxPt(0, 0, 0, 1);
	GetMinMax3D(projected, minPt, maxPt, ndx);
	dx = maxPt.x() - minPt.x();
	dy = maxPt.y() - minPt.y();
	dz = maxPt.z() - minPt.z();

	// the seventh point is the center (0,0,0) so it's not fill in
	T p[7];
	for (int i = 0; i < 6; i++)
	{
		//p[i] = projected.points[ndx[i]];
		switch (i)
		{
		case 0: case 3:
			p[i].x = projected.points[ndx[i]].x;
			break;
		case 1: case 4:
			p[i].y = projected.points[ndx[i]].y;
			break;
		case 2: case 5:
			p[i].z = projected.points[ndx[i]].z;
			break;
		}
	}

	T pr[7];
	points.clear();
	for (int i = 0; i < 7; i++)
	{
		pca.reconstruct(p[i], pr[i]);
		Eigen::Vector4f pv(pr[i].x, pr[i].y, pr[i].z, 1.0);
		points.push_back(pv);
	}

}
template <typename T>
void FindObjects<T>::GetMinMax3D(const typename pcl::PointCloud<T> &src, Eigen::Vector4f& pMin, Eigen::Vector4f& pMax, int ndx[6])
{
	pMin.x() = FLT_MAX; pMin.y() = FLT_MAX; pMin.z() = FLT_MAX;
	pMax.x() = -FLT_MAX; pMax.y() = -FLT_MAX; pMax.z() = -FLT_MAX;
	ndx[0] = ndx[1] = ndx[2] = ndx[3] = ndx[4] = ndx[5] = 0;
	for (size_t i = 0; i < src.points.size(); ++i)
	{
		// Check if the point is invalid
		if (!pcl_isfinite(src.points[i].x) ||
			!pcl_isfinite(src.points[i].y) ||
			!pcl_isfinite(src.points[i].z))
			continue;

		if (pMin.x() > src.points[i].x)
		{
			pMin.x() = src.points[i].x;
			ndx[0] = i;
		}
		if (pMin.y() > src.points[i].y)
		{
			pMin.y() = src.points[i].y;
			ndx[1] = i;
		}
		if (pMin.z() > src.points[i].z)
		{
			pMin.z() = src.points[i].z;
			ndx[2] = i;
		}

		if (pMax.x() < src.points[i].x)
		{
			pMax.x() = src.points[i].x;
			ndx[3] = i;
		}
		if (pMax.y() < src.points[i].y)
		{
			pMax.y() = src.points[i].y;
			ndx[4] = i;
		}
		if (pMax.z() < src.points[i].z)
		{
			pMax.z() = src.points[i].z;
			ndx[5] = i;
		}
	}
}
template <typename T>
sMsgSend FindObjects<T>::grab(){
	sMsgSend msg_send;
	std::list<sBox<T>>::iterator it;
	msg_send.robotTgt.x = lstBox.begin()->pCentre.x;
	msg_send.robotTgt.y = lstBox.begin()->pCentre.y;
	msg_send.robotTgt.z = lstBox.begin()->pCentre.z;
	msg_send.robotTgt.q1 = lstBox.begin()->orient.w();
	msg_send.robotTgt.q2 = lstBox.begin()->orient.x();
	msg_send.robotTgt.q3 = lstBox.begin()->orient.y();
	msg_send.robotTgt.q4 = lstBox.begin()->orient.z();
	msg_send.objType = lstBox.begin()->boxType;
	std::cout << "tell robot to grab...\n";
	lstBox.sort(compare_height);
	it = lstBox.begin();
	msg_send.height_fst = (it++)->pTop.z;
	msg_send.height_sec = it->pTop.z;
	cout << "First height" << msg_send.height_fst << endl;
	cout << "Second height" << msg_send.height_sec << endl;
	return msg_send;
}