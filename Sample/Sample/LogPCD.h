#pragma once
#include "kinect2_grabber.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>

class LogPCD{
public:
	static const int BINARY_FORMAT = 0;
	static const int BINARY_COMPR_FORMAT = 1;
	static const int ASCII_FORMAT = 2;
	static const int XYZRGB = 0;
	static const int XYZ = 1;
	static const int XYZI = 2;
	LogPCD(int mode, int outputFormat, bool rt) :viewer("Point Cloud Viewer"){
		inputCloudFormat = mode; outputFileFormat = outputFormat; rtDisplay = rt ;
		if (!rt){ viewer.~CloudViewer(); }
		
	}

	//start viewer
	void startGrabber();//mode 0:XYZRGB mode 1:XYZI

	//get the captured cloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr getLastCloudXYZI();
	pcl::PointCloud<pcl::PointXYZ>::Ptr getLastCloudXYZ();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getLastCloudXYZRGB();
	void stopGrabber();
	void capture(int cloudFormat);
	bool checkFrame();
private:
	bool frameReady;
	pcl::Grabber* grabber;
	bool grabbing=false;
	int cloudCount = 0;
	int inputCloudFormat;
	int outputFileFormat;
	bool rtDisplay;


	//captured cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB;
	//control flags
	bool xyzi_capture_ASCII, xyzi_capture_Binary, xyzi_capture_Binary_Compressed;
	bool xyz_capture_ASCII, xyz_capture_Binary, xyz_capture_Binary_Compressed;
	bool xyzrgb_capture_ASCII, xyzrgb_capture_Binary, xyzrgb_capture_Binary_Compressed;

	pcl::visualization::CloudViewer viewer;
	//callbacks
	void cloud_xyzi_(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud);
	void cloud_xyzrgb_(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);
	void cloud_xyz_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
};
