#include "stdafx.h"
#include "LogPCD.h"

void LogPCD::cloud_xyzi_(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud){
	if (rtDisplay){
		if (!viewer.wasStopped()){
			viewer.showCloud(cloud);	
		}
	}
	if (xyzi_capture_ASCII){
		//cloudXYZI = (cloud);
		cloudXYZI = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>(*(cloud.get())));
		frameReady = true;
		pcl::io::savePCDFileASCII("output/xyzi_ASCII" + std::to_string(cloudCount++) + ".PCD", *cloud);
		cout << "captured a PCD in ASCII\n";
		xyzi_capture_ASCII = false;
		 
	}
	else if (xyzi_capture_Binary){
	///	cloudXYZI = (cloud);
		cloudXYZI = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>(*(cloud.get())));
		frameReady = true;
		pcl::io::savePCDFileBinary("output/xyzi_Binary" + std::to_string(cloudCount++) + ".PCD", *cloud);
		cout << "captured a PCD in Binary\n";
		xyzi_capture_Binary = false;
		 
	}
	else if (xyzi_capture_Binary_Compressed){
	//	cloudXYZI = (cloud);
		cloudXYZI = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>(*(cloud.get())));
		frameReady = true;
		pcl::io::savePCDFileBinaryCompressed("output/xyzi_BinaryCompressed" + std::to_string(cloudCount++) + ".PCD", *cloud);
		cout << "captured a PCD in Binary Compressed\n";
		xyzi_capture_Binary_Compressed = false;
		 
	}
	

}
void LogPCD::cloud_xyz_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud){
	if (rtDisplay){
		if (!viewer.wasStopped()){
			viewer.showCloud(cloud);
		}
		
	}
	if (xyz_capture_ASCII){
		//cloudXYZ = (cloud);
		cloudXYZ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*(cloud.get())));
		frameReady = true;
		pcl::io::savePCDFileASCII("output/xyz_ASCII" + std::to_string(cloudCount++) + ".PCD", *cloud);
		cout << "captured a PCD in ASCII\n";
		xyz_capture_ASCII = false;

	}
	else if (xyz_capture_Binary){
		//cloudXYZ = (cloud);
		cloudXYZ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*(cloud.get())));
		frameReady = true;
		pcl::io::savePCDFileBinary("output/xyz_Binary" + std::to_string(cloudCount++) + ".PCD", *cloud);
		cout << "captured a PCD in Binary\n";
		xyz_capture_Binary = false;

	}
	else if (xyz_capture_Binary_Compressed){
		//cloudXYZ = (cloud);
		cloudXYZ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*(cloud.get())));
		frameReady = true;
		pcl::io::savePCDFileBinaryCompressed("output/xyz_BinaryCompressed" + std::to_string(cloudCount++) + ".PCD", *cloud);
		cout << "captured a PCD in Binary Compressed\n";
		xyz_capture_Binary_Compressed = false;

	}


}
void LogPCD::cloud_xyzrgb_(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud){
	if (rtDisplay){
		if (!viewer.wasStopped()){
			viewer.showCloud(cloud);
		}
	}
	if (xyzrgb_capture_ASCII){
		//cloudXYZRGB = (cloud);
		cloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>(*(cloud.get())));
		frameReady = true;
		pcl::io::savePCDFileASCII("output/xyzrgb_ASCII" + std::to_string(cloudCount++) + ".PCD", *cloud);
		cout << "captured a PCD in ASCII\n";
		xyzrgb_capture_ASCII = false;
		
	}
	else if (xyzrgb_capture_Binary){
		//cloudXYZRGB = (cloud);
		cloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>(*(cloud.get())));
		frameReady = true;
		pcl::io::savePCDFileBinary("output/xyzrgb_Binary" + std::to_string(cloudCount++) + ".PCD", *cloud);
		cout << "captured a PCD in Binary\n";
		xyzrgb_capture_Binary = false;
		 
		
	}
	else if (xyzrgb_capture_Binary_Compressed){
		//cloudXYZRGB = (cloud);
		cloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>(*(cloud.get())));
		frameReady = true;
		pcl::io::savePCDFileBinaryCompressed("output/xyzrgb_BinaryCompressed" + std::to_string(cloudCount++) + ".PCD", *cloud);
		cout << "captured a PCD in Binary Compressed\n";
		xyzrgb_capture_Binary_Compressed = false;
		 
		
	}

}
void LogPCD::startGrabber(){
	//mode0:XYZRGB mode1:XYZI
	// Create Kinect2Grabber
	frameReady = false;
	grabber = new pcl::Kinect2Grabber();
	grabbing = true;
	if (inputCloudFormat == XYZRGB){
		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> function =
			boost::bind(&LogPCD::cloud_xyzrgb_, this, _1);
		grabber->registerCallback(function);
	}

	else if (inputCloudFormat == XYZ){
		boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> function =
			boost::bind(&LogPCD::cloud_xyz_, this, _1);
		// Regist Callback Function
		grabber->registerCallback(function);
	}
	
	else if (inputCloudFormat == XYZI){
		boost::function<void(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr&)> function =
			boost::bind(&LogPCD::cloud_xyzi_, this, _1);
		// Regist Callback Function
		grabber->registerCallback(function);
	}
	
	// Start Retrieve Data
	grabber->start();

	
	
	
}
void LogPCD::stopGrabber(){
	grabber->stop();
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	grabber->~Grabber();
	
}

void LogPCD::capture(int format){
	frameReady = false;
	switch (format){
	case BINARY_COMPR_FORMAT:
		xyzi_capture_Binary_Compressed = true; xyzrgb_capture_Binary_Compressed = true; xyz_capture_Binary_Compressed = true;
		break;
	case BINARY_FORMAT:
		xyzi_capture_Binary = true; xyzrgb_capture_Binary = true; xyz_capture_Binary = true;
		break;
	case ASCII_FORMAT:
		xyzi_capture_ASCII = true; xyzrgb_capture_ASCII = true; xyz_capture_ASCII = true;
		break;
	}
}
bool LogPCD::checkFrame(){
	return frameReady;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LogPCD::getLastCloudXYZI(){
	//return a packed Ptr instead of a ConstPtr
	return  cloudXYZI;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr LogPCD::getLastCloudXYZRGB(){
	
	return  cloudXYZRGB;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr LogPCD::getLastCloudXYZ(){

	return  cloudXYZ;
}