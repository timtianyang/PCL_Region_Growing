// Sample.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#define WIN32_LEAN_AND_MEAN
#include "LogPCD.h"
#include "CustomFilter.h"
#include <string.h>
#include "RGBRegGrowing.h"
#include "NormalRegGrowing.h"
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include "findObjects.h"
#include "robot.h"

double xmin, xmax = 0, ymin, ymax = 0, zmin, zmax = 0, leafSize, minCluster, maxCluster, pointColorThr, regionColorThr, distanceThr;
double smoothThr, curThr;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudxyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr cloudxyzi(new pcl::PointCloud<pcl::PointXYZI>());
time_t t_start, t_end, t_total = 0, t_diff; //For performance observation

sMsgSend processXYZ(bool show);
sMsgSend processXYZRGB(bool show);

int main(int argc, char** argv)
{
	if (!SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS)){
		std::cout << "failed to set priority...";
	}

	//parsing commnadline args
	std::string fileName;
	int cloudFormat = 0;//xyzrgb=0 xyz=1 xyzi=2
	int outFormat = 0;//binary=0 binaryCompressed=1 Ascii=2
	bool rtDisplay = false;//show camera update in realtime
	pcl::console::parse_argument(argc, argv, "-Xmax", xmax);
	pcl::console::parse_argument(argc, argv, "-Ymax", ymax);
	pcl::console::parse_argument(argc, argv, "-Zmax", zmax);
	pcl::console::parse_argument(argc, argv, "-Xmin", xmin);
	pcl::console::parse_argument(argc, argv, "-Ymin", ymin);
	pcl::console::parse_argument(argc, argv, "-Zmin", zmin);
	pcl::console::parse_argument(argc, argv, "-M", cloudFormat);
	pcl::console::parse_argument(argc, argv, "-OutFormat", outFormat);
	pcl::console::parse_argument(argc, argv, "-FileName", fileName);
	pcl::console::parse_argument(argc, argv, "-LeafSize", leafSize);
	pcl::console::parse_argument(argc, argv, "-MinCluster", minCluster);
	pcl::console::parse_argument(argc, argv, "-MaxCluster", maxCluster);
	pcl::console::parse_argument(argc, argv, "-PointColorThr", pointColorThr);
	pcl::console::parse_argument(argc, argv, "-RegionColorThr", regionColorThr);
	pcl::console::parse_argument(argc, argv, "-DistanceThr", distanceThr);
	pcl::console::parse_argument(argc, argv, "-SmThr", smoothThr);
	pcl::console::parse_argument(argc, argv, "-CurThr", curThr);
	if (pcl::console::find_argument(argc, argv, "-Rt") >= 0){ rtDisplay = true; }
	//*******************************************************************************************
	//*******************************************************************************************
	//offline mode
	if (pcl::console::find_argument(argc, argv, "-ofl") >= 0){

		//XYZRGB
		if (cloudFormat == LogPCD::XYZRGB){
			if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(fileName, *cloudxyzrgb) == -1){
				std::cout << "Error opening the file!";
				boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
				return -1;
			}
			else{
				std::cout << "Opening the file!";
				pcl::visualization::CloudViewer viewerInput("Iutput");
				viewerInput.showCloud(cloudxyzrgb);
				while (!viewerInput.wasStopped()){}
				processXYZRGB(true);
			}
		}
		//XYZ
		else if (cloudFormat == LogPCD::XYZ){
			if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *cloudxyz) == -1){
				std::cout << "Error opening the file!";
				boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
				return -1;
			}
			else{
				std::cout << "Opening the file!";
				pcl::visualization::CloudViewer viewerInput("Iutput");
				viewerInput.showCloud(cloudxyz);
				while (!viewerInput.wasStopped()){}
				processXYZ(true);
			}
		}
		//XYZI

		else if (cloudFormat == LogPCD::XYZI){
			if (pcl::io::loadPCDFile<pcl::PointXYZI>(fileName, *cloudxyzi) == -1){
				std::cout << "Error opening the file!";
				boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
				return -1;
			}
			else{
				std::cout << "Opening the file!\n";
				std::cout << "converting XYZI to XYZRGB...\n";

				cloudxyzrgb->points.resize(cloudxyzi->height * cloudxyzi->width);
				pcl::PointXYZRGB* ptrgb = &cloudxyzrgb->points[0];
				pcl::PointXYZI* pti = &cloudxyzi->points[0];
				for (int y = 0; y < cloudxyzi->height; y++)
				{
					for (int x = 0; x < cloudxyzi->width; x++, ptrgb++, pti++)
					{
						ptrgb->r = pti->intensity*0.3;
						ptrgb->g = pti->intensity*0.4;
						ptrgb->b = pti->intensity*0.3;
						ptrgb->x = pti->x;
						ptrgb->y = pti->y;
						ptrgb->z = pti->z;




					}

				}

				std::cout << "done...\n";
				pcl::visualization::CloudViewer viewerInput("Iutput");
				viewerInput.showCloud(cloudxyzrgb);
				while (!viewerInput.wasStopped()){}
				processXYZRGB(true);
			}
		}


	}
	//*******************************************************************************************
	//*******************************************************************************************
	//online mode
	else{
		LogPCD logpcd(cloudFormat, outFormat, rtDisplay);//displays and logs point clouds
		robot ro;
		//waiting for trigger
		bool appRunning = true;
		while (appRunning){
			switch (ro.waitForCommand()){
			case 0:
				continue;
			case 1:
				logpcd.startGrabber();
				std::cout << "capture binary command received" << std::endl;
				boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
				logpcd.capture(LogPCD::BINARY_FORMAT);
				break;
			case 100:
				return 0;
			}
			logpcd.stopGrabber();

			while (!logpcd.checkFrame());//wait till the frame is actually captured
			//get cloud out of the logger
			if (logpcd.getLastCloudXYZRGB()){ cloudxyzrgb = logpcd.getLastCloudXYZRGB(); }
			else if (logpcd.getLastCloudXYZ()){ cloudxyz = logpcd.getLastCloudXYZ(); }
			else{ return 0; }

			sMsgSend m;
			if (cloudFormat == LogPCD::XYZRGB){ m = processXYZRGB(rtDisplay); }
			else if (cloudFormat == LogPCD::XYZ){ m = processXYZ(rtDisplay); }

			ro.SendMessageW(&m);
		}
	}
	return 0;
}

sMsgSend processXYZ(bool sh){
	t_start = GetTickCount();
	CustomFilter<pcl::PointXYZ> mFilterXYZ(cloudxyz, xmin, xmax, ymin, ymax, zmin, zmax, leafSize);
	t_end = GetTickCount();
	t_diff = difftime(t_end, t_start);
	std::cout << "spent " << t_diff << "ms in passthrough filter\n";
	t_total += t_diff;
	t_start = GetTickCount();
	NormalRegGrowing regGrowing(mFilterXYZ.getOutput(), mFilterXYZ.getIndices(), minCluster, maxCluster, smoothThr, curThr);
	t_end = GetTickCount();
	t_diff = difftime(t_end, t_start);
	std::cout << "spent " << t_diff << "ms in RGB region growing algorithm\n";
	t_total += t_diff;


	t_start = GetTickCount();
	FindObjects<pcl::PointXYZ> finder(regGrowing.getClusters(), mFilterXYZ.getOutput(), regGrowing.getColoredCloud());
	finder.find();
	t_end = GetTickCount();
	t_diff = difftime(t_end, t_start);
	std::cout << "spent " << t_diff << "ms in finding\ objectsn";
	t_total += t_diff;
	std::cout << "spent " << t_total << "ms in total\n";
	if (sh){ finder.show(); }
	return finder.grab();
}
sMsgSend processXYZRGB(bool sh){
	t_start = GetTickCount();
	CustomFilter<pcl::PointXYZRGB> mFilterXYZRGB(cloudxyzrgb, xmin, xmax, ymin, ymax, zmin, zmax, leafSize);
	t_end = GetTickCount();
	t_diff = difftime(t_end, t_start);
	std::cout << "spent " << t_diff << "ms in passthrough filter\n";
	t_total += t_diff;
	t_start = GetTickCount();
	RGBRegGrowing regGrowing(mFilterXYZRGB.getOutput(), mFilterXYZRGB.getIndices(), minCluster, maxCluster, pointColorThr, regionColorThr, distanceThr);
	t_end = GetTickCount();
	t_diff = difftime(t_end, t_start);
	std::cout << "spent " << t_diff << "ms in RGB region growing algorithm\n";
	t_total += t_diff;


	t_start = GetTickCount();
	FindObjects<pcl::PointXYZRGB> finder(regGrowing.getClusters(), mFilterXYZRGB.getOutput(), regGrowing.getColoredCloud());
	finder.find();
	t_end = GetTickCount();
	t_diff = difftime(t_end, t_start);
	std::cout << "spent " << t_diff << "ms in finding\ objectsn";
	t_total += t_diff;
	std::cout << "spent " << t_total << "ms in total\n";
	if (sh){ finder.show(); }
	return finder.grab();

}
