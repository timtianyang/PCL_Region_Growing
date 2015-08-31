#pragma once
#include <list>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#pragma pack(1)
typedef struct
{
	float x;
	float y;
	float z;
	float q1;
	float q2;
	float q3;
	float q4;
} sRobotTarget;

typedef struct
{
	char time[14];
	sRobotTarget robotTgt;
	float height_fst;
	float height_sec;
	int objType;
} sMsgSend;

template <typename T>
struct sBox
{
	float dim_l;
	float dim_w;
	float dim_h;
	float area;
	T pCentre;
	T pTop;
	T pBottom;
	T max_point_OBB;
	T min_point_OBB;
	Eigen::Quaternionf orient;
	int boxType;
};
#pragma pack()