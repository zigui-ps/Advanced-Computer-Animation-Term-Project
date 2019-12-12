#ifndef MOTIONDB
#define MOTIONDB

#include<Eigen/Dense>
#include<Eigen/Geometry>
#include<vector>
#include<string>
#include<memory>
#include "Skeleton/Skeleton.h"

class MotionDB{
	public:
		MotionDB(std::vector<std::vector<SkeletonPosePtr>> motionData);
		MotionDB(std::vector<std::string> motionFiles);
		SkeletonPosePtr getPosition(int movie, int frame);
		std::vector<std::vector<SkeletonPosePtr>> motionData;

		SkeletonPtr skel;
};

using MotionDBPtr = std::shared_ptr<MotionDB>;

#endif
