#include<tinyxml.h>
#include<iostream>
#include<fstream>
#include<sstream>
#include<string>
#include<vector>

#include "Motion/MotionDB.h"
#include "Skeleton/Skeleton.h"

namespace Parser{
	struct BVHNode{
		std::string name;
		double offset[3];
		std::vector<std::string> channelList;
		std::vector<BVHNode*> child;
		void readFile(std::ifstream &file, SkeletonPosePtr pose);
	};
	std::vector<SkeletonPosePtr> Parser(std::string filename, SkeletonPtr &skel);
	std::vector<std::vector<SkeletonPosePtr>> Parser(std::vector<std::string> filename, SkeletonPtr &skel);
}

