#include<cmath>
#include<vector>
#include<string>
#include<fstream>
#include<tinyxml.h>
#include "Motion/MotionDB.h"
#include "Motion/Parser.h"

// Parsing

const double PI = acos(-1);

MotionDB::MotionDB(std::vector<std::string> motionFiles){
	SkeletonPtr skel;
	motionData = Parser::Parser(motionFiles, skel);
}

MotionDB::MotionDB(std::vector<std::vector<SkeletonPosePtr>> motionData) : motionData(motionData){
}

SkeletonPosePtr MotionDB::getPosition(int db, int frame){
	if(motionData[db].size() <= frame){
		printf("WARNING: motiondata size(%d) <= status.frame(%d) + 1\n", motionData[db].size(), frame);
		int sz = motionData[db].size();
		return motionData[db][frame % sz];
	}
	return motionData[db][frame];
}
