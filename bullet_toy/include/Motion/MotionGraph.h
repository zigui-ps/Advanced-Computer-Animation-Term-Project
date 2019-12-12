#ifndef MOTIONGRAPH
#define MOTIONGRAPH

#include "Motion/MotionDB.h"
#include <vector>

class MotionGraph{
public:
	class MotionGraphEdge{
		public:
		MotionGraphEdge(int db, int nxt, int key);
		int db, nxt, key;
	};

	MotionDBPtr motionDB;
	std::vector<std::vector<MotionGraphEdge>> graph;
	int key;

	MotionGraph(std::string infoFile);
	SkeletonPosePtr getPosition(int node, int edge, int frame);
	bool step(int &node, int &edge, int &frame);
};

using MotionGraphPtr = std::shared_ptr<MotionGraph>;

#endif