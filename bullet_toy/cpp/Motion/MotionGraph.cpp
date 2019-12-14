#include "Motion/MotionGraph.h"
#include "Motion/Parser.h"
#include <map>
#include <string>
#include <fstream>

MotionGraph::MotionGraphEdge::MotionGraphEdge(int db, int nxt, int key):db(db), nxt(nxt), key(key){}

MotionGraph::MotionGraph(std::string infoFile) : key(0){
	std::ifstream info(infoFile);

	int cnt = 0;
	std::map<std::string, int> nodeList;
	std::map<std::string, int> motionList;
	std::vector<std::string> motion;

	std::string tmp;
	info >> tmp; assert(tmp == "NODE");
	while(true){
		info >> tmp;
		if(tmp == "END") break;
		nodeList[tmp] = cnt++;
	}
	graph.resize(cnt);
	
	info >> tmp;
	assert(tmp == "MOTION");
	while(true){
		info >> tmp; if(tmp == "END") break; motion.push_back(tmp);
		info >> tmp; motionList[tmp] = (int)motion.size() - 1;
	}
	SkeletonPtr skel;
	motionDB = MotionDBPtr(new MotionDB(motion));

	info >> tmp;
	assert(tmp == "EDGE");
	for(int i = 0; i < cnt; i++){
		info >> tmp; assert(tmp == "START");
		info >> tmp; assert(nodeList[tmp] == i);
		while(true){
			int db, key, nxt;
			info >> tmp; if(tmp == "END") break; db = motionList[tmp];
			info >> tmp; nxt = nodeList[tmp];
			info >> tmp; key = tmp[0];
			graph[i].emplace_back(db, nxt, key);
		}
	}
}

SkeletonPosePtr MotionGraph::getPosition(int node, int edge, int frame){
	int db = graph[node][edge].db;
	return motionDB->getPosition(db, frame);
}

bool MotionGraph::step(int &node, int &edge, int &frame){
	++frame;
	MotionGraphEdge currentEdge = graph[node][edge];
	if (motionDB->motionData[currentEdge.db].size() == frame + 1){
		node = currentEdge.nxt;
		if(graph[node].size() == 0){
			edge = frame = -1;
			return false;
		}
		MotionGraphEdge nextEdge = graph[node][0]; edge = 0;
		for (int i = 1; i < graph[node].size(); i++){
			if(graph[node][i].key == key){
				nextEdge = graph[node][i];
				edge = i;
			}
		}
		frame = 0; 
		return true;
	}
	else return false;
}
