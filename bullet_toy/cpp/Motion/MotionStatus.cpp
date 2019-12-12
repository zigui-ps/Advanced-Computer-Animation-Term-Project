#include "Motion/MotionStatus.h"
#include "Skeleton/Helper.h"
#include<algorithm>

const int BLENDINGFRAME = 10;
const double PI = acos(-1);

// Graph

MotionStatusGraph::MotionStatusGraph(MotionGraphPtr graph, int node, int edge, int frame, double Yrot, double Xpos, double Zpos):
	graph(graph), node(node), edge(edge), frame(frame), Yrot(Yrot), Xpos(Xpos), Zpos(Zpos){
}

SkeletonPosePtr MotionStatusGraph::getPosition(){
	SkeletonPosePtr cur = SkeletonPosePtr(new SkeletonPose());
	SkeletonPosePtr base = graph->getPosition(node, edge, frame);
	cur->location = base->location; cur->joint = base->joint;
	rotateY(cur, Yrot); cur->location += Eigen::Vector3d(Xpos, 0, Zpos);
	return cur;
}

MotionStatusPtr MotionStatusGraph::step(MotionStatusPtr self){
	SkeletonPosePtr to = getPosition();
	if(!graph->step(node, edge, frame)) return self;
	else{	
		Xpos = Zpos = Yrot = 0;
		SkeletonPosePtr from = getPosition();
		
		Yrot = getYrot(from, to); 
		rotateY(from, Yrot);
		Xpos = to->location[0] - from->location[0]; from->location[0] += Xpos;
		Zpos = to->location[2] - from->location[2]; from->location[2] += Zpos;
		SkeletonPosePtr diff = getPoseDifference(to, from);
		graph->step(node, edge, frame);
		return MotionStatusOffsetPtr(new MotionStatusOffset(self, diff));
	}
}

// Offset

MotionStatusOffset::MotionStatusOffset(MotionStatusPtr to, SkeletonPosePtr diff):
	to(to), diff(diff), currentFrame(0){
}

SkeletonPosePtr MotionStatusOffset::getPosition(){
	SkeletonPosePtr result = to->getPosition();
	if(currentFrame >= BLENDINGFRAME) return result;

	double rate = std::clamp(1 - currentFrame / (double)BLENDINGFRAME, 0.0, 1.0);
	rate = (sin(rate*PI - PI/2) + 1) / 2;

	// position
	result->location = result->location + diff->location * rate;
	// rotation
	for(int i = 0; i < result->joint.size(); i++){
		Eigen::AngleAxisd tmp = Eigen::AngleAxisd(diff->joint[i]); tmp.angle() *= rate;
		result->joint[i] = tmp * result->joint[i];
	}
	return result;
}

MotionStatusPtr MotionStatusOffset::step(MotionStatusPtr self){
	currentFrame += 1;
	to = to->step(to);
	if(currentFrame >= BLENDINGFRAME) return to;
	else return self;
}
