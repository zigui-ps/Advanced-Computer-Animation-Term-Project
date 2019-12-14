#ifndef MOTIONSTATUS
#define MOTIONSTATUS

#include "Motion/MotionGraph.h"
#include "Skeleton/Skeleton.h"

class MotionStatus;
using MotionStatusPtr = std::shared_ptr<MotionStatus>;

class MotionStatus{
	public:
		virtual SkeletonPosePtr getPosition() = 0;
		virtual MotionStatusPtr step(MotionStatusPtr self) = 0;
		virtual bool isTerminate() = 0;
};

class MotionStatusGraph : public MotionStatus{
	public:
		MotionStatusGraph(MotionGraphPtr graph, int node, int edge, int frame, double Yrot, double Xpos, double Zpos);
		int node, edge, frame;
		double Yrot, Xpos, Zpos; // offset
		MotionGraphPtr graph;
		
		virtual SkeletonPosePtr getPosition();
		virtual MotionStatusPtr step(MotionStatusPtr self);
		virtual bool isTerminate();
};

class MotionStatusOffset : public MotionStatus{
	public:
		MotionStatusOffset(MotionStatusPtr to, SkeletonPosePtr diff);
		MotionStatusPtr to;
		SkeletonPosePtr diff;
		int currentFrame;
		
		virtual SkeletonPosePtr getPosition();
		virtual MotionStatusPtr step(MotionStatusPtr self);
		virtual bool isTerminate();
};

using MotionStatusGraphPtr = std::shared_ptr<MotionStatusGraph>;
using MotionStatusOffsetPtr = std::shared_ptr<MotionStatusOffset>;

#endif