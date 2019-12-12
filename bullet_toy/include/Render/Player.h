#ifndef PLAYER
#define PLAYER

#include "Skeleton/Skeleton.h"
#include "Motion/Motion.h"
#include "Render/Drawable.h"

class BVHPlayer : public Drawable{
	public:
	BVHPlayer(SkeletonPtr skel, MotionDBPtr motionDB);
	
	void setMotionClip(int db, int start, int end);
	void display();
	void nextTimestep(int time);
	void keyboard(unsigned char key, int x, int y);

	SkeletonPtr skel;
	MotionDBPtr motionDB;
	int db, start, end, frame;
};

using BVHPlayerPtr = std::shared_ptr<BVHPlayer>;

class IKPlayer : public Drawable{
	public:
	IKPlayer(SkeletonPtr skel);
	
	void display();
	void nextTimestep(int time);
	void keyboard(unsigned char key, int x, int y);

	SkeletonPtr skel;
	std::vector<IKConstraint> constraint;
	std::vector<bool> enable;
	bool IKEnable;
	double angle;
	void pushConstraint(IKConstraint condition);
	void popConstraint();
};

using IKPlayerPtr = std::shared_ptr<IKPlayer>;

class GraphPlayer : public Drawable{
	public:
	GraphPlayer(SkeletonPtr skel, MotionGraphPtr graph);
	
	void display();
	void nextTimestep(int time);
	void keyboard(unsigned char key, int x, int y);
	void keyboardUp(unsigned char key, int x, int y);

	SkeletonPtr skel;
	MotionGraphPtr graph;
	MotionStatusPtr status;
};

using GraphPlayerPtr = std::shared_ptr<GraphPlayer>;

#endif
