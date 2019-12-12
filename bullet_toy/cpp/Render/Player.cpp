#include "Render/Player.h"
#include "Render/GLfunctions.h"
#include "Skeleton/Helper.h"

BVHPlayer::BVHPlayer(SkeletonPtr skel, MotionDBPtr motionDB):skel(skel), motionDB(motionDB),
	db(-1){
}

void BVHPlayer::setMotionClip(int db, int start, int end){
	this->db = db;
	this->start = this->frame = start;
	this->end = end;
}

void BVHPlayer::display()
{
}

void BVHPlayer::keyboard(unsigned char key, int x, int y){
	if('1' <= key && key <= '9'){
		int n = key - '1';
		if(motionDB->motionData.size() <= n) return;
		db = n;
		start = frame = 0;
		end = motionDB->motionData[n].size() - 1;
	}
}

void BVHPlayer::nextTimestep(int time)
{
	if(db == -1 || !motionDB) return;
	frame = ++frame == end? start : frame;
	SkeletonPosePtr pose = motionDB->getPosition(db, frame);
	skel->location = pose->location;
	for(int i = 0; i < skel->nodeList.size(); i++)
		skel->nodeList[i]->jointTransform = pose->joint[i];
}

IKPlayer::IKPlayer(SkeletonPtr skel):skel(skel), angle(0), enable(6, true), IKEnable(true){
	pushConstraint(IKConstraint(skel->nodeName["Torso"], 
		Eigen::Affine3d(Eigen::Translation3d(0, 0.0, 0))));
	pushConstraint(IKConstraint(skel->nodeName["Head"], 
		Eigen::Affine3d(Eigen::Translation3d(0, 0.9, 0))));

	pushConstraint(IKConstraint(skel->nodeName["HandR"], 
		Eigen::Affine3d(Eigen::Translation3d(-0.3, 0.5, 0.3))));
	pushConstraint(IKConstraint(skel->nodeName["HandL"], 
		Eigen::Affine3d(Eigen::Translation3d(0.3, 0.5, -0.3))));

	pushConstraint(IKConstraint(skel->nodeName["FootR"], 
		Eigen::Affine3d(Eigen::Translation3d(-0.3, -0.7, 0.3))));
	pushConstraint(IKConstraint(skel->nodeName["FootL"], 
		Eigen::Affine3d(Eigen::Translation3d(0.3, -0.7, -0.3))));

	for(int i = 0; i < skel->nodeList.size(); i++){
		Eigen::Vector3d axis = Eigen::Vector3d::Random().normalized();
		skel->nodeList[i]->jointTransform = Eigen::AngleAxisd(0.1, axis);
	}
}

void IKPlayer::display()
{
	skel->forwardKinematics();
	for(int i = 0; i < constraint.size(); i++){
		IKConstraint c = constraint[i];
		Eigen::Vector3d s = skel->nodeLocation[c.node->idx].translation();
		Eigen::Vector3d e = c.target.translation();
		glBegin(GL_LINES); glVertex3d(s[0], s[1], s[2]); glVertex3d(e[0], e[1], e[2]); glEnd();

		glPushMatrix();
		glMultMatrixd(c.target.data());

		if(enable[i] && IKEnable) glColor3f(1.0, 0.0, 0.0); 
		else glColor3f(0.0, 1.0, 0.0);
		glutSolidSphere(0.03, 20, 20);
		glColor3f(1.0, 0.0, 0.0); 
		glBegin(GL_LINES); glVertex3d(0.0, 0.0, 0.0); glVertex3d(1.0, 0.0, 0.0); glEnd();
		glColor3f(0.0, 1.0, 0.0);
		glBegin(GL_LINES); glVertex3d(0.0, 0.0, 0.0); glVertex3d(0.0, 1.0, 0.0); glEnd();
		glColor3f(0.0, 0.0, 1.0); 
		glBegin(GL_LINES); glVertex3d(0.0, 0.0, 0.0); glVertex3d(0.0, 0.0, 1.0); glEnd();

		glPopMatrix();
	}
}

void IKPlayer::keyboard(unsigned char key, int x, int y){
	static int id = 0;
	if('1' <= key && key <= '6') id = key - '1';
	if(key == 'r'){
		for(int i = 1; i < constraint.size(); i++){
			IKConstraint &c = constraint[i];
			Eigen::Vector3d axis = Eigen::Vector3d::Random().normalized();
			c.target.rotate(Eigen::AngleAxisd(0.1, axis));
		}
		return;
	}
	if(key == 'p') enable[id] = !enable[id];
	if(key == 'o') IKEnable = !IKEnable;

	Eigen::Vector3d dx;
	dx.Zero();
	if(key == 'w') dx = Eigen::Vector3d(0, 0, -1);
	if(key == 'a') dx = Eigen::Vector3d(-1, 0, 0);
	if(key == 's') dx = Eigen::Vector3d(0, 0, 1);
	if(key == 'd') dx = Eigen::Vector3d(1, 0, 0);
	if(key == 'q') dx = Eigen::Vector3d(0, 1, 0);
	if(key == 'e') dx = Eigen::Vector3d(0, -1, 0);
	dx *= 0.01;
	constraint[id].target.pretranslate(dx);
}

void IKPlayer::nextTimestep(int time)
{
	std::vector<IKConstraint> tmp;
	for(int i = 0; i < constraint.size(); i++){
		if(IKEnable && enable[i]) tmp.push_back(constraint[i]);
	}
	skel->inverseKinematics(tmp);
}

void IKPlayer::pushConstraint(IKConstraint condition){
	constraint.push_back(condition);
}

void IKPlayer::popConstraint(){
	constraint.pop_back();
}

GraphPlayer::GraphPlayer(SkeletonPtr skel, MotionGraphPtr graph):
	skel(skel), graph(graph){
	status = MotionStatusGraphPtr(new MotionStatusGraph(graph, 0, 0, 0, 0, 0, 0));
}

void GraphPlayer::display(){
	GUI::DrawGround(skel->location[0], skel->location[2], 0);
}

void GraphPlayer::nextTimestep(int time){
	status = status->step(status);
	SkeletonPosePtr pose = status->getPosition();
	
	skel->location = pose->location;
	for(int i = 0; i < skel->nodeList.size(); i++)
		skel->nodeList[i]->jointTransform = pose->joint[i];
}

void GraphPlayer::keyboard(unsigned char key, int x, int y){
	if(key != 'x' && key != 'z') graph->key = key;
}

void GraphPlayer::keyboardUp(unsigned char key, int x, int y){
}
