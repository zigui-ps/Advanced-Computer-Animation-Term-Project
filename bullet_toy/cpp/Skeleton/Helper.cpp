#include "Skeleton/Helper.h"

// Helper

static const double PI = acos(-1);

void rotateY(SkeletonPosePtr pose, double dY){
	pose->joint[0] = Eigen::AngleAxisd(dY, Eigen::Vector3d::UnitY()) * pose->joint[0];
	Eigen::Vector3d x = pose->location;
	pose->location = Eigen::Vector3d(x[0] * cos(dY) + x[2] * sin(dY), x[1], x[0] * -sin(dY) + x[2] * cos(dY));
}

double getYrot(SkeletonPosePtr from, SkeletonPosePtr to){
	return getYrot(from->joint[0], to->joint[0]);
}

double getYrot(Eigen::Quaterniond from, Eigen::Quaterniond to){
	// http://mrl.snu.ac.kr/courses/CourseAnimation/ComputerAnimationLectureNote/12_MotionEditing.pdf 22 page
	// https://github.com/snumrl/PmQm/blob/master/src/qmGeodesic.cpp line 13~37

	Eigen::Quaterniond qs = to, q0 = from;
	Eigen::Vector3d t, u = Eigen::Vector3d::UnitY();
	double a = qs.w() * q0.w() + qs.vec().dot(q0.vec());
	//       \ | /
	//        \|/
	//    this v sign is wrong in pdf
	double b = -qs.w() * u.dot(q0.vec()) + q0.w() * u.dot(qs.vec()) + qs.vec().dot(u.cross(q0.vec()));
	double alpha = atan2(a, b);
	t = u * sin(-alpha + PI/2); Eigen::Quaterniond Ga = Eigen::Quaterniond(cos(-alpha + PI/2), t[0], t[1], t[2]) * q0;
	t = u * sin(-alpha - PI/2); Eigen::Quaterniond Gs = Eigen::Quaterniond(cos(-alpha - PI/2), t[0], t[1], t[2]) * q0;
	if(qs.dot(Ga) > qs.dot(Gs)) return (-alpha + PI/2) * 2;
	else return (-alpha - PI/2) * 2;
}

SkeletonPosePtr getPoseDifference(SkeletonPosePtr to, SkeletonPosePtr from){
	SkeletonPosePtr cur = SkeletonPosePtr(new SkeletonPose());
	cur->location = to->location - from->location;
	for(int i = 0; i < to->joint.size(); i++)
		cur->joint.push_back(to->joint[i] * from->joint[i].inverse());
	return cur;
}
