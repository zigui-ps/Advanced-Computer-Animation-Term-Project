#ifndef SKELETON
#define SKELETON

#include<Eigen/Dense>
#include<memory>
#include<map>
#include<vector>
#include<tinyxml.h>
#include<string>
#include<iostream>
#include "Render/Drawable.h"
#include "Render/ShapeInfo.h"
#include "CollisionObject.h"

class SkeletonNode;
class Skeleton;
using SkeletonNodePtr = std::shared_ptr<SkeletonNode>;
using SkeletonPtr = std::shared_ptr<Skeleton>;
using CollisionObjectPtr = std::shared_ptr<CollisionObject>;

class SkeletonNode : public Drawable{
public:
	SkeletonNode(Eigen::Affine3d parentTransform);
	Eigen::Affine3d parentTransform;
	Eigen::Quaterniond jointTransform;
	std::vector<SkeletonNodePtr> childNodeList;
	std::vector<ShapeInfoPtr> shapeList;
	SkeletonNodePtr parent;
	int idx;

	void setTransform(Eigen::Affine3d m);
	void display();
};

struct IKConstraint{
	IKConstraint(SkeletonNodePtr node, Eigen::Affine3d target);
	SkeletonNodePtr node;
	Eigen::Affine3d target;
};

class SkeletonPose{
	public:
		SkeletonPose() : location(Eigen::Vector3d::Zero()){}
		Eigen::Vector3d location;
		std::vector<Eigen::Quaterniond> joint;
		friend std::ostream& operator<<(std::ostream& os, const SkeletonPose& dt);
};

class Skeleton : public Drawable{
public:
	Skeleton(TiXmlDocument doc);

	std::string name;
	std::map<std::string, SkeletonNodePtr> nodeName;
	std::vector<SkeletonNodePtr> nodeList;
	std::vector<Eigen::Affine3d> nodeLocation;
	SkeletonNodePtr root;
	Eigen::Vector3d location;

	void forwardKinematics();
	void setTransform();
	void inverseKinematics(const std::vector<IKConstraint> &constraint);
	Eigen::MatrixXd getJacobian(const std::vector<IKConstraint> &constraint);
	void turnOffKinematics();
	CollisionObjectPtr getCollisionObject(std::string name);

	void display();
};

using SkeletonPosePtr = std::shared_ptr<SkeletonPose>;

#endif
