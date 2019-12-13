#include<Eigen/Dense>
#include <GL/glew.h>
#include <GL/glu.h>
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include<GL/freeglut.h>
#include<string>
#include<memory>
#include <tinyxml.h>
#include "Skeleton/Skeleton.h"

double scale = 10.0;

// SkeletonNode

const double PI = acos(-1);

SkeletonNode::SkeletonNode(Eigen::Affine3d parentTransform):
	parentTransform(parentTransform),
	jointTransform(1, 0, 0, 0),
	parent(NULL){
}

static void glRotated(Eigen::Quaterniond Q){
	Eigen::AngleAxisd tmp(Q);
	glRotated(tmp.angle() * 180 / PI, tmp.axis()[0], tmp.axis()[1], tmp.axis()[2]);
}

void SkeletonNode::display(){
	for(ShapeInfoPtr shape : shapeList) shape->display();
	/*
	for(SkeletonNodePtr child : childNodeList){
		Eigen::Vector3d p = child->parentTransform.translation();
		glBegin(GL_LINES);
		glColor3f(1.0, 1.0, 1.0);
		glVertex3f(0, 0, 0);
		glVertex3f(p[0], p[1], p[2]);
		glEnd();
	}
	*/
}

void SkeletonNode::setTransform(Eigen::Affine3d m){
	for(ShapeInfoPtr shape : shapeList) shape->setTransform(m);
}

// Skeleton

static Eigen::Vector3d stringToVector3d(std::string data){
	double a, b, c;
	sscanf(data.c_str(), "%lf %lf %lf", &a, &b, &c);
	return Eigen::Vector3d(a, b, c);
}

static double stringToDouble(std::string data){
	double a;
	sscanf(data.c_str(), "%lf", &a);
	return a;
}

static Eigen::AngleAxisd stringToAngleAxisd(std::string data){
	double a, b, c, d;
	sscanf(data.c_str(), "%lf %lf %lf %lf", &a, &b, &c, &d);
	return Eigen::AngleAxisd(a * PI / 180, Eigen::Vector3d(b, c, d));
}

static Eigen::Affine3d getAffineTransform(TiXmlElement *body)
{
	Eigen::Affine3d affine = Eigen::Affine3d::Identity();
	if(auto it = body->Attribute("rotate"))
		affine.rotate(stringToAngleAxisd(it));
	if(auto it = body->Attribute("translate"))
		affine.translate(stringToVector3d(it) * scale);
	
	return affine;
}

static SkeletonNodePtr helper(TiXmlElement *body, Skeleton* skel)
{
	Eigen::Affine3d parentTransform = getAffineTransform(body);
	SkeletonNodePtr current = SkeletonNodePtr(new SkeletonNode(parentTransform));

	if((std::string)body->Attribute("name") != "Site"){
		skel->nodeName[body->Attribute("name")] = current;
		current->idx = skel->nodeList.size();
		skel->nodeList.push_back(current);
	}

	for(TiXmlElement *child = body->FirstChildElement(); child != NULL; child = child->NextSiblingElement()){
		if(child->ValueStr() == "Cuboid"){
			Eigen::Vector3d info = stringToVector3d(child->Attribute("info")) * scale;
			Eigen::Affine3d affine = getAffineTransform(child);
			double m_mass;
			if(auto it = child->Attribute("mass")) m_mass = stringToDouble(it);
			else{
				m_mass = info[0] * info[1] * info[2] * 3000 / scale / scale / scale;
			}
			current->shapeList.push_back(ShapeInfoPtr((ShapeInfo*)new Cuboid(affine, info, m_mass)));
		}
		/*
		else if(child->ValueStr() == "Sphere"){
			double radius = stringToDouble(child->Attribute("radius"));
			Eigen::Affine3d affine = getAffineTransform(child);
			current->shapeList.push_back(ShapeInfoPtr((ShapeInfo*)new Sphere(affine, radius)));
		}
		else if(child->ValueStr() == "Cylinder"){
			double radius = stringToDouble(child->Attribute("radius"));
			double height = stringToDouble(child->Attribute("height"));
			Eigen::Affine3d affine = getAffineTransform(child);
			current->shapeList.push_back(ShapeInfoPtr((ShapeInfo*)new Cylinder(affine, radius, height)));
		}*/
		else if(child->ValueStr() == "Joint"){
			current->childNodeList.push_back(helper(child, skel));
			current->childNodeList.back()->parent = current;
		}
	}

	return current;
}

Skeleton::Skeleton(TiXmlDocument doc) : location(Eigen::Vector3d::Zero()){
	TiXmlElement *skeldoc = doc.FirstChildElement("Skeleton");
	name = skeldoc->Attribute("name");
	root = helper(skeldoc->FirstChildElement(), this);
	nodeLocation.resize(nodeList.size());
}

void Skeleton::display(){
	forwardKinematics();

	glColor3f(0.0, 0.0, 1.0);
	for(int i = 0; i < nodeList.size(); i++){
		glPushMatrix();
		glMultMatrixd(nodeLocation[i].data());
		nodeList[i]->display();
		glPopMatrix();
	}
	/*
	for(int i = 0; i < 3; i++){
		Eigen::Vector3d t = Eigen::Vector3d(i == 0, i == 1, i == 2);
		Eigen::Vector3d end = nodeList[0]->jointTransform._transformVector(t) + location;
		glColor3dv(t.data());
		glBegin(GL_LINES);
		glVertex3dv(location.data());
		glVertex3dv(end.data());
		glEnd();
	} // */
}

void Skeleton::forwardKinematics()
{
	for(int i = 0; i < nodeList.size(); i++){
		auto cur = nodeList[i];
		nodeLocation[i] = Eigen::Affine3d::Identity();
		if(!i) nodeLocation[i].translate(location);
		nodeLocation[i].rotate(cur->jointTransform);
		nodeLocation[i] = cur->parentTransform * nodeLocation[i];
		if(i) nodeLocation[i] = nodeLocation[cur->parent->idx] * nodeLocation[i];
	}
}

void Skeleton::setTransform(){
	this->forwardKinematics();
	for(int i = 0; i < nodeList.size(); i++){
		nodeList[i]->setTransform(nodeLocation[i]);
	}
}

static Eigen::Matrix3d crossMatrixL(Eigen::Vector3d v)
{
	Eigen::Matrix3d t;
	t <<    0, v[2],-v[1],
	    -v[2],    0, v[0],
	     v[1],-v[0],    0;
	return t;
}

static Eigen::Matrix3d crossMatrixR(Eigen::Vector3d v)
{
	Eigen::Matrix3d t;
	t <<    0,-v[2], v[1],
	     v[2],    0,-v[0],
	    -v[1], v[0],    0;
	return t;
}

Eigen::MatrixXd Skeleton::getJacobian(const std::vector<IKConstraint> &constraint){
	forwardKinematics();

	int n = constraint.size(), m = nodeList.size() + 1;
	Eigen::MatrixXd jacobian;
	jacobian = Eigen::MatrixXd::Zero(6*n, 3*m);

	for(int i = 0; i < n; i++){
		jacobian.block(i*6, 0, 3, 3) = Eigen::Matrix3d::Identity();
		SkeletonNodePtr node = constraint[i].node, cur = node;
		while(cur){
			Eigen::Vector3d dist = nodeLocation[cur->idx].translation() - nodeLocation[node->idx].translation();
			jacobian.block(i*6, 3*(cur->idx+1), 3, 3) = crossMatrixR(dist);
			jacobian.block(i*6+3, 3*(cur->idx+1), 3, 3) = Eigen::Matrix3d::Identity();
			cur = cur->parent;
		}
	}
	return jacobian;
}

std::ostream& operator<<(std::ostream& os, const SkeletonPose& dt)
{
	std::cout << "translate : " << dt.location << "\nrotation : ";
	for(auto c : dt.joint) std::cout << c.w() << " " << c.vec() << "\n";
    return os;
}
