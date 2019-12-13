#ifndef SHAPEINFO
#define SHAPEINFO

#include <GL/glew.h>
#include<GL/freeglut.h>
#include <GL/glu.h>
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include<Eigen/Geometry>
#include<memory>
#include "Render/Drawable.h"
#include "CollisionObject.h"
#include "bulletHelper.h"

class ShapeInfo : public Drawable, public CollisionObject{
public:
	ShapeInfo(Eigen::Affine3d location, double m_mass);
	Eigen::Affine3d location;
	virtual void display() = 0;
    virtual void setTransform(Eigen::Affine3d m) = 0;
};

using ShapeInfoPtr = std::shared_ptr<ShapeInfo>;

class Cuboid : public ShapeInfo{
public:
	Cuboid(Eigen::Affine3d location, Eigen::Vector3d info, double m_mass);
	Eigen::Vector3d info;
	
	void display();
    virtual void setTransform(Eigen::Affine3d m);
};
/*
class Sphere : public ShapeInfo{
public:
	double radius;
	Sphere(Eigen::Affine3d location, double radius);
	
	void display();
};

class Cylinder : public ShapeInfo{
public:
	Cylinder(Eigen::Affine3d location, double radius, double height);
	
	void display();
	double radius, height;
};
*/
class GroundShape : public ShapeInfo{
public:
	GroundShape(int n, int m, double x, double z);
	
    virtual void setTransform(Eigen::Affine3d m);
	void display();
	int n, m, x, z;
};

#endif
