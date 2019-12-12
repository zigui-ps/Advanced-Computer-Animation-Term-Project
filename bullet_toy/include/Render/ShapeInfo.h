#ifndef SHAPEINFO
#define SHAPEINFO

#include <GL/glew.h>
#include<GL/freeglut.h>
#include <GL/glu.h>
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include<Eigen/Geometry>
#include<memory>
#include "Render/Drawable.h"

class ShapeInfo : public Drawable{
public:
	ShapeInfo(Eigen::Affine3d location);
	Eigen::Affine3d location;
	virtual void display() = 0;
};

using ShapeInfoPtr = std::shared_ptr<ShapeInfo>;

class Cuboid : public ShapeInfo{
public:
	Cuboid(Eigen::Affine3d location, Eigen::Vector3d info);
	Eigen::Vector3d info;
	
	void display();
};

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

class GroundShape : public ShapeInfo{
public:
	GroundShape(int n, int m, double x, double z);
	
	void display();
	int n, m, x, z;
};

#endif
