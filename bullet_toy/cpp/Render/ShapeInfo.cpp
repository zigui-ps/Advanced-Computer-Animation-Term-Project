#include <GL/glew.h>
#include<GL/freeglut.h>
#include <GL/glu.h>
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include<assert.h>
#include "Render/ShapeInfo.h"

const double PI = acos(-1);

// ShapeInfo

ShapeInfo::ShapeInfo(Eigen::Affine3d location, double m_mass) : CollisionObject(m_mass), location(location){}

// Cuboid

Cuboid::Cuboid(Eigen::Affine3d location, Eigen::Vector3d info, double m_mass) : 
		ShapeInfo(location * Eigen::Affine3d(Eigen::Translation3d(-info[0]/2, -info[1]/2, -info[2]/2)), m_mass), info(info){
    btBoxShape *box = new btBoxShape(btVector3(info[0]/2, info[1]/2, info[2]/2));
    btTransform m_trans; m_trans.setIdentity();
    m_obj = create_rigid_body(m_mass, m_trans, box, true);
}

void Cuboid::setTransform(Eigen::Affine3d m){
	Eigen::Affine3d center = Eigen::Affine3d(Eigen::Translation3d(info[0]/2, info[1]/2, info[2]/2));
	CollisionObject::setTransform(m * location * center); 
}

//    6----7
//   /|   /|
//  4-+--5 |
//  | 2--+-3
//  |/   |/
//  0----1

void Cuboid::display()
{
	static int DB[6][4] = {
		{4, 5, 7, 6},
		{0, 4, 6, 2},
		{0, 2, 3, 1},
		{1, 3, 7, 5},
		{0, 1, 5, 4},
		{2, 6, 7, 3}};
	static Eigen::Vector3d n[] = {
		Eigen::Vector3d::UnitZ(), 
		-Eigen::Vector3d::UnitX(),
		-Eigen::Vector3d::UnitZ(),
		Eigen::Vector3d::UnitX(),
		-Eigen::Vector3d::UnitY(),
		Eigen::Vector3d::UnitY()};
	
	glPushMatrix();
	glMultMatrixd(location.data());
	for(int i = 0; i < 6; i++){
		glBegin(GL_POLYGON);
		glNormal3d(n[i][0], n[i][1], n[i][2]);
		for(int j = 0; j < 4; j++){
			int x = DB[i][j];
			Eigen::Vector3d t = Eigen::Vector3d(!!(x&1), !!(x&2), !!(x&4)).cwiseProduct(info);
			glVertex3d(t[0], t[1], t[2]);
		}
		glEnd();
	}
	glPopMatrix();
}

// Sphere
/*
Sphere::Sphere(Eigen::Affine3d location, double radius):
	ShapeInfo(location), radius(radius){
}

void Sphere::display()
{
	glPushMatrix();
	{
		glMultMatrixd(location.data());
		glutSolidSphere(radius, 20, 20);
	}
	glPopMatrix();
}

// Cylinder

Cylinder::Cylinder(Eigen::Affine3d location, double radius, double height):
	ShapeInfo(location), radius(radius), height(height){
}

void Cylinder::display()
{
	glPushMatrix();
	{
		glMultMatrixd(location.data());
		glBegin(GL_QUAD_STRIP);
		for(int i = 0 ; i <= 20; i++){
			glNormal3d(cos(PI/10 * i), sin(PI/10 * i), 0);
			glVertex3d(cos(PI/10 * i) * radius, sin(PI/10 * i) * radius, height);
			glVertex3d(cos(PI/10 * i) * radius, sin(PI/10 * i) * radius, 0);
		}
		glEnd();

		glBegin(GL_POLYGON);
		glNormal3d(0, 0, -1);
		for(int i = 0 ; i < 20; i++)
			glVertex3d(cos(PI/10 * i) * radius, sin(PI/10 * i) * radius, 0);
		glEnd();
		
		glBegin(GL_POLYGON);
		glNormal3d(0, 0, 1);
		for(int i = 0 ; i < 20; i++)
			glVertex3d(cos(PI/10 * i) * radius, sin(PI/10 * i) * radius, height);
		glEnd();
	}
	glPopMatrix();
}
*/
// Ground

GroundShape::GroundShape(int n, int m, double x, double z):
	ShapeInfo(Eigen::Affine3d::Identity(), 0), n(n), m(m), x(x), z(z){
    // btBoxShape *box = new btBoxShape(btVector3(n*x*2, 10, m*z*2));
    // btTransform m_trans; m_trans.setOrigin(btVector3(0, -5, 0));
    // m_obj = create_rigid_body(0.0, m_trans, box, false);
	m_obj = create_ground(n*x, 5, m*z);
}

void GroundShape::display()
{
	glColor3d(0.5, 0.5, 0.5);
	glBegin(GL_LINES);
	for(int i = -n; i <= n; i++){
		if(i == 0){
			glVertex3d(i*x, 0, -m*z);
			glVertex3d(i*x, 0, -z);
			glVertex3d(i*x, 0, z);
			glVertex3d(i*x, 0, m*z);
		}else{
			glVertex3d(i*x, 0, -m*z);
			glVertex3d(i*x, 0, m*z);
		}
	}
	for(int j = -m; j <= m; j++){
		if(j == 0){
			glVertex3d(-n*x, 0, j*z);
			glVertex3d(-x, 0, j*z);
			glVertex3d(x, 0, j*z);
			glVertex3d(n*x, 0, j*z);
		}else{
			glVertex3d(-n*x, 0, j*z);
			glVertex3d(n*x, 0, j*z);
		}
	}
	glEnd();
	glBegin(GL_POLYGON);
	glNormal3d(0, 1, 0);
	glVertex3d(-n*x, 0, -m*z);
	glVertex3d(-n*x, 0, m*z);
	glVertex3d(n*x, 0, m*z);
	glVertex3d(n*x, 0, -m*z);
	glEnd();
}

void GroundShape::setTransform(Eigen::Affine3d m){ assert(false); }
