#ifndef __BULLET_HELPER_H__
#define __BULLET_HELPER_H__

#include "btBulletDynamicsCommon.h"
#include <GL/glew.h>
#include <GL/glu.h>
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include <GL/freeglut.h>

extern btDiscreteDynamicsWorld* g_dynamicsWorld;


void init_bullet_world();
btRigidBody* create_rigid_body(float mass, const btTransform& trans, btCollisionShape* shape, bool isKinematics = false);
void draw_box(const btVector3& half_size);
void draw_axes();
#endif //__BULLET_HELPER_H__