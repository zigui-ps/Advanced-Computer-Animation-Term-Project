#ifndef __BULLET_HELPER_H__
#define __BULLET_HELPER_H__

#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"

extern btSoftRigidDynamicsWorld* g_dynamicsWorld;

void init_bullet_world();
btRigidBody* create_rigid_body(float mass, const btTransform& trans, btCollisionShape* shape, bool isKinematics = false);

btRigidBody* create_ground(float x, float y, float z);
btRigidBody* create_jump_building();

btSoftBody* create_rope(btVector3 from,btVector3 to);
btSoftBody* create_cloak();

void draw_soft_body(btSoftBody* psb);
void draw_rope(btSoftBody* psb, double R, int c1, int c2);

#endif //__BULLET_HELPER_H__
