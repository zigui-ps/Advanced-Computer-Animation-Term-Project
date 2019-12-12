#include "bulletHelper.h"
#include "openglHelper.h"
#include <map>

btDeformableMultiBodyDynamicsWorld* g_dynamicsWorld;

void init_bullet_world(){
    auto m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	auto m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	auto m_broadphase = new btDbvtBroadphase();
	btDeformableBodySolver* deformableBodySolver = new btDeformableBodySolver();
	btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
	sol->setDeformableSolver(deformableBodySolver);
	auto m_solver = sol;

	g_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, deformableBodySolver);

    g_dynamicsWorld->setGravity(btVector3(0,-10,0));

    printf("Init bullet world\n");
}

btRigidBody* create_rigid_body(float mass, const btTransform& trans, btCollisionShape* shape, bool isKinematics/* = false*/){
	//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			shape->calculateLocalInertia(mass, localInertia);
		
		btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
		body->setWorldTransform(trans);

		body->setUserIndex(-1);
		if(isKinematics){
			body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
			body->setActivationState( DISABLE_DEACTIVATION );
		}
		g_dynamicsWorld->addRigidBody(body);
		return body;
}

void draw_soft_body(btSoftBody* psb){
	int n = psb->m_nodes.size();

	btVector3* normal = new btVector3[n];
	std::map<btSoftBody::Node*, int> db;

	for(int i = 0; i < n; i++){
		normal[i].setZero();
		db[&psb->m_nodes[i]] = i;
	}
	for(int i = 0; i < psb->m_faces.size(); ++i){
		const btSoftBody::Face& f=psb->m_faces[i];	
		const btVector3 x[]={f.m_n[0]->m_x,f.m_n[1]->m_x,f.m_n[2]->m_x};
		btVector3 n = (x[2] - x[0]).cross(x[1] - x[0]).safeNormalize();
		for(int j = 0; j < 3; j++){
			int idx = db[f.m_n[j]];
			normal[idx] += n;
		}
	}
	for(int i = 0; i < n; i++) normal[i] = normal[i].safeNormalize();
	for(int i = 0; i < psb->m_faces.size(); ++i){
		const btSoftBody::Face& f=psb->m_faces[i];	
		btVector3 x[3], y[3];
		for(int j = 0; j < 3; j++){
			x[j] = f.m_n[j]->m_x;
			y[j] = normal[db[f.m_n[j]]];
		}
		drawTriangle(x[0], x[2], x[1], y[0], y[2], y[1], btVector3(0.0, 0.0, 1.0));
	}
	delete normal;
}
