#include "bulletHelper.h"
#include "openglHelper.h"
#include <map>
#include <vector>

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

		//body->setUserIndex(-1);
		if(isKinematics){
			body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
			body->setActivationState( DISABLE_DEACTIVATION );
		}
		g_dynamicsWorld->addRigidBody(body);
		return body;
}

btRigidBody* create_ground(float x, float y, float z){
	///create a ground
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(x), btScalar(y), btScalar(z)));

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -y, 0));
	groundTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0.));
	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	btScalar mass(0.);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		groundShape->calculateLocalInertia(mass, localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	body->setFriction(1);

	g_dynamicsWorld->addRigidBody(body);
	return body;
}

btRigidBody* create_jump_building(){
	///create a ground
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(10), btScalar(40), btScalar(20)));

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, 40, -40));
	groundTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI * 0.));
	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	btScalar mass(0.);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		groundShape->calculateLocalInertia(mass, localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	body->setFriction(1);

	g_dynamicsWorld->addRigidBody(body);
	return body;
}

btSoftBody* create_rope(btVector3 from,btVector3 to){
	

	btSoftBody* rope = btSoftBodyHelpers::CreateRope(g_dynamicsWorld->getWorldInfo(),from, to, 50, 1);
	rope->getCollisionShape()->setMargin(0.1);
	rope->m_cfg.kKHR = 1; // collision hardness with kinematic objects
	rope->m_cfg.kCHR = 1; // collision hardness with rigid body
	rope->m_cfg.kDF = 2;
	rope->m_cfg.drag =0.002;
	//rope->m_cfg.collisions = btSoftBody::fCollision::SDF_RS | btSoftBody::fCollision::CL_RS; // | btSoftBody::fCollision::VF_SS ;
	rope->m_cfg.collisions = btSoftBody::fCollision::RVSmask;

	rope->m_materials[0]->m_kLST = 1;

	rope->setTotalMass(5.f);
	g_dynamicsWorld->addSoftBody(rope);

	btDeformableMassSpringForce* mass_spring = new btDeformableMassSpringForce(100, 1, true);
	g_dynamicsWorld->addForce(rope, mass_spring);

	btDeformableGravityForce* gravity_force =  new btDeformableGravityForce(btVector3(0,-10, 0));
	g_dynamicsWorld->addForce(rope, gravity_force);

	return rope;
}

btSoftBody* create_cloak(){
	
	
	const btScalar s = 4;
	const btScalar h = 94;
	const int r = 10;
	btSoftBody* psb = btSoftBodyHelpers::CreatePatch(g_dynamicsWorld->getWorldInfo(), btVector3(-s/2, h, -s-56),
			btVector3(+s/2, h, -s-56),
			btVector3(-s/2, h, +s-56),
			btVector3(+s/2, h, +s-56), r/2, r, 0, true);
	psb->getCollisionShape()->setMargin(0.1);
	psb->generateBendingConstraints(2);
	psb->setTotalMass((btScalar)1.);
	psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
	psb->m_cfg.kCHR = 1; // collision hardness with rigid body
	psb->m_cfg.kDF = 2;
	psb->m_cfg.drag =0.01;
	psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RS | btSoftBody::fCollision::CL_RS; // | btSoftBody::fCollision::VF_SS ;
	g_dynamicsWorld->addSoftBody(psb);

	btDeformableMassSpringForce* mass_spring = new btDeformableMassSpringForce(100, 1, true);
	g_dynamicsWorld->addForce(psb, mass_spring);

	btVector3 gravity = btVector3(0, -10, 0);
	btDeformableGravityForce* gravity_force =  new btDeformableGravityForce(gravity);
	g_dynamicsWorld->addForce(psb, gravity_force);

	return psb;
}

void get_spline(btVector3 p0, btVector3 p1, btVector3 p2, btVector3 p3, std::vector<btVector3> &points, int cnt){
	for(int i = 0; i <= cnt; i++){
		double t = i / (double)cnt;
		points.push_back((1-t)*(1-t)*(1-t)*p0 + 3*t*(1-t)*(1-t)*p1 + 3*t*t*(1-t)*p2 + t*t*t*p3);
	}
}

void draw_rope(btSoftBody* psb, double R, int c1, int c2){
	const double PI = acos(-1);
	int n = psb->m_nodes.size();
	std::vector<btVector3> points;
	for(int i = 0; i+1 < psb->m_nodes.size(); i++){
		btVector3 p0 = psb->m_nodes[i].m_x, p1 = psb->m_nodes[i+1].m_x;
		btVector3 t0, t1;
		t0 = i == 0 ? (p1 - p0) : (p1 - psb->m_nodes[i-1].m_x) * 0.5;
		t1 = i+2 == psb->m_nodes.size() ? (p1 - p0) : (psb->m_nodes[i+2].m_x - p0) * 0.5;
		get_spline(p0, p0 + t0/3, p1 - t1/3, p1, points, c1);
		points.pop_back();
	}
	btVector3 up;
	up = (points[0] - points[1]).cross(btVector3(0, 0, 1));
	if(up.norm() < 1e-10) up = (points[0] - points[1]).cross(btVector3(0, 1, 0));
	up = up.normalized();

	for(int i = 0; i+1 < points.size(); i++){
		btVector3 n, m, p0 = points[i], p1 = points[i+1], up2;
		n = i == 0? points[i+1] - points[i] : points[i+1] - points[i-1];
		m = i+2 == points.size()? points[i+1] - points[i] : points[i+2] - points[i];
		n = n.normalized(); m = m.normalized();
		up2 = (up - m.dot(up2) * m).normalized();

		btVector3 l = up.cross(n), l2 = up2.cross(m);

		glBegin(GL_QUAD_STRIP);
		for(int i = 0; i <= c2; i++){
			double rad = PI * 2 / c2 * i;
			btVector3 d0 = l * cos(rad) + up * sin(rad), q0 = p0 + d0 * R;
			btVector3 d1 = l2 * cos(rad) + up2 * sin(rad), q1 = p1 + d1 * R;
			glNormal3d(d0.x(), d0.y(), d0.z());
			glVertex3d(q0.x(), q0.y(), q0.z());
			glNormal3d(d1.x(), d1.y(), d1.z());
			glVertex3d(q1.x(), q1.y(), q1.z());
		}
		glEnd();
		up = up2;
	}
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
		drawTriangle(x[0], x[2], x[1], y[0], y[2], y[1], btVector3(1.0, 0.0, 0.0));
	}
	delete normal;
}
