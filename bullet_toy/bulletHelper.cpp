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

		body->setUserIndex(-1);
		if(isKinematics){
			body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
			body->setActivationState( DISABLE_DEACTIVATION );
		}
		g_dynamicsWorld->addRigidBody(body);
		return body;
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
		drawTriangle(x[0], x[2], x[1], y[0], y[2], y[1], btVector3(0.0, 0.0, 1.0));
	}
	delete normal;
}
