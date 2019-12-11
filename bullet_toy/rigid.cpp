#include <GL/glew.h>
#include <GL/glu.h>
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include <GL/freeglut.h>
#include <stdio.h> //printf debugging
#include <map>

#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"

#include "LinearMath/btVector3.h"

// #include "bvh.h"

class DebugDrawer : public btIDebugDraw
{
	int m_debugMode;

public:
	DebugDrawer();
	virtual void	drawLine(const btVector3& from,const btVector3& to, const btVector3& color);
	virtual void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color);
	virtual void	draw3dText(const btVector3& location,const char* textString);
	virtual void	reportErrorWarning(const char* warningString);
	virtual void	setDebugMode(int debugMode);
	virtual int		getDebugMode() const { return m_debugMode;}

};

DebugDrawer::DebugDrawer(){

}

void DebugDrawer::drawLine(const btVector3& from,const btVector3& to,const btVector3& color){
	glBegin(GL_LINES);
	glColor3f(0,0,0);
	glVertex3d(from.getX(), from.getY(),from.getZ());
	glVertex3d(to.getX(), to.getY(), to.getZ());
	glEnd();
}

void DebugDrawer::drawContactPoint(const btVector3& pointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)
{
	{
		btVector3 to=pointOnB+normalOnB*1;//distance;
		const btVector3&from = pointOnB;
		glColor4f(color.getX(), color.getY(), color.getZ(),1.f);
		glBegin(GL_LINES);
		glVertex3d(from.getX(), from.getY(), from.getZ());
		glVertex3d(to.getX(), to.getY(), to.getZ());
		glEnd();
	}
}

void DebugDrawer::draw3dText(const btVector3& location,const char* textString)
{
	glRasterPos3f(location.x(),  location.y(),  location.z());
}



void DebugDrawer::reportErrorWarning(const char* warningString)
{
	printf("%s\n",warningString);
}

void  DebugDrawer::setDebugMode(int debugMode)
{
	m_debugMode = debugMode;
}

DebugDrawer* debugDrawer = new DebugDrawer();
GLfloat light_diffuse[] = {1.0, 0.0, 0.0, 1.0};  /* Red diffuse light. */
GLfloat light_position[] = {1.0, 1.0, 1.0, 0.0};  /* Infinite light location. */

btDeformableMultiBodyDynamicsWorld* g_dynamicsWorld;
//btDeformableMultiBodyDynamicsWorld* m_dynamicsWorld;
btRigidBody *box1, *box2;

void draw_box(const btVector3& point_enter, const btVector3& half_size){
	float width = half_size.getX();
	float height = half_size.getY();
	float depth = half_size.getZ();

	glBegin(GL_TRIANGLES);
	glColor3f(0,0,0);

	// Top
	glNormal3f(0,1,0);
	glVertex3f(width, height, depth);
	glVertex3f(-width, height, -depth);
	glVertex3f(-width, height, depth);

	glVertex3f(width, height, depth);
	glVertex3f(width, height, -depth);
	glVertex3f(-width, height, -depth);

	glNormal3f(0,0,-1);
	glVertex3f(-width, -height, depth);
	glVertex3f(-width, height, depth);
	glVertex3f(width, -height, depth);
	
	glVertex3f(width, -height, depth);
	glVertex3f(width, height, depth);
	glVertex3f(-width, height, depth);

	glNormal3f(1,0,0);
	glVertex3f(width, -height, depth);
	glVertex3f(width, height, -depth);
	glVertex3f(width, -height, -depth);

	glVertex3f(width, -height, depth);
	glVertex3f(width, height, -depth);
	glVertex3f(width, height, depth);

	// Bottom
	glNormal3f(0,-1,0);
	glVertex3f(width, -height, depth);
	glVertex3f(-width, -height, -depth);
	glVertex3f(-width, -height, depth);

	glVertex3f(width, -height, depth);
	glVertex3f(width, -height, -depth);
	glVertex3f(-width, -height, -depth);

	glNormal3f(0,0,1);	
	glVertex3f(-width, -height, -depth);
	glVertex3f(-width, height, -depth);
	glVertex3f(width, -height, -depth);
	
	glVertex3f(width, -height, -depth);
	glVertex3f(width, height, -depth);
	glVertex3f(-width, height, -depth);

	glNormal3f(-1,0,0);
	glVertex3f(width, -height, depth);
	glVertex3f(width, height, -depth);
	glVertex3f(width, -height, -depth);

	glVertex3f(width, -height, depth);
	glVertex3f(width, height, -depth);
	glVertex3f(width, -height, -depth);

	glNormal3f(-1,0,0);
	glVertex3f(-width, -height, depth);
	glVertex3f(-width, height, -depth);
	glVertex3f(-width, -height, -depth);

	glVertex3f(-width, -height, depth);
	glVertex3f(-width, height, -depth);
	glVertex3f(-width, height, depth);
	glEnd();
}

void drawTriangle(btVector3 p0, btVector3 p1, btVector3 p2, btVector3 c){
	glBegin(GL_TRIANGLES);
	glColor3f(c.x(), c.y(), c.z());
	btVector3 n = (p1-p0).cross(p2-p0).safeNormalize();
	glNormal3f(n.x(), n.y(), n.z());
	glVertex3f(p0.x(), p0.y(), p0.z());
	glVertex3f(p1.x(), p1.y(), p1.z());
	glVertex3f(p2.x(), p2.y(), p2.z());

	glEnd();
}

void drawTriangle(btVector3 p0, btVector3 p1, btVector3 p2, 
		btVector3 n0, btVector3 n1, btVector3 n2, btVector3 c){
	glBegin(GL_TRIANGLES);
	glColor3f(c.x(), c.y(), c.z());
	glNormal3f(n0.x(), n0.y(), n0.z());
	glVertex3f(p0.x(), p0.y(), p0.z());
	glNormal3f(n1.x(), n1.y(), n1.z());
	glVertex3f(p1.x(), p1.y(), p1.z());
	glNormal3f(n2.x(), n2.y(), n2.z());
	glVertex3f(p2.x(), p2.y(), p2.z());

	glEnd();
}

void Draw_SoftBody(btSoftBody* psb){
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

void display(void)
{
	glClearColor(0.9, 0.9, 0.9, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// g_dynamicsWorld->stepSimulation(1.f/ 60.f, 10);
	//Debug
	// g_dynamicsWorld->debugDrawWorld();
	for(int i=g_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--){
		btCollisionObject* obj = g_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if(!body) continue;
		btTransform trans;

		// For Test
		/*
		if(i == 2){
			btTransform t;
			t.setOrigin(btVector3(((float)rand()/RAND_MAX)*5, 0, 0));

			body->setWorldTransform(t);
			body->applyCentralForce(btVector3(((float)rand()/RAND_MAX)*5, 0, 0));
			body->activate();
		} // */

		trans = obj->getWorldTransform();

		float trans_x = float(trans.getOrigin().getX());
		float trans_y = float(trans.getOrigin().getY());
		float trans_z = float(trans.getOrigin().getZ());

		printf("world pos object %d = %f,%f,%f\n", i, trans_x, trans_y, trans_z);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		double m[16];
		trans.getOpenGLMatrix(m);
		glMultMatrixd(m);

		draw_box(trans.getOrigin(), btVector3(5,5,5));
		glPopMatrix();
	}
	for (int i = 0; i < g_dynamicsWorld->getSoftBodyArray().size(); i++)
	{
		btSoftBody* psb = (btSoftBody*)g_dynamicsWorld->getSoftBodyArray()[i];
		Draw_SoftBody(psb);
	}
	glutSwapBuffers();
}

void nextTimestep(int time){
	glutTimerFunc(1000.0 / 60.0, nextTimestep, 0);
	float internalTimeStep = 1. / 240.f, deltaTime = 1. / 60.f;
	
	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(((float)rand()/RAND_MAX) * 20, 0, 0));
//	if(box1) box1->setWorldTransform(t);
	
	g_dynamicsWorld->stepSimulation(deltaTime, 4, internalTimeStep);
	
	glutPostRedisplay();
}

void init_gl(int argc, char* argv[]){
    glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB|GLUT_DEPTH);
	glutInitWindowSize(1600, 900);
	glutCreateWindow("Rigid");

    glutDisplayFunc(display);
	glutTimerFunc(1000.0 / 60.0, nextTimestep, 0);

    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);

    glEnable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION);
	gluPerspective(40.0, 1.0, 1.0, 1e4);

	glMatrixMode(GL_MODELVIEW);
	gluLookAt(100.0, 100.0, 100.0,   /* eye is at () */
			0.0, 0.0, 0.0,      /* center is at (0,0,0) */
			0.0, 1.0, 0.);      /* up is in positive Y direction */

    printf("Init GL\n");
}

void init_bullet_world(){
	auto m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	auto m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	auto m_broadphase = new btDbvtBroadphase();
	btDeformableBodySolver* deformableBodySolver = new btDeformableBodySolver();
	btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
	sol->setDeformableSolver(deformableBodySolver);
	auto m_solver = sol;

	g_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, deformableBodySolver);
	/*
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    g_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	*/
    g_dynamicsWorld->setGravity(btVector3(0,-10,0));

    printf("Init bullet world\n");
}

btRigidBody* createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, bool isKinematics = false, const btVector4& color = btVector4(1, 0, 0, 1)){
	//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			shape->calculateLocalInertia(mass, localInertia);
		
		btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
		body->setWorldTransform(startTransform);

		body->setUserIndex(-1);
		if(isKinematics){
			body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
			body->setActivationState( DISABLE_DEACTIVATION );
		}
		g_dynamicsWorld->addRigidBody(body);
		return body;
}

int main(int argc, char* argv[]){
    init_gl(argc, argv);

    init_bullet_world();
	
	// Load BVH
//	Bvh bvh;
//	bvh.load("../vsctut/bvh/16_01_jump.bvh");

	debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);
	g_dynamicsWorld->setDebugDrawer(debugDrawer);
	// create a ground
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
	
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-150, 0));
	{
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}/*
	{
		btBoxShape *box = new btBoxShape(btVector3(btScalar(5.), btScalar(5.), btScalar(5.)));
		
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(5.f);
		
		startTransform.setOrigin(btVector3(0,40,0));
		box1 = createRigidBody(mass, startTransform, box, false);
	}
	{
		btBoxShape *box = new btBoxShape(btVector3(btScalar(5.), btScalar(5.), btScalar(5.)));
		
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(5.f);
		
		startTransform.setOrigin(btVector3(5,25,0));
		box2 = createRigidBody(mass, startTransform, box);
	}// */
	{
		const btScalar s = 4;
		const btScalar h = 6;
		const int r = 9;
		btSoftBody* psb = btSoftBodyHelpers::CreatePatch(g_dynamicsWorld->getWorldInfo(), btVector3(-s, h, -s),
				btVector3(+s, h, -s),
				btVector3(-s, h, +s),
				btVector3(+s, h, +s), r, r, 0, true);
		psb->getCollisionShape()->setMargin(0.1);
		psb->generateBendingConstraints(2);
		psb->setTotalMass((btScalar)1.);
		psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
		psb->m_cfg.kCHR = 1; // collision hardness with rigid body
		psb->m_cfg.kDF = 2;
		psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RS | btSoftBody::fCollision::CL_RS; // | btSoftBody::fCollision::VF_SS ;
		g_dynamicsWorld->addSoftBody(psb);

		btDeformableMassSpringForce* mass_spring = new btDeformableMassSpringForce(100, 1, true);
		g_dynamicsWorld->addForce(psb, mass_spring);
	
		btVector3 gravity = btVector3(0, -10, 0);
		btDeformableGravityForce* gravity_force =  new btDeformableGravityForce(gravity);
		g_dynamicsWorld->addForce(psb, gravity_force);

		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(btVector3(0, 11, -(s + 5)));

		btCollisionShape *shape = new btBoxShape(btVector3(5, 5, 5));
		double mass = 10;

		btVector3 localInertia(0, 0, 0);
		shape->calculateLocalInertia(mass, localInertia);

		btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
		body->setWorldTransform(startTransform);

		body->setUserIndex(-1);
		body->setLinearVelocity(btVector3(0, 20, 0));

		psb->appendDeformableAnchor(r-1, body);
		psb->appendDeformableAnchor(0, body);

		g_dynamicsWorld->addRigidBody(body);
	}

    glutMainLoop();

    return 0;
}
/*
btSoftBody*             btSoftBodyHelpers::CreatePatch(btSoftBodyWorldInfo& worldInfo,const btVector3& corner00,
		const btVector3& corner10,
		const btVector3& corner01,
		const btVector3& corner11,
		int resx,
		int resy,
		int fixeds,
		bool gendiags)
{
#define IDX(_x_,_y_)    ((_y_)*rx+(_x_))
	if((resx<2)||(resy<2)) return(0);
	const int       rx=resx;
	const int       ry=resy;
	const int       tot=rx*ry;
	btVector3*      x=new btVector3[tot];
	btScalar*       m=new btScalar[tot];
	int iy;

	for(iy=0;iy<ry;++iy)
	{
		const btScalar  ty=iy/(btScalar)(ry-1);
		const btVector3 py0=lerp(corner00,corner01,ty);
		const btVector3 py1=lerp(corner10,corner11,ty);
		for(int ix=0;ix<rx;++ix)
		{
			const btScalar  tx=ix/(btScalar)(rx-1);
			x[IDX(ix,iy)]=lerp(py0,py1,tx);
			m[IDX(ix,iy)]=1;
		}
	}
	btSoftBody*             psb=new btSoftBody(&worldInfo,tot,x,m);
	if(fixeds&1)    psb->setMass(IDX(0,0),0);
	if(fixeds&2)    psb->setMass(IDX(rx-1,0),0);
	if(fixeds&4)    psb->setMass(IDX(0,ry-1),0);
	if(fixeds&8)    psb->setMass(IDX(rx-1,ry-1),0);
	delete[] x;
	delete[] m;
	for(iy=0;iy<ry;++iy)
	{
		for(int ix=0;ix<rx;++ix)
		{
			const int       idx=IDX(ix,iy);
			const bool      mdx=(ix+1)<rx;
			const bool      mdy=(iy+1)<ry;
			if(mdx) psb->appendLink(idx,IDX(ix+1,iy));
			if(mdy) psb->appendLink(idx,IDX(ix,iy+1));
			if(mdx&&mdy)
			{
				if((ix+iy)&1)
				{
					psb->appendFace(IDX(ix,iy),IDX(ix+1,iy),IDX(ix+1,iy+1));
					psb->appendFace(IDX(ix,iy),IDX(ix+1,iy+1),IDX(ix,iy+1));
					if(gendiags)
					{
						psb->appendLink(IDX(ix,iy),IDX(ix+1,iy+1));
					}
				}
				else
				{
					psb->appendFace(IDX(ix,iy+1),IDX(ix,iy),IDX(ix+1,iy));
					psb->appendFace(IDX(ix,iy+1),IDX(ix+1,iy),IDX(ix+1,iy+1));
					if(gendiags)
					{
						psb->appendLink(IDX(ix+1,iy),IDX(ix,iy+1));
					}
				}
			}
		}
	}
#undef IDX
	return(psb);
}
// */
