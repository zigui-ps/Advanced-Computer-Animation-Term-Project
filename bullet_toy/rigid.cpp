#include <GL/glew.h>
#include <GL/glu.h>
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include <GL/freeglut.h>
#include <stdio.h> //printf debugging

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"

#include "bulletHelper.h"
#include "openglHelper.h"
#include "CollisionObject.h"

#include "bvh.h"

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
GLfloat light_position[] = {1.0, 1.0, -1.0, 0.0};  /* Infinite light location. */

btRigidBody *box2;
btSoftBody *rope;

Bvh bvh;

int cnt=0;
void display(void)
{
	glClearColor(0.9, 0.9, 0.9, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixf((float*)glm::value_ptr(get_view_mat()));
	// gluLookAt(250.0, -70.0, 150.0,   /* eye is at () */
	// 		0.0, 20.0, -95.0,      /* center is at (0,0,0) */
	// 		0.0, 1.0, 0.);      /* up is in positive Y direction */


	//Debug
	g_dynamicsWorld->debugDrawWorld();


	btRigidBody* body = box2;
	btTransform trans;

	trans = body->getWorldTransform();

	float trans_x = float(trans.getOrigin().getX());
	float trans_y = float(trans.getOrigin().getY());
	float trans_z = float(trans.getOrigin().getZ());

	//printf("world pos object %d = %f,%f,%f\n", 0, trans_x, trans_y, trans_z);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	double m[16];
	trans.getOpenGLMatrix(m);
	glMultMatrixd(m);

	draw_box(5,5,5);
	glPopMatrix();

	// Bvh simulation
	if(cnt>=bvh.motionData.num_frames) cnt =0;
	bvh.draw_bvh(cnt++);

	for (int i = 0; i < g_dynamicsWorld->getSoftBodyArray().size(); i++)
	{
		btSoftBody* psb = (btSoftBody*)g_dynamicsWorld->getSoftBodyArray()[i];
		draw_soft_body(psb);
	}

	glutSwapBuffers();
}

void nextTimestep(int time){
	glutTimerFunc(1000.0 / 60.0, nextTimestep, 0);
	float internalTimeStep = 1. / 240.f, deltaTime = 1. / 60.f;

	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(((float)rand()/RAND_MAX) * 20, 0, 0));
	//box1->setTransform(t);

	g_dynamicsWorld->stepSimulation(deltaTime, 4, internalTimeStep);

	glutPostRedisplay();
}

void init_gl(int argc, char* argv[]){
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB|GLUT_DEPTH);
	glutInitWindowSize(1600, 900);
	glutCreateWindow("Rigid");

	glutDisplayFunc(display);

	// Register callback functions
	// ------------------------------------------------------------
	glutMouseFunc(mouse_callback);
	glutMotionFunc(mouse_drag_callback);
	glutKeyboardFunc(keyboard_callback);
	// ------------------------------------------------------------

	glutTimerFunc(1000.0 / 60.0, nextTimestep, 0);

	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);

	glEnable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION);
	gluPerspective(40.0, 1.0, 1.0, 10000.0);

	printf("Init GL\n");

	printf("== How to Handle Camera ==========================\n");
	printf(" - R: Reset to default condition.");
	printf(" - W: Make the camera go front.\n");
	printf(" - A: Make the camera go left.\n");
	printf(" - S: Make the camera go backward.\n");
	printf(" - D: Make the camera go right.\n");
	printf(" - Z: Make the camera go up.\n");
	printf(" - X: Make the camera go down.\n");
	printf(" - MOUSE RIGHT CLICK & DRAG: Rotate the camera.\n");
	printf("==================================================\n");

}

int main(int argc, char* argv[]){
	init_gl(argc, argv);

	init_cameras();

	init_bullet_world();


	debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);
	g_dynamicsWorld->setDebugDrawer(debugDrawer);
	// create a ground
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(500.), btScalar(50.), btScalar(500.)));

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50, 0));
	{
		btScalar mass(0.);
		create_rigid_body(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}

	// {
	// 	// btBoxShape *box = new btBoxShape(btVector3(btScalar(5.), btScalar(5.), btScalar(5.)));

	// 	// btTransform startTransform;
	// 	// startTransform.setIdentity();

	// 	// btScalar mass(5.f);

	// 	// startTransform.setOrigin(btVector3(0,10,0));
	// 	// box1 = createRigidBody(mass, startTransform, box, true);

	// 	box1 = new CollisionObject();
	// 	box1->createCollisionObject(0,10,0, 5,5,5);
	// }
	{
		btSoftBody* psb = btSoftBodyHelpers::CreateRope(g_dynamicsWorld->getWorldInfo(),btVector3(-500, 55, 50), btVector3(-400, 55, 50),10, 1);
		psb->getCollisionShape()->setMargin(0.1);
		psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
		psb->m_cfg.kCHR = 1; // collision hardness with rigid body
		psb->m_cfg.kDF = 2;
		psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RS | btSoftBody::fCollision::CL_RS; // | btSoftBody::fCollision::VF_SS ;

		psb->m_materials[0]->m_kLST = 0;
		
		psb->setTotalMass(5.f);
		g_dynamicsWorld->addSoftBody(psb);

		btDeformableMassSpringForce* mass_spring = new btDeformableMassSpringForce(100, 1, true);
		g_dynamicsWorld->addForce(psb, mass_spring);

		btDeformableGravityForce* gravity_force =  new btDeformableGravityForce(btVector3(0,-10, 0));
		g_dynamicsWorld->addForce(psb, gravity_force);
	
		btBoxShape *box = new btBoxShape(btVector3(btScalar(5.), btScalar(5.), btScalar(5.)));

		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(10.0f);

		startTransform.setOrigin(btVector3(-400,50,50));
		box2 = create_rigid_body(mass, startTransform, box);
	

		psb->appendDeformableAnchor(psb->m_nodes.size() - 1, box2);

		box2->setLinearVelocity(btVector3(0, 20, 0));
		box2->setAngularVelocity(btVector3(10, 0, 0));
	}

	// Load BVH
	bvh.load("../../vsctut/bvh/16_15_walk.bvh");

	glutMainLoop();

	return 0;
}
