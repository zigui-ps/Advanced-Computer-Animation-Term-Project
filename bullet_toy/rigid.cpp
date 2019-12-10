#include <GL/glew.h>
#include <GL/glu.h>
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include <GL/freeglut.h>
#include <stdio.h> //printf debugging

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"

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

btDiscreteDynamicsWorld* g_dynamicsWorld;

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

void display(void)
{
	glClearColor(0.9, 0.9, 0.9, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	

	g_dynamicsWorld->stepSimulation(1.f/ 60.f, 10);
	//Debug
	g_dynamicsWorld->debugDrawWorld();
	for(int i=g_dynamicsWorld->getNumCollisionObjects() -1; i>=0;i--){
		btCollisionObject* obj = g_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		btTransform trans;

		trans = obj->getWorldTransform();

		float trans_x = float(trans.getOrigin().getX());
		float trans_y = float(trans.getOrigin().getY());
		float trans_z = float(trans.getOrigin().getZ());

		printf("world pos object %d = %f,%f,%f\n", i, trans_x, trans_y, trans_z);


		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glTranslatef(trans_x, trans_y, trans_z);

		draw_box(trans.getOrigin(), btVector3(5,5,5));
		glPopMatrix();
	}

	glutSwapBuffers();
}

void nextTimestep(int time){
	glutTimerFunc(1000.0 / 60.0, nextTimestep, 0);
	float internalTimeStep = 1. / 240.f, deltaTime = 1. / 60.f;
	//m_dynamicsWorld->stepSimulation(deltaTime, 4, internalTimeStep);
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
	gluPerspective(40.0, 1.0, 1.0, 100.0);

	glMatrixMode(GL_MODELVIEW);
	gluLookAt(50.0, 50.0, 50.0,   /* eye is at () */
			0.0, 0.0, 0.0,      /* center is at (0,0,0) */
			0.0, 1.0, 0.);      /* up is in positive Y direction */

    printf("Init GL\n");
}

void init_bullet_world(){
    btBroadphaseInterface* broadphase = new btDbvtBroadphase();
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    g_dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
    g_dynamicsWorld->setGravity(btVector3(0,-10,0));

    printf("Init bullet world\n");
}

btRigidBody* createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, const btVector4& color = btVector4(1, 0, 0, 1)){
	//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			shape->calculateLocalInertia(mass, localInertia);
		
		btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
		body->setWorldTransform(startTransform);

		body->setUserIndex(-1);
		g_dynamicsWorld->addRigidBody(body);
		return body;
}

int main(int argc, char* argv[]){
    init_gl(argc, argv);

    init_bullet_world();
	
	//		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);
	g_dynamicsWorld->setDebugDrawer(debugDrawer);
	// create a ground
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
	
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50, 0));
	{
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}

	{
		btBoxShape* box = new btBoxShape(btVector3(btScalar(5.), btScalar(5.), btScalar(5.)));
		
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(5.f);
		
		startTransform.setOrigin(btVector3(0,50,0));
		createRigidBody(mass, startTransform, box);
	}
	{
		btBoxShape* box = new btBoxShape(btVector3(btScalar(5.), btScalar(5.), btScalar(5.)));
		
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(5.f);
		
		startTransform.setOrigin(btVector3(5,70,0));
		createRigidBody(mass, startTransform, box);
	}

    glutMainLoop();

    return 0;
}