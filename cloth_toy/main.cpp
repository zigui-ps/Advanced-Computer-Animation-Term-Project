#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btDeformableMultiBodyDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btDeformableBodySolver.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include<vector>
#include <GL/glew.h>
#include <GL/glu.h>
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include<GL/freeglut.h>
#include <stdio.h> //printf debugging

class GLDebugDrawer : public btIDebugDraw
{
	int m_debugMode;
	public:
	GLDebugDrawer();
	virtual ~GLDebugDrawer();
	virtual void	drawLine(const btVector3& from,const btVector3& to,const btVector3& fromColor, const btVector3& toColor);
	virtual void	drawLine(const btVector3& from,const btVector3& to,const btVector3& color);
	virtual void	drawSphere (const btVector3& p, btScalar radius, const btVector3& color);
	virtual void	drawTriangle(const btVector3& a,const btVector3& b,const btVector3& c,const btVector3& color,btScalar alpha);
	virtual void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color);
	virtual void	reportErrorWarning(const char* warningString);
	virtual void	draw3dText(const btVector3& location,const char* textString);
	virtual void	setDebugMode(int debugMode);
	virtual int		getDebugMode() const { return m_debugMode;}
};

GLDebugDrawer::GLDebugDrawer()
	:m_debugMode(0)
{
}
GLDebugDrawer::~GLDebugDrawer()
{
}
void	GLDebugDrawer::drawLine(const btVector3& from,const btVector3& to,const btVector3& fromColor, const btVector3& toColor)
{
	glBegin(GL_LINES);
	glColor3f(fromColor.getX(), fromColor.getY(), fromColor.getZ());
	glVertex3d(from.getX(), from.getY(), from.getZ());
	glColor3f(toColor.getX(), toColor.getY(), toColor.getZ());
	glVertex3d(to.getX(), to.getY(), to.getZ());
	glEnd();
}
void	GLDebugDrawer::drawLine(const btVector3& from,const btVector3& to,const btVector3& color)
{
	drawLine(from,to,color,color);
}
void GLDebugDrawer::drawSphere (const btVector3& p, btScalar radius, const btVector3& color)
{
	glColor4f (color.getX(), color.getY(), color.getZ(), btScalar(1.0f));
	glPushMatrix ();
	glTranslatef (p.getX(), p.getY(), p.getZ());
	int lats = 5;
	int longs = 5;
	int i, j;
	for(i = 0; i <= lats; i++) {
		btScalar lat0 = SIMD_PI * (-btScalar(0.5) + (btScalar) (i - 1) / lats);
		btScalar z0  = radius*sin(lat0);
		btScalar zr0 =  radius*cos(lat0);
		btScalar lat1 = SIMD_PI * (-btScalar(0.5) + (btScalar) i / lats);
		btScalar z1 = radius*sin(lat1);
		btScalar zr1 = radius*cos(lat1);
		glBegin(GL_QUAD_STRIP);
		for(j = 0; j <= longs; j++) {
			btScalar lng = 2 * SIMD_PI * (btScalar) (j - 1) / longs;
			btScalar x = cos(lng);
			btScalar y = sin(lng);
			glNormal3f(x * zr0, y * zr0, z0);
			glVertex3f(x * zr0, y * zr0, z0);
			glNormal3f(x * zr1, y * zr1, z1);
			glVertex3f(x * zr1, y * zr1, z1);
		}
		glEnd();
	}
	glPopMatrix();
}

void	GLDebugDrawer::drawTriangle(const btVector3& a,const btVector3& b,const btVector3& c,const btVector3& color,btScalar alpha)
{
	{
		const btVector3	n=btCross(b-a,c-a).normalized();
		glBegin(GL_TRIANGLES);
		glColor4f(color.getX(), color.getY(), color.getZ(),alpha);
		glNormal3d(n.getX(),n.getY(),n.getZ());
		glVertex3d(a.getX(),a.getY(),a.getZ());
		glVertex3d(b.getX(),b.getY(),b.getZ());
		glVertex3d(c.getX(),c.getY(),c.getZ());
		glEnd();
		
		glBegin(GL_TRIANGLES);
		glColor4f(color.getX(), color.getY(), color.getZ(),alpha);
		glNormal3d(-n.getX(),-n.getY(),-n.getZ());
		glVertex3d(a.getX(),a.getY(),a.getZ());
		glVertex3d(c.getX(),c.getY(),c.getZ());
		glVertex3d(b.getX(),b.getY(),b.getZ());
		glEnd();
	}
}
void	GLDebugDrawer::setDebugMode(int debugMode)
{
	m_debugMode = debugMode;
}
void	GLDebugDrawer::draw3dText(const btVector3& location,const char* textString)
{
	glRasterPos3f(location.x(),  location.y(),  location.z());
}
void	GLDebugDrawer::reportErrorWarning(const char* warningString)
{
	printf("%s\n",warningString);
}
void	GLDebugDrawer::drawContactPoint(const btVector3& pointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)
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


GLfloat light_diffuse[] = {1.0, 0.0, 0.0, 1.0};  /* Red diffuse light. */
GLfloat light_position[] = {1.0, 1.0, 1.0, 0.0};  /* Infinite light location. */

GLDebugDrawer* drawer = new GLDebugDrawer();
btDeformableMultiBodyDynamicsWorld* m_dynamicsWorld;
std::vector<btCollisionShape*> m_collisionShapes;
std::vector<btDeformableLagrangianForce*> m_forces;

void display(void)
{
	glClearColor(0.9, 0.9, 0.9, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	m_dynamicsWorld->debugDrawWorld();
	for (int i = 0; i < m_dynamicsWorld->getSoftBodyArray().size(); i++)
	{
		btSoftBody* psb = (btSoftBody*)m_dynamicsWorld->getSoftBodyArray()[i];
		btSoftBodyHelpers::Draw(psb, drawer, fDrawFlags::Faces);
	}
	glutSwapBuffers();
}

void nextTimestep(int time){
	glutTimerFunc(1000.0 / 60.0, nextTimestep, 0);
	float internalTimeStep = 1. / 240.f, deltaTime = 1. / 60.f;
	m_dynamicsWorld->stepSimulation(deltaTime, 4, internalTimeStep);
	glutPostRedisplay();
}

void init(){
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);

	glEnable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION);
	gluPerspective(40.0, 1.0, 1.0, 100.0);

	glMatrixMode(GL_MODELVIEW);
	gluLookAt(0.0, 0.0, 50.0,  /* eye is at (0,0,5) */
			0.0, 0.0, 0.0,      /* center is at (0,0,0) */
			0.0, 1.0, 0.);      /* up is in positive Y direction */
}

int main(int argc, char* argv[])
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB|GLUT_DEPTH);
	glutInitWindowSize(1600, 900);
	glutCreateWindow(argv[0]);

	glutDisplayFunc(display);
	glutTimerFunc(1000.0 / 60.0, nextTimestep, 0);
	init();

	auto m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	auto m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	auto m_broadphase = new btDbvtBroadphase();
	btDeformableBodySolver* deformableBodySolver = new btDeformableBodySolver();
	btDeformableMultiBodyConstraintSolver* sol = new btDeformableMultiBodyConstraintSolver();
	sol->setDeformableSolver(deformableBodySolver);
	auto m_solver = sol;

	m_dynamicsWorld = new btDeformableMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration, deformableBodySolver);
	btVector3 gravity = btVector3(0, -10, 0);
	m_dynamicsWorld->setGravity(gravity);
	m_dynamicsWorld->getWorldInfo().m_gravity = gravity;

	m_dynamicsWorld->setDebugDrawer(drawer);
	m_dynamicsWorld->setDrawFlags(-1);
	{
		///create a ground
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(150.), btScalar(25.), btScalar(150.)));

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, -50, 0));
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

		m_dynamicsWorld->addRigidBody(body);
	}
	{
		const btScalar s = 4;
		const btScalar h = 6;
		const int r = 9;
		btSoftBody* psb = btSoftBodyHelpers::CreatePatch(m_dynamicsWorld->getWorldInfo(), btVector3(-s, h, -s),
				btVector3(+s, h, -s),
				btVector3(-s, h, +s),
				btVector3(+s, h, +s), r, r, 4 + 8, true);
		psb->getCollisionShape()->setMargin(0.1);
		psb->generateBendingConstraints(2);
		psb->setTotalMass((btScalar)1.);
		psb->m_cfg.kKHR = 1; // collision hardness with kinematic objects
		psb->m_cfg.kCHR = 1; // collision hardness with rigid body
		psb->m_cfg.kDF = 2;
		psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RD;
		m_dynamicsWorld->addSoftBody(psb);

		btDeformableMassSpringForce* mass_spring = new btDeformableMassSpringForce(100,1, true);
		m_dynamicsWorld->addForce(psb, mass_spring);
		m_forces.push_back(mass_spring);

		btDeformableGravityForce* gravity_force =  new btDeformableGravityForce(gravity);
		m_dynamicsWorld->addForce(psb, gravity_force);
		m_forces.push_back(gravity_force);

		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(btVector3(0, h, -(s + 3.5)));

		btCollisionShape *shape = new btBoxShape(btVector3(s, 1, 3));
		double mass = 10;

		btVector3 localInertia(0, 0, 0);
		shape->calculateLocalInertia(mass, localInertia);

		btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
		body->setWorldTransform(startTransform);

		body->setUserIndex(-1);

		psb->appendDeformableAnchor(0, body);
		psb->appendDeformableAnchor(r - 1, body);

		m_dynamicsWorld->addRigidBody(body);
	}
	m_dynamicsWorld->setImplicit(false);
	m_dynamicsWorld->setLineSearch(false);

	glutMainLoop();
}
