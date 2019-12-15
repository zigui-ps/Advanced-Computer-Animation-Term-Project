#include <GL/glew.h>
#include <GL/glu.h>
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include <GL/freeglut.h>
#include <stdio.h> //printf debugging
#include <tinyxml2.h>

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"

#include "bulletHelper.h"
#include "openglHelper.h"
#include "CollisionObject.h"
#include "Render/Render.h"
#include "Skeleton/Skeleton.h"
#include "Motion/MotionGraph.h"

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
GLfloat light_diffuse[] = {1.0, 1.0, 1.0, 1.0};  /* Red diffuse light. */
GLfloat light_position[] = {10.0, 10.0, 1.0, 0.0};  /* Infinite light location. */

btRigidBody *invisible_box;
btRigidBody *jump_building;
btSoftBody *rope;
btSoftBody *cloak;
GraphPlayerPtr player;
SkeletonPtr skel;
ShapeInfoPtr ground;

int cnt=0;
void display(void)
{
	glClearColor(0.9, 0.9, 0.9, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixf((float*)glm::value_ptr(get_view_mat()));


	//Debug
	g_dynamicsWorld->debugDrawWorld();


	// btRigidBody* body = invisible_box;
	// btTransform trans;

	// trans = body->getWorldTransform();

	// float trans_x = float(trans.getOrigin().getX());
	// float trans_y = float(trans.getOrigin().getY());
	// float trans_z = float(trans.getOrigin().getZ());

	// //printf("world pos object %d = %f,%f,%f\n", 0, trans_x, trans_y, trans_z);

	// glMatrixMode(GL_MODELVIEW);
	// glPushMatrix();
	// double m[16];
	// trans.getOpenGLMatrix(m);
	// glMultMatrixd(m);

	// draw_box(0.5,0.5,0.5);
	// glPopMatrix();

	for (int i = 0; i < g_dynamicsWorld->getSoftBodyArray().size(); i++)
	{
		btSoftBody* psb = (btSoftBody*)g_dynamicsWorld->getSoftBodyArray()[i];
		draw_soft_body(psb);
	}
	skel->display();
	ground->display();
	draw_rope(rope, 0.5, 20, 20);

	glutSwapBuffers();
}

bool rope_anchor=false;
void nextTimestep(int time){
	glutTimerFunc(1000.0 / 60.0, nextTimestep, 0);
	float internalTimeStep = 1. / 240.f, deltaTime = 1. / 60.f;

	player->nextTimestep(time);
	// btTransform trans = invisible_box->getWorldTransform();

	// skel->location = Eigen::Vector3d(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
	//skel->root->jointTransform = Eigen::Quaterniond::Identity();
	//skel->setTransform();
	g_dynamicsWorld->stepSimulation(deltaTime, 4, internalTimeStep);

		btTransform trans = skel->getCollisionObject("pelvis")->m_obj->getWorldTransform();

		float trans_x = float(trans.getOrigin().getX());
		float trans_y = float(trans.getOrigin().getY());
		float trans_z = float(trans.getOrigin().getZ());
		printf("world pos object root = %f,%f,%f\n", trans_x, trans_y, trans_z);

	if(!rope_anchor){
		rope->m_nodes[rope->m_nodes.size()-1].m_x = trans.getOrigin();
		rope->appendDeformableAnchor(rope->m_nodes.size() - 1, skel->getCollisionObject("pelvis")->m_obj);

		cloak->translate(btVector3(0.0,0.0,0.5));
		btRigidBody* lclavicle = skel->getCollisionObject("lclavicle")->m_obj;
		trans = lclavicle->getWorldTransform();
		//cloak->m_nodes[5*9].m_x = trans.getOrigin() + btVector3(0.5,1,-1);
		cloak->appendDeformableAnchor(5*9, lclavicle);

		btRigidBody* rclavicle = skel->getCollisionObject("rclavicle")->m_obj;
		trans = rclavicle->getWorldTransform();
		//cloak->m_nodes[5*9+4].m_x = trans.getOrigin() + btVector3(-0.5, 0.8,-1);
		cloak->appendDeformableAnchor(5*9+4, rclavicle);

		rope_anchor=true;
	}

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

	// Color
	glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
	glEnable(GL_COLOR_MATERIAL);

	glEnable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION);
	gluPerspective(40.0, 1.0, 1.0, 10000.0);

	printf("Init GL\n");

	printf("== How to Handle Camera ==========================\n");
	printf(" - R: Reset to default condition.\n");
	printf(" - W: Make the camera go front.\n");
	printf(" - A: Make the camera go left.\n");
	printf(" - S: Make the camera go backward.\n");
	printf(" - D: Make the camera go right.\n");
	printf(" - Zcompound2: Make the camera go up.\n");
	printf(" - X: Make the camera go down.\n");
	printf(" - MOUSE RIGHT CLICK & DRAG: Rotate the camera.\n");
	printf("==================================================\n");

}

void make_rigid_body(SkeletonPtr skel){
	// TODO
	printf("make rigid body called\n");

	btVector3 f_v;
	btCompoundShape* compoundShape = new btCompoundShape();

	btScalar* masses = new btScalar[skel->nodeList.size()];
	for(int i = 0; i < skel->nodeList.size(); i++){
		auto cur = skel->nodeList[i];
		auto shape_list = cur->shapeList;
		for(auto shape : shape_list){
			auto obj = shape->m_obj;
			btTransform trans = obj->getWorldTransform();
			
			compoundShape->addChildShape(trans, obj->getCollisionShape());

			if(i==0){// pelvis
			f_v = obj->getLinearVelocity();
			}
		}
		masses[i] = 1; // how to get obj mass? // TODO: change to obj->m_mass;
	}

	btTransform principal;
	principal.setIdentity();

	btScalar mass(15.0f); // maybe sumup masses
	


	// Remove
	for(int i = 0; i < skel->nodeList.size(); i++){
		auto cur = skel->nodeList[i];
		auto shape_list = cur->shapeList;
		for(auto shape : shape_list){
			auto obj = shape->m_obj;
			g_dynamicsWorld->removeCollisionObject(obj);
			delete obj;		
		}
	}

	btRigidBody* body = create_rigid_body(mass, principal, compoundShape);
	body->setLinearVelocity(f_v);

	//g_dynamicsWorld->addRigidBody(body);


	//rope->m_nodes[rope->m_nodes.size()-1].m_x = trans.getOrigin();
	rope->appendDeformableAnchor(rope->m_nodes.size() - 1, body);

	cloak->m_cfg.kCHR = 0; // collision hardness with rigid body
	cloak->m_cfg.collisions = btSoftBody::fCollision::SDF_RS | btSoftBody::fCollision::CL_SS; // collision between soft and rigid makes weird.
	cloak->appendDeformableAnchor(5*9, body);
	cloak->appendDeformableAnchor(5*9+4, body);
}

int main(int argc, char* argv[]){
	init_gl(argc, argv);

	init_cameras();

	init_bullet_world();


	debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);
	g_dynamicsWorld->setDebugDrawer(debugDrawer);

	{
		//create_ground();

	}

	{
		//invisible_box = create_rigid_body(mass, startTransform, box);

		// Deactivate collision for box.
		//invisible_box->setCollisionFlags(invisible_box->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);

		//rope->appendDeformableAnchor(rope->m_nodes.size() - 1, invisible_box);

		//invisible_box->setLinearVelocity(btVector3(0, 2, 0));
		//invisible_box->setAngularVelocity(btVector3(10, 0, 0));

		// ground = ShapeInfoPtr(new GroundShape(100, 100, 1, 1));
		// TiXmlDocument doc; doc.LoadFile("../character/gen2.xml");
		// skel = SkeletonPtr(new Skeleton(doc));
		// skel->turnOffKinematics();
		// CollisionObjectPtr coll_obj = skel->getCollisionObject("pelvis");
		// btRigidBody* body = coll_obj->m_obj;
		
		// skel->location = Eigen::Vector3d(5, 80.5, -3);
		// skel->setTransform();

		// btTransform trans = body->getWorldTransform();

		// float trans_x = float(trans.getOrigin().getX());
		// float trans_y = float(trans.getOrigin().getY());
		// float trans_z = float(trans.getOrigin().getZ());

		// rope->appendDeformableAnchor(rope->m_nodes.size() - 1, body);


		// printf("world pos object %d = %f,%f,%f\n", 0, trans_x, trans_y, trans_z);



		// MotionGraphPtr graph = MotionGraphPtr(new MotionGraph("../motion/MotionGraph.cfg"));
		// player = GraphPlayerPtr(new GraphPlayer(skel, graph));

		// skel->location = Eigen::Vector3d(5, 80.5, -3);
		// skel->setTransform();
	}
	// Ground
	{
		ground = ShapeInfoPtr(new GroundShape(100, 100, 1, 1));
	}
	// Jump building
	{
		// jump_building = create_jump_building();
		// btTransform trans = jump_building->getWorldTransform();

		// float trans_x = float(trans.getOrigin().getX());
		// float trans_y = float(trans.getOrigin().getY());
		// float trans_z = float(trans.getOrigin().getZ());
//		printf("world pos object %d = %f,%f,%f\n", 0, trans_x, trans_y, trans_z);
	}
	// Human
	{
		Eigen::Vector3d location = Eigen::Vector3d(0, 80, -65);
		TiXmlDocument doc; doc.LoadFile("../character/gen2.xml");
		skel = SkeletonPtr(new Skeleton(doc));
		MotionGraphPtr graph = MotionGraphPtr(new MotionGraph("../motion/MotionGraph.cfg"));
		player = GraphPlayerPtr(new GraphPlayer(skel, graph, location));
	}
	// Rope
	{
		CollisionObjectPtr root = skel->getCollisionObject("pelvis");
		btTransform root_origin = root->m_obj->getWorldTransform();

		float trans_x = float(root_origin.getOrigin().getX());
		float trans_y = float(root_origin.getOrigin().getY());
		float trans_z = float(root_origin.getOrigin().getZ());
		printf("world pos object %d = %f,%f,%f\n", 0, trans_x, trans_y, trans_z);
		rope = create_rope(btVector3(0, 85,-60),btVector3(0,85,-55));

		//rope->appendDeformableAnchor(rope->m_nodes.size() - 1, root->m_obj);

	}

	// Cloak
	{
		cloak = create_cloak();
	}

//*
	
// */
	glutMainLoop();

	return 0;
}
