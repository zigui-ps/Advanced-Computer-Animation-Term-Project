#include <GL/glew.h>
#include <GL/glu.h>
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include <GL/freeglut.h>
#include <stdio.h> //printf debugging
#include <tinyxml2.h>
#include <GLFW/glfw3.h>

#include <shader.h>
#include <model.h>

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
btRigidBody *jump_building, *human;
btSoftBody *rope;
btSoftBody *cloak;
GraphPlayerPtr player;
SkeletonPtr skel;
ShapeInfoPtr ground;
Eigen::Affine3d offset;
bool enable = true;

int cnt=0;
void display(void)
{
	//Debug
	g_dynamicsWorld->debugDrawWorld();

	// Draw cloak
	draw_soft_body(cloak);
	skel->display();
	ground->display();
	if(rope) draw_rope(rope, 0.5, 20, 20);
	else printf("??\n");

//	glutSwapBuffers();
}

bool rope_anchor=false;
void nextTimestep(int time){
//	glutTimerFunc(1000.0 / 60.0, nextTimestep, 0);
	float internalTimeStep = 1. / 6000.f, deltaTime = 1. / 600.f;

	static int cnt = 0;
	if(cnt++%10 == 0) player->nextTimestep(time);
	// btTransform trans = invisible_box->getWorldTransform();

	// skel->location = Eigen::Vector3d(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
	//skel->root->jointTransform = Eigen::Quaterniond::Identity();
	//skel->setTransform();
	if(enable) g_dynamicsWorld->stepSimulation(deltaTime, 10, internalTimeStep);
	if(player->status->isTerminate()){
		btTransform trans = human->getWorldTransform();
		Eigen::Affine3d t2; trans.getOpenGLMatrix(t2.data());
		t2 = t2 * offset;
		skel->location = t2.translation();
		skel->root->jointTransform = Eigen::Quaterniond(t2.linear());
	}

		btTransform trans = skel->getCollisionObject("pelvis")->m_obj->getWorldTransform();

		float trans_x = float(trans.getOrigin().getX());
		float trans_y = float(trans.getOrigin().getY());
		float trans_z = float(trans.getOrigin().getZ());
		printf("world pos object root = %f,%f,%f\n", trans_x, trans_y, trans_z);

	if(!rope_anchor){

		rope = create_rope(btVector3(0, 85,-10),btVector3(100,85,-60));
		for(int i=0;i<rope->m_nodes.size()-1;i++){
			trans_x = rope->m_nodes[i].m_q.getX();
			trans_y = rope->m_nodes[i].m_q.getY();
			trans_z = rope->m_nodes[i].m_q.getZ();

			printf("before = %f,%f,%f\n", trans_x, trans_y, trans_z);
		}
		rope->m_nodes[rope->m_nodes.size()-1].m_x = trans.getOrigin();
		rope->appendAnchor(rope->m_nodes.size() - 1, skel->getCollisionObject("pelvis")->m_obj);
		for(int i=0;i<rope->m_nodes.size()-1;i++){
			rope->m_nodes[i].m_v = btVector3(0,0,0);
			trans_x = rope->m_nodes[i].m_x.getX();
			trans_y = rope->m_nodes[i].m_x.getY();
			trans_z = rope->m_nodes[i].m_x.getZ();

			printf("after = %f,%f,%f\n", trans_x, trans_y, trans_z);
		
		}
	
		cloak->translate(btVector3(0.0,0.0,0.5));
		btRigidBody* lclavicle = skel->getCollisionObject("lclavicle")->m_obj;
		trans = lclavicle->getWorldTransform();
		//cloak->m_nodes[5*9].m_x = trans.getOrigin() + btVector3(0.5,1,-1);
		cloak->appendAnchor(5*9, lclavicle);

		btRigidBody* rclavicle = skel->getCollisionObject("rclavicle")->m_obj;
		trans = rclavicle->getWorldTransform();
		//cloak->m_nodes[5*9+4].m_x = trans.getOrigin() + btVector3(-0.5, 0.8,-1);
		cloak->appendAnchor(5*9+4, rclavicle);

		rope_anchor=true;
	}

	//glutPostRedisplay();
}

const unsigned int SCR_WIDTH = 2400;
const unsigned int SCR_HEIGHT = 1800;

GLFWwindow* window;
Camera camera(glm::vec3(-50, 85, -50));
bool firstMouse = true;
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = -xpos + lastX;
    float yoffset = -ypos + lastY;

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods){
	player->keyboard(key, 0, 0);
    switch(key){
        case 'w':
        case 'W':
            camera.ProcessKeyboard(FORWARD, 0.01);
            break;
        case 'a':
        case 'A':
						camera.ProcessKeyboard(LEFT, 0.01);
            break;
        case 's':
        case 'S':
						camera.ProcessKeyboard(BACKWARD, 0.01);
        break;
        case 'd':
        case 'D':
						camera.ProcessKeyboard(RIGHT, 0.01);
        break;
        case 'z':
        case 'Z':
						camera.ProcessKeyboard(UP, 0.01);
        break;
        case 'x':
        case 'X':
						camera.ProcessKeyboard(DOWN, 0.01);
        break;
        case 'r':
        case 'R':
						camera.reset();
        break;
    }
}


void init_gl(int argc, char* argv[]){
//	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
//  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
//  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
    if (!glfwInit())
        exit(EXIT_FAILURE);
    
		window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Superman", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window);

		GLenum res = glewInit();
    
		glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
		glfwSetKeyCallback(window, key_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

		glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
		glLightfv(GL_LIGHT0, GL_POSITION, light_position);
		glEnable(GL_LIGHT0);
		glEnable(GL_LIGHTING);
		glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
		glEnable(GL_COLOR_MATERIAL);
		glEnable(GL_DEPTH_TEST);

		/*
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
		// */
}

void make_rigid_body(SkeletonPtr skel){
	// TODO
	printf("make rigid body called\n");

	btVector3 f_v;
	btTransform dx;
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
				dx.setFromOpenGLMatrix(skel->nodeLocation[0].data());
			}
		}
		masses[i] = 1; // how to get obj mass? // TODO: change to obj->m_mass;
	}

	btTransform principal;
	principal.setIdentity();

	btScalar mass(100.0f); // maybe sumup masses



	// Remove
	for(int i = 0; i < skel->nodeList.size(); i++){
		auto cur = skel->nodeList[i];
		auto shape_list = cur->shapeList;
		for(auto shape : shape_list){
			auto obj = shape->m_obj;
			g_dynamicsWorld->removeCollisionObject(obj);
			//delete obj;		
		}
	}

	human = create_rigid_body(mass, principal, compoundShape);
	human->setLinearVelocity(f_v);
	dx = human->getWorldTransform().inverse() * dx;
	dx.getOpenGLMatrix(offset.data());

	//g_dynamicsWorld->addRigidBody(body);


	//rope->m_nodes[rope->m_nodes.size()-1].m_x = trans.getOrigin();
	rope->m_anchors.pop_back();
	rope->appendAnchor(rope->m_nodes.size() - 1, human);

	//cloak->m_cfg.kCHR = 1; // collision hardness with rigid body
	//cloak->m_cfg.collisions = btSoftBody::fCollision::CL_SS | btSoftBody::fCollision::CL_RS; // collision between soft and rigid makes weird.
	cloak->m_anchors.pop_back();
	cloak->m_anchors.pop_back();
	cloak->appendAnchor(5*9, human);
	cloak->appendAnchor(5*9+4, human);
}

int main(int argc, char* argv[]){
	init_gl(argc, argv);

	init_bullet_world();

	debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);
	g_dynamicsWorld->setDebugDrawer(debugDrawer);

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
		//rope = create_rope(btVector3(0, 85,-60),btVector3(100,85,-60));

		//rope->appendDeformableAnchor(rope->m_nodes.size() - 1, root->m_obj);

	}

	// Cloak
	{
		cloak = create_cloak();
	}

	Shader ourShader("../modelloader/vs.vs", "../modelloader/fs.fs");
//	Shader ourShader("../modelloader/vertexshader.txt", "../modelloader/fragshader.txt");
	//*
	Model ourModel("../obj/unit_sphere/unit_sphere.obj");
	// Model ourModel1("../obj/mill/wood_tower1.obj");
	Model ourModel1("../obj/ab/Hot_air_balloon_v1_L2.123c69a97f0e-9977-45dd-9570-457189ce2941/11809_Hot_air_balloon_l2.obj");
	Model ourModel2("../obj/unit_sphere/unit_sphere.obj");
	Model ourModel3("../obj/crane/Amaryllis City.obj");
// */
	while (!glfwWindowShouldClose(window))
	{
		glfwMakeContextCurrent(window);
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		glViewport(0, 0, width, height);
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		ourShader.use();
		{
			// view/projection transformations
			glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100000.0f);
			glm::mat4 view = camera.GetViewMatrix();
			ourShader.setMat4("projection", projection);
			ourShader.setMat4("view", view);
		}

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(40.0, 1.0, .1, 1e5);
		glm::mat4 view = camera.GetViewMatrix();
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glMultMatrixf((float*)glm::value_ptr(view));
		{
        // render the loaded model
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::scale(model, glm::vec3(0.0f, 0.0f, 0.0f));	// it's a bit too big for our scene, so scale it down
        ourShader.setMat4("model", model);
        ourModel.Draw(ourShader);
        
        glm::mat4 tower = glm::mat4(1.0f);
        tower = glm::rotate(tower, float(-3.141592/2.0),glm::vec3(1.0f, 0.0f, 0.0f));
        tower = glm::scale(tower, glm::vec3(0.5f, 0.5f, 0.5f));	// it's a bit too big for our scene, so scale it down
        ourShader.setMat4("model", tower);
        ourModel1.Draw(ourShader);
        glm::mat4 crane = glm::mat4(1.0f);
        crane = glm::scale(crane, glm::vec3(0.5f, 0.5f, 0.5f));	// it's a bit too big for our scene, so scale it down
        ourShader.setMat4("model", crane);
        ourModel3.Draw(ourShader);
		}

		glUseProgram(0);
		display();
		nextTimestep(0);

		glfwSwapBuffers(window);
		glfwPollEvents();
		// */
	}
	//glutMainLoop();

	return 0;
}
