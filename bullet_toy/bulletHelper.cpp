#include "bulletHelper.h"

btDiscreteDynamicsWorld* g_dynamicsWorld;

btRigidBody* createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, bool isKinematics/* = false*/){
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

void draw_box(const btVector3& half_size){
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

void draw_axes(){
	glBegin(GL_LINES);
	// draw line for x axis
	glColor3f(1.0, 0.0, 0.0);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(1.0, 0.0, 0.0);
	// draw line for y axis
	glColor3f(0.0, 1.0, 0.0);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(0.0, 1.0, 0.0);
	// draw line for Z axis
	glColor3f(0.0, 0.0, 1.0);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(0.0, 0.0, 1.0);
	glEnd();
}