#include "openglHelper.h"

std::vector<Camera*> camera_list;
int cur_camera_idx = 0;

void init_cameras(){
    Camera* camera_1 = new Camera(glm::vec3(-150, 30, 250));

    camera_list.push_back(camera_1);
}

glm::mat4 get_view_mat(){
    return camera_list[cur_camera_idx]->GetViewMatrix();
}

void keyboard_callback(unsigned char key, int x, int y){
    switch(key){
        case 'w':
        case 'W':
            camera_list[cur_camera_idx]->ProcessKeyboard(FORWARD, 0.1);
            break;
        case 'a':
        case 'A':
            camera_list[cur_camera_idx]->ProcessKeyboard(LEFT, 0.1);
            break;
        case 's':
        case 'S':
            camera_list[cur_camera_idx]->ProcessKeyboard(BACKWARD, 0.1);
        break;
        case 'd':
        case 'D':
            camera_list[cur_camera_idx]->ProcessKeyboard(RIGHT, 0.1);
        break;
    }
    glutPostRedisplay();
}


void draw_box(float width, float height, float depth){

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