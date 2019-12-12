#ifndef __OPENGL_HELPER_H__
#define __OPENGL_HELPER_H__

#include <GL/glew.h>
#include <GL/glu.h>
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include <GL/freeglut.h>
#include <glm/gtc/type_ptr.hpp>

#include <vector>

#include "LinearMath/btVector3.h"
#include "Camera.h"

void init_cameras();
glm::mat4 get_view_mat();

void mouse_callback(int button, int state, int x, int y);
void mouse_drag_callback(int x, int y);
void keyboard_callback(unsigned char key, int x, int y);

void draw_box(float width, float height, float depth);

void drawTriangle(btVector3 p0, btVector3 p1, btVector3 p2, btVector3 c);
void drawTriangle(btVector3 p0, btVector3 p1, btVector3 p2, 
		btVector3 n0, btVector3 n1, btVector3 n2, btVector3 c);

void draw_axes();

#endif // __OPENGL_HELPER_H__