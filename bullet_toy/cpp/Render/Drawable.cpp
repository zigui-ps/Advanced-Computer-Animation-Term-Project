#include <GL/glew.h>
#include <GL/glu.h>
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include<GL/freeglut.h>
#include "Render/Drawable.h"

// Drawable

Drawable::Drawable(){}
void Drawable::nextTimestep(int time){}
void Drawable::reshape(int width, int height){}
void Drawable::keyboard(unsigned char key, int x, int y){}
void Drawable::keyboardUp(unsigned char key, int x, int y){}
void Drawable::special(int key, int x, int y){}
void Drawable::specialUp(int key, int x, int y){}
void Drawable::mouse(int button, int state, int x, int y){}
void Drawable::motion(int x, int y){}
void Drawable::passiveMotion(int x, int y){}

