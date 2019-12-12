#ifndef DRAWABLE
#define DRAWABLE

#include <memory>

// Drawable

class Drawable{
public:
	Drawable();
	virtual void display() = 0;

	virtual void nextTimestep(int time);
	virtual void reshape(int width, int height);
	virtual void keyboard(unsigned char key, int x, int y);
	virtual void keyboardUp(unsigned char key, int x, int y);
	virtual void special(int key, int x, int y);
	virtual void specialUp(int key, int x, int y);
	virtual void mouse(int button, int state, int x, int y);
	virtual void motion(int x, int y);
	virtual void passiveMotion(int x, int y);
};
using DrawablePtr = std::shared_ptr<Drawable>;

// DrawableWrap

template <typename T>
class DrawableWrap : Drawable{
public:
	DrawableWrap(T object);
	
	void display() override;
private:
	T object;
};

template <typename T>
DrawableWrap<T>::DrawableWrap(T object):
		object(object){
}

template <typename T>
void DrawableWrap<T>::display(){
	displayFunction(object);
}

#endif
