// Include GLFW
#include <GLFW/glfw3.h>
extern GLFWwindow* window; // The "extern" keyword here is to access the variable "window" declared in tutorialXXX.cpp. This is a hack to keep the tutorials simple. Please avoid this.

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
using namespace glm;

#define SCREEN_WIDTH 850
#define SCREEN_HEIGHT 1000

#include "controls.hpp"

namespace ViewControl{
	glm::mat4 ViewMatrix;
	glm::mat4 ProjectionMatrix;
	float alpha=1;
	// float lightPower = 80000.0f;
	float lightPower = 8.0f;

	glm::mat4 getViewMatrix(){
		return ViewMatrix;
	}
	glm::mat4 getProjectionMatrix(){
		return ProjectionMatrix;
	}

	glm::vec3 position = glm::vec3( 3, 2, 2 );
	glm::vec3 lookshere = glm::vec3(0, 0, 0);
	glm::vec3 CameraDirection = glm::vec3(0, 0, 1);
	glm::vec3 CameraOffset = glm::vec3(0,0,0);

	glm::vec3 getCameraOffset(){
		return CameraOffset;
	}

	// Initial Field of View
	float initialFoV = 45.0f;

	float speed = 3.0f; // 3 units / second
	float mouseSpeed = 0.005f;


	void setInitCameraPosition(const glm::vec3 &init_pos)
	{
		position = init_pos;
	}

	void setLookHere(const glm::vec3 &look_here)
	{
		lookshere = look_here;
	}

	void setCameraDirection(const glm::vec3 &camera_direction)
	{
		CameraDirection = camera_direction;
	}

	float getalpha()
	{
		return alpha;
	}

	float getlightPower()
	{
		return lightPower;
	}

	void computeMatricesFromInputs(){

		// glfwGetTime is called only once, the first time this function is called
		static double lastTime = glfwGetTime();

		// Compute time difference between current and last frame
		double currentTime = glfwGetTime();
		float deltaTime = float(currentTime - lastTime);

		int screen_w;
		int screen_h;
		glfwGetWindowSize(window, &screen_w, &screen_h);

		// glm::vec3 lookat_vec = glm::vec3(position.x, position.y, position.z) - lookshere;
		glm::vec3 lookat_vec = -glm::vec3(position.x, position.y, position.z) + lookshere;
		float lookat_norm = sqrt(lookat_vec.x*lookat_vec.x + lookat_vec.y*lookat_vec.y + lookat_vec.z*lookat_vec.z);
		glm::vec3 z_rot_direction;
		if (abs(lookat_vec.z) < 0.0001)
		{z_rot_direction = glm::vec3(0,0,1);}
		else
		{
			glm::vec3 side_dir = glm::cross(lookat_vec,glm::vec3(0,0,1));

			// z_rot_direction = lookat_norm * glm::normalize(glm::vec3(-lookat_vec.x, -lookat_vec.y, (lookat_vec.x*lookat_vec.x + lookat_vec.y*lookat_vec.y)/abs(lookat_vec.z) ));
			z_rot_direction = -lookat_norm * glm::normalize(glm::cross(lookat_vec, side_dir));
		}

		// Move forward
		if (glfwGetKey( window, GLFW_KEY_N ) == GLFW_PRESS){
			position += lookat_vec * deltaTime * speed;
		}
		// Move backward
		if (glfwGetKey( window, GLFW_KEY_M ) == GLFW_PRESS){
			position -= lookat_vec * deltaTime * speed;
		}
		// Strafe right
		if (glfwGetKey( window, GLFW_KEY_RIGHT ) == GLFW_PRESS){
			position += glm::vec3(lookat_vec.y, -lookat_vec.x, 0) * deltaTime * speed;
		}
		// Strafe left
		if (glfwGetKey( window, GLFW_KEY_LEFT ) == GLFW_PRESS){
			position -= glm::vec3(lookat_vec.y, -lookat_vec.x, 0)* deltaTime * speed;
		}
		// Strafe right
		if (glfwGetKey( window, GLFW_KEY_UP) == GLFW_PRESS){
			position += z_rot_direction * deltaTime * speed;
		}
		// Strafe left
		if (glfwGetKey( window, GLFW_KEY_DOWN) == GLFW_PRESS){
			position -= z_rot_direction * deltaTime * speed;
		}
		// transparent
		if (glfwGetKey( window, GLFW_KEY_H ) == GLFW_PRESS){
			alpha -= 0.03;
			if (alpha <= 0.1) alpha = 0.1;
		}
		if (glfwGetKey( window, GLFW_KEY_J ) == GLFW_PRESS){
			alpha += 0.03;
			if (alpha >= 1) alpha = 1;
		}
		// light power
		if (glfwGetKey( window, GLFW_KEY_Y ) == GLFW_PRESS){
			lightPower -= 0.5;
			if (lightPower <= 1) lightPower = 1;
		}
		if (glfwGetKey( window, GLFW_KEY_U ) == GLFW_PRESS){
			lightPower += 0.5;
			if (lightPower >= 25) lightPower = 25;
		}
		// camera pos print
		if (glfwGetKey( window, GLFW_KEY_P ) == GLFW_PRESS){
			std::cout << "camera pos : " << position.x << " " << position.y << " " << position.z << std::endl;
		}
		// camera lookat move
		if (glfwGetKey( window, GLFW_KEY_W ) == GLFW_PRESS){
			CameraOffset.z += 0.005;
		}
		if (glfwGetKey( window, GLFW_KEY_S ) == GLFW_PRESS){
			CameraOffset.z -= 0.005;
		}
		if (glfwGetKey( window, GLFW_KEY_A ) == GLFW_PRESS){
			CameraOffset += (float)0.005 * glm::normalize(glm::vec3(lookat_vec.y, -lookat_vec.x, 0));
		}
		if (glfwGetKey( window, GLFW_KEY_D ) == GLFW_PRESS){
			CameraOffset -= (float)0.005 * glm::normalize(glm::vec3(lookat_vec.y, -lookat_vec.x, 0));
		}
		// camera lookat reset
		if (glfwGetKey( window, GLFW_KEY_E ) == GLFW_PRESS){
			CameraOffset.x = 0;
			CameraOffset.y = 0;
			CameraOffset.z = 0;
		}

		float proj_width = 5.0;
		float proj_height = proj_width * SCREEN_HEIGHT/SCREEN_WIDTH;
		ProjectionMatrix = glm::perspective(glm::radians(45.0f), proj_width / proj_height, 0.01f, 700.0f);
		// Camera matrix
		ViewMatrix       = glm::lookAt(
				position,           // Camera is here
				lookshere, // and looks here : at the same position, plus "direction"
				CameraDirection  // Head is up (set to 0,-1,0 to look upside-down)
		);

		// For the next frame, the "last time" will be "now"
		lastTime = currentTime;
	}
} // end namespace