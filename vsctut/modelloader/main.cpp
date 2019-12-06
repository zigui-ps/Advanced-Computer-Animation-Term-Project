#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <shader.h>
#include <camera.h>
#include <model.h>
//#include <Eigen/Dense>
#include <bvh.h>
#include <unistd.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);
void moveTo(unsigned frame, MOTION motionData, JOINT* rootJoint);
void moveJoint(JOINT* joint, MOTION* motionData, int frame_starts_index);
void drawJoint(JOINT* joint, Shader ourShader, Model ourModel);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// camera
Camera camera(glm::vec3(0.0f, 150.0f, 600.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

int main()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // uncomment this statement to fix compilation on OS X
#endif
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);
    Shader ourShader("../modelloader/vs.vs", "../modelloader/fs.fs");

    Model ourModel("../obj/sphere/sphere.obj");
    Model ourModel1("../obj/unit_sphere/unit_sphere.obj");
    
    Bvh walk;
    walk.load("../bvh/16_15_walk.bvh");
    JOINT*  root_walk = walk.getRootJoint();
    int frame = 0;

    // draw in wireframe
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    while (!glfwWindowShouldClose(window))
    {
        // per-frame time logic
        // --------------------
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        // -----
        processInput(window);

        // render
        // ------
        //glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // don't forget to enable shader before setting uniforms
        ourShader.use();

        // view/projection transformations
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 10000.0f);
        glm::mat4 view = camera.GetViewMatrix();
        ourShader.setMat4("projection", projection);
        ourShader.setMat4("view", view);

        // render the loaded model
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, glm::vec3(0.0f, 0.0f, 0.0f)); // translate it down so it's at the center of the scene
        model = glm::scale(model, glm::vec3(1.0f, 1.0f, 1.0f));	// it's a bit too big for our scene, so scale it down
        ourShader.setMat4("model", model);
        ourModel.Draw(ourShader);

        model = glm::translate(model, glm::vec3(1.0f, 0.0f, 0.0f)); // translate it down so it's at the center of the scene
        model = glm::scale(model, glm::vec3(1.0f, 1.0f, 1.0f));	// it's a bit too big for our scene, so scale it down
        ourShader.setMat4("model", model);
        ourModel1.Draw(ourShader);
    
        moveTo(frame, walk.motionData, root_walk);
        drawJoint(root_walk, ourShader, ourModel);
        frame++;

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // glfw: terminate, clearing all previously allocated GLFW resources.
    glfwTerminate();
    return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
}

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

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

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






void moveTo(unsigned frame, MOTION motionData, JOINT* rootJoint)
{
    unsigned start_index = frame * motionData.num_motion_channels;
    moveJoint(rootJoint, &motionData, start_index);
}
void moveJoint(JOINT* joint, MOTION* motionData, int frame_starts_index)
{
    int start_index = frame_starts_index + joint->channel_start;
    joint->matrix = glm::translate(glm::mat4(1.0),
                                   glm::vec3(joint->offset.x,
                                             joint->offset.y,
                                             joint->offset.z));

    for(int i = 0; i < joint->num_channels; i++)
    {
        const short& channel = joint->channels_order[i];

        float value = motionData->data[start_index + i];
        if( channel & Xposition )
        {
            joint->matrix = glm::translate(joint->matrix, glm::vec3(value, 0, 0));
        }
        if( channel & Yposition )
        {
            joint->matrix = glm::translate(joint->matrix, glm::vec3(0, value, 0));
        }
        if( channel & Zposition )
        {
            joint->matrix = glm::translate(joint->matrix, glm::vec3(0, 0, value));
        }

        if( channel & Xrotation )
        {
            joint->matrix = glm::rotate(joint->matrix, float(3.141592*(value)/180), glm::vec3(1, 0, 0));
        }
        if( channel & Yrotation )
        {
            joint->matrix = glm::rotate(joint->matrix, float(3.141592*(value)/180), glm::vec3(0, 1, 0));
        }
        if( channel & Zrotation )
        {
            joint->matrix = glm::rotate(joint->matrix, float(3.141592*(value)/180), glm::vec3(0, 0, 1));
        }
    }

    if( joint->parent != NULL )
        joint->matrix = joint->parent->matrix * joint->matrix;

    for(auto& child : joint->children)
        moveJoint(child, motionData, frame_starts_index);
}

void drawJoint(JOINT* joint, Shader ourShader, Model ourModel)
{
    // ourShader.setMat4("model", joint->matrix);
    // ourModel.Draw(ourShader);
    
    glm::mat4 scale = glm::scale(glm::mat4(1.0f), glm::vec3(joint->offset.x+1,joint->offset.y+1,joint->offset.z+1));
    glm::mat4 draw = glm::mat4(1.0f);
    if( joint->parent != NULL )
        {
            draw = glm::translate(joint->parent->matrix, glm::vec3(0.5*joint->offset.x,0.5*joint->offset.y,0.5*joint->offset.z));
        }
    ourShader.setMat4("model", draw * scale);
    ourModel.Draw(ourShader);
    

    for(auto& child : joint->children)
        drawJoint(child, ourShader, ourModel);
}
