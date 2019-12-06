#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <shader.h>
#include <Eigen/Dense>
#include <bvh.h>
#include <unistd.h>
using namespace Eigen;

//int jacobian(GLFWwindow *window);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);
void drawjoint(float* root_p, JOINT* joint, MOTION* motionData, int frame_starts_index, Shader ourShader, int n_channels, int n_frames, int* j );
int opt( JOINT* joint, MOTION* motionData, int frame_starts_index, int n_frames, JOINT* refer, MOTION* motion_ref, float cost, int j);


class state
{
public:
    float x_i;
    float y_i;
    float z_i;
    
    float x_f;
    float y_f;
    float z_f;

    float x;
    float y;
    float z;

};


// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
// camera
glm::vec3 cameraPos   = glm::vec3(0.0f, 150.0f, 600.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp    = glm::vec3(0.0f, 1.0f, 0.0f);

bool firstMouse = true;
float yaw   = -90.0f;	// yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float pitch =  0.0f;
float lastX =  800.0f / 2.0;
float lastY =  600.0 / 2.0;
float fov   =  45.0f;

// timing
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;


int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // uncomment this statement to fix compilation on OS X
#endif

    // glfw window creation
    // --------------------
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

    // tell GLFW to capture our mouse
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


    Shader ourShader("../IK/vs.vs", "../IK/fs.fs");

    float vertices[] = {
        -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,
         0.5f, -0.5f, -0.5f,  1.0f, 0.0f,
         0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
         0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
        -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, 0.0f,

        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
         0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
        -0.5f,  0.5f,  0.5f,  0.0f, 1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,

        -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
        -0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
        -0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

         0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
         0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
         0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
         0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
         0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,
         0.5f, -0.5f, -0.5f,  1.0f, 1.0f,
         0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
         0.5f, -0.5f,  0.5f,  1.0f, 0.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, 0.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f,

        -0.5f,  0.5f, -0.5f,  0.0f, 1.0f,
         0.5f,  0.5f, -0.5f,  1.0f, 1.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
        -0.5f,  0.5f,  0.5f,  0.0f, 0.0f,
        -0.5f,  0.5f, -0.5f,  0.0f, 1.0f
    };
    glm::vec3 cubePositions[] = {
        glm::vec3(0.0f,  0.0f,  0.0f), // BODY
        glm::vec3(0.0f,  1.0f,  0.0f), //HEAD
        glm::vec3(1.5f,  0.0f, 0.0f),
        glm::vec3(-2.0f,  0.0f,  0.0f), //arm
        glm::vec3(1.0f,  0.0f,  0.0f),
        glm::vec3(-1.0f,  0.0f,  0.0f)// hand
        };
    unsigned int VBO, VAO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // texture coord attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);



    ////////////////            TEXTURE             ///////////////
    // load and create a texture 
    unsigned int texture1, texture2;
    // texture 1
    glGenTextures(1, &texture1);
    glBindTexture(GL_TEXTURE_2D, texture1);
    // set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // load image, create texture and generate mipmaps
    int width, height, nrChannels;
    stbi_set_flip_vertically_on_load(true); // tell stb_image.h to flip loaded texture's on the y-axis.
    unsigned char *data = stbi_load("../obj/BLACK.jpg", &width, &height, &nrChannels, 0);
    if (data)
    {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    else
    {
        std::cout << "Failed to load texture" << std::endl;
    }
    stbi_image_free(data);
    // texture 2
    // ---------
    glGenTextures(1, &texture2);
    glBindTexture(GL_TEXTURE_2D, texture2);
    // set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // set texture filtering parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // load image, create texture and generate mipmaps
    data = stbi_load("../obj/awesomeface.png", &width, &height, &nrChannels, 0);
    if (data)
    {
        // note that the awesomeface.png has transparency and thus an alpha channel, so make sure to tell OpenGL the data type is of GL_RGBA
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    else
    {
        std::cout << "Failed to load texture" << std::endl;
    }
    stbi_image_free(data);

    // tell opengl for each sampler to which texture unit it belongs to (only has to be done once)
    // -------------------------------------------------------------------------------------------
    ourShader.use();
    ourShader.setInt("texture1", 0);
    ourShader.setInt("texture2", 1);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    //////bvh////////////////
    Bvh motion;
    Bvh jump;
    //motion.load("../bvh/16_01_jump.bvh");
    motion.load("../bvh/16_15_walk.bvh");
    jump.load("../bvh/16_34_slow walk, stop.bvh");
    
    JOINT*  root = motion.getRootJoint();
    JOINT*  root_jump = jump.getRootJoint();
    //std::cout<<motion.motionData.num_frames<<std::endl;
    int j = 0;
    // float *root_p = new float(3);
    float root_x = 0, root_y = 0, root_z = 0;
    float off_x = 0, off_y = 0, off_z = 0;
    float ini_x = 0, ini_y = 0, ini_z = 0;
    float* root_p[9] = {&root_x, &root_y, &root_z, &off_x, &off_y, &off_z, &ini_x, &ini_y, &ini_z};
    //std::cout<<*root_p[3]<<std::endl;
    
    for (j = 0;j<80;j++)
    {
        float cost = 0;
        cost = opt( root, &motion.motionData, j*motion.motionData.num_motion_channels,
        motion.motionData.num_frames, root_jump, &jump.motionData, 0, j);
        std::cout<<j<<","<<cost<<std::endl;
    }
    





    // render loop
    // -----------
    while (!glfwWindowShouldClose(window))
    {
        processInput(window);
        // per-frame time logic
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

        // bind textures on corresponding texture units
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture1);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, texture2);

        // activate shader
        ourShader.use();

        // pass projection matrix to shader (note that in this case it could change every frame)
        glm::mat4 projection = glm::perspective(glm::radians(fov), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 10000.0f);
        ourShader.setMat4("projection", projection);

        // camera/view transformation
        glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
        ourShader.setMat4("view", view);









        
        
        glBindVertexArray(VAO);
        
        //body
        glm::mat4 model = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
        glm::mat4 trans = glm::mat4(1.0f);
        glm::mat4 rot1 = glm::mat4(1.0f);
        glm::mat4 rot2 = glm::mat4(1.0f);
        glm::mat4 rot3 = glm::mat4(1.0f);
        glm::mat4 scale = glm::scale(glm::mat4(1.0f), glm::vec3(2.0f, 3.0f, 1.0f));
        ourShader.setMat4("model", model = model * trans * scale);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        drawjoint(*root_p, root, &motion.motionData,j*motion.motionData.num_motion_channels, ourShader, motion.motionData.num_motion_channels, motion.motionData.num_frames, &j );
        //std::cout<<*root_p[0]<<","<<*root_p[1]<<std::endl;
        if ( j >= (69) )
        {
            j = -1;
            *root_p[3] = *root_p[0] + *root_p[3]-*root_p[6];
            *root_p[4] = *root_p[1] + *root_p[4]-*root_p[7];
            *root_p[5] = *root_p[2] + *root_p[5]-*root_p[8];
            std::cout<<"fin"<<root_p[3]<<std::endl;
        }
        j++;


        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
        usleep(33333);

    }

    // optional: de-allocate all resources once they've outlived their purpose:
    // ------------------------------------------------------------------------
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    // delete[] root_p;

    return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    float cameraSpeed = 500 * deltaTime;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        cameraPos += cameraSpeed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        cameraPos -= cameraSpeed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
}

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

    float sensitivity = 0.1f; // change this value to your liking
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    yaw += xoffset;
    pitch += yoffset;

    // make sure that when pitch is out of bounds, screen doesn't get flipped
    if (pitch > 89.0f)
        pitch = 89.0f;
    if (pitch < -89.0f)
        pitch = -89.0f;

    glm::vec3 front;
    front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    front.y = sin(glm::radians(pitch));
    front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    cameraFront = glm::normalize(front);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    if (fov >= 1.0f && fov <= 45.0f)
        fov -= yoffset;
    if (fov <= 1.0f)
        fov = 1.0f;
    if (fov >= 45.0f)
        fov = 45.0f;
}

void drawjoint(float* root_p, JOINT* joint, MOTION* motionData, int frame_starts_index, Shader ourShader, int n_channels, int n_frames, int* j )
{
    
    // if ( *j >= (69) )
    // //if ( *j >= (n_frames ) )
    // {
    //     *j = -1;
    //     root_p[3] = root_p[0] + root_p[3]-root_p[6];
    //     root_p[4] = root_p[1] + root_p[4]-root_p[7];
    //     root_p[5] = root_p[2] + root_p[5]-root_p[8];
    //     std::cout<<"fin"<<root_p[3]<<std::endl;
    // }
    //else
    {
        std::cout<<*j<<std::endl;
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
                if (*j == 0)
                {
                    root_p[6] = value;
                }
                root_p[0] = value;
                joint->matrix = glm::translate(joint->matrix, glm::vec3(value, 0, 0));
                joint->matrix = glm::translate(joint->matrix, glm::vec3(root_p[3]-root_p[6], 0, 0));
            }
            if( channel & Yposition )
            {
                if (*j == 0)
                {
                    root_p[7] = value;
                }
                joint->matrix = glm::translate(joint->matrix, glm::vec3(0, value, 0));
                root_p[1] = value;
            }
            if( channel & Zposition )
            {
                if (*j == 0)
                {
                    root_p[8] = value;
                }
                root_p[2] = value;
                joint->matrix = glm::translate(joint->matrix, glm::vec3(0, 0, value));
                joint->matrix = glm::translate(joint->matrix, glm::vec3(0, 0, root_p[5]-root_p[8]));
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
        glm::mat4 scale = glm::scale(glm::mat4(1.0f), glm::vec3(joint->offset.x+1,joint->offset.y+1,joint->offset.z+1));
        glm::mat4 draw = glm::mat4(1.0f);
        if( joint->parent != NULL )
        {
            draw = glm::translate(joint->parent->matrix, glm::vec3(0.5*joint->offset.x,0.5*joint->offset.y,0.5*joint->offset.z));
        }

        ourShader.setMat4("model", draw * scale);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        for(auto& child : joint->children)
            drawjoint(root_p,child, motionData, frame_starts_index, ourShader, n_channels, n_frames, j);
    }   
    
}
int opt( JOINT* joint, MOTION* motionData, int frame_starts_index, int n_frames, 
JOINT* refer, MOTION* motion_ref, float cost, int j)
{
    {
        int start_index = frame_starts_index + joint->channel_start;
        int ref_index =  refer->channel_start;
                                     
        for(int i = 0; i < joint->num_channels; i++)
        {
            const short& channel = joint->channels_order[i];

            float value = motionData->data[start_index + i];
            float value_ref = motion_ref->data[ref_index + i];
            if( channel & Xposition )
            {
            }
            if( channel & Yposition )
            {
                cost = cost + (value-value_ref) * (value-value_ref) ;
            }
            if( channel & Zposition )
            {
            }

            if( channel & Xrotation )
            {
                cost = cost + (value-value_ref) * (value-value_ref) ;
            }
            if( channel & Yrotation )
            {
                if (i != 0)
                {
                    cost = cost + (value-value_ref) * (value-value_ref) ;
                }
                // refer->matrix = glm::rotate(refer->matrix, float(3.141592*(value_ref)/180), glm::vec3(0, 1, 0));
                // joint->matrix = glm::rotate(joint->matrix, float(3.141592*(value)/180), glm::vec3(0, 1, 0));
            }
            if( channel & Zrotation )
            {
                cost = cost + (value-value_ref) * (value-value_ref) ;
                // refer->matrix = glm::rotate(refer->matrix, float(3.141592*(value_ref)/180), glm::vec3(0, 0, 1));
                // joint->matrix = glm::rotate(joint->matrix, float(3.141592*(value)/180), glm::vec3(0, 0, 1));
            }
        }

    //     if( joint->parent != NULL )
    //             joint->matrix = joint->parent->matrix * joint->matrix;        
    //     if( joint->parent != NULL )
    //             joint->matrix = joint->parent->matrix * joint->matrix;           
        for(auto& child : joint->children)
            opt( child, motionData, frame_starts_index, n_frames, refer, motion_ref, cost, j);
    }
    return cost;

}


