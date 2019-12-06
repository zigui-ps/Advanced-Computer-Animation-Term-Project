#ifndef CONTROLS_HPP
#define CONTROLS_HPP

namespace ViewControl{
    void computeMatricesFromInputs();
    glm::mat4 getViewMatrix();
    glm::mat4 getProjectionMatrix();
    void setInitCameraPosition(const glm::vec3 &init_pos);
    void setLookHere(const glm::vec3 &look_here);
    void setCameraDirection(const glm::vec3 &camera_direction);
    float getalpha();
    float getlightPower();
    glm::vec3 getCameraOffset();
}

#endif