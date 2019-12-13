#include "CollisionObject.h"
#include "openglHelper.h"

CollisionObject::CollisionObject(double m_mass) : m_mass(m_mass){
}

void CollisionObject::setTransform(float* m){
    // Double conversion
    double dm[16] = {0.0};
    for(int i=0;i<16;i++){
        dm[i] = m[i];
    }
    this->setTransform(dm);
}
void CollisionObject::setTransform(double* m){
    btTransform m_trans;
    m_trans.setFromOpenGLMatrix(m);
    if(m_obj) m_obj->setWorldTransform(m_trans);
}

void CollisionObject::setTransform(Eigen::Affine3d m){
    this->setTransform(m.data());
}
