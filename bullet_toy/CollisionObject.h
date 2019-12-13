#ifndef __COLLISION_OBJECT_H__
#define __COLLISION_OBJECT_H__

#include "bulletHelper.h"
#include<Eigen/Dense>

class CollisionObject{
public:
    CollisionObject(double m_mass);
    virtual void setTransform(Eigen::Affine3d m);
//protected:
    btScalar m_mass;
    btRigidBody* m_obj;
private:
    virtual void setTransform(double* m);
    virtual void setTransform(float* m);
};

#endif //__COLLISION_OBJECT_H__
