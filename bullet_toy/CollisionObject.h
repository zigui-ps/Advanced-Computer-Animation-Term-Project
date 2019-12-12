#ifndef __COLLISION_OBJECT_H__
#define __COLLISION_OBJECT_H__

#include "bulletHelper.h"

class CollisionObject{
public:
    CollisionObject();

    void createCollisionObject(float world_x, float world_y, float world_z, float half_width, float half_height, float half_depth);
    void createCollisionObject(float *m, float half_width, float half_height, float half_depth);
    void setTransform(float* m);
    void setTransform(btTransform m);
    void setSize(float half_width, float half_height, float half_depth);
    void draw();

protected:
    btTransform m_trans;
    
    float m_width, m_height, m_depth;
    btScalar m_mass;

    btRigidBody* m_obj;
};



#endif //__COLLISION_OBJECT_H__