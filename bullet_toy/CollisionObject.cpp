#include "CollisionObject.h"

CollisionObject::CollisionObject(){

}

void CollisionObject::createCollisionObject(float world_x, float world_y, float world_z, float half_width, float half_height, float half_depth){
    setSize(half_width, half_height, half_depth);
    btBoxShape *box = new btBoxShape(btVector3(btScalar(5.), btScalar(5.), btScalar(5.)));
    
    m_trans.setIdentity();

    m_mass = 5.f;
    
    m_trans.setOrigin(btVector3(world_x, world_y, world_z));
    m_obj = createRigidBody(m_mass, m_trans, box, true);
	m_obj->setWorldTransform(m_trans);

    printf("created\n");
}


void CollisionObject::setTransform(double* m){
    m_trans.setFromOpenGLMatrix(m);

    if(m_obj) m_obj->setWorldTransform(m_trans);
}

void CollisionObject::setTransform(btTransform m){
    m_trans = m;

    if(m_obj) m_obj->setWorldTransform(m_trans);
}


void CollisionObject::setSize(float half_width, float half_height, float half_depth){
    m_width = half_width;
    m_height = half_height;
    m_depth = half_depth;
}


void CollisionObject::draw(){
    if(m_obj){
        btTransform trans = m_obj->getWorldTransform();
        printf("world pos object  = %f,%f,%f\n", trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());

    } 

    draw_box(btVector3(m_width, m_height, m_depth));
}
