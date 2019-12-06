#ifndef BVH
#define BVH
#include <string>
#include <fstream>
#include <vector>
#include <set>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <sstream>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#define Xposition 0x01
#define Yposition 0x02
#define Zposition 0x04
#define Zrotation 0x10
#define Xrotation 0x20
#define Yrotation 0x40
typedef struct
{
    float x, y, z;
} OFFSET;

//typedef struct JOINT JOINT;

struct JOINT
{
    const char* name = NULL;
    JOINT* parent = NULL;
    OFFSET offset;
    unsigned int num_channels = 0;
    short* channels_order = NULL;
    std::vector<JOINT*> children;
    glm::mat4 matrix;
    unsigned int channel_start = 0;
};

typedef struct
{
    JOINT* rootJoint;
    int num_channels;
} HIERARCHY;

typedef struct
{
    unsigned int num_frames;           
    unsigned int num_motion_channels = 0; 
    float* data = NULL;                
    unsigned* joint_channel_offsets;   
} MOTION;

class Bvh
{
    JOINT* loadJoint(std::istream& stream, JOINT* parent = NULL);
    void loadHierarchy(std::istream& stream);
    void loadMotion(std::istream& stream);
    

public:
    Bvh();
    ~Bvh();
    void load(const std::string& filename);
    // const JOINT* getRootJoint() const { return rootJoint; }
    JOINT* getRootJoint() const { return rootJoint; }
    MOTION motionData;
    //void testOutput() const;
    //void printJoint(const JOINT* const joint) const;
    //void moveTo(unsigned frame);
    //const JOINT* getRootJoint() const { return rootJoint; }
    //unsigned getNumFrames() const { return motionData.num_frames; }
    //void moveJoint(JOINT* joint, MOTION* motionData, int frame_starts_index);
private:
    JOINT* rootJoint;
};

#endif