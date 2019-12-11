#include "bvh.h"
#include <glm/gtc/type_ptr.hpp>
#include <stdio.h>

static inline std::string &ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
        return s;
}

static inline std::string &rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
        return s;
}

static inline std::string &trim(std::string &s) {
        return ltrim(rtrim(s));
}

Bvh::Bvh()
    : rootJoint(NULL)
{
    motionData.data = 0;
}

void deleteJoint(JOINT* joint)
{
    if( joint == NULL )
        return;
    for(JOINT* child : joint->children)
        deleteJoint(child);
    if( joint->channels_order != NULL )
        delete joint->channels_order;
    delete joint;
}

Bvh::~Bvh()
{
    deleteJoint(rootJoint);
    if( motionData.data != NULL )
        delete[] motionData.data;
}

void Bvh::load(const std::string& filename)
{
    std::fstream file;
    file.open(filename.c_str(), std::ios_base::in);
    //std::cout <<"start"<<std::endl;
    if( file.is_open() )
    {
        std::string line;
        while( file.good() )
        {
            file >> line;
            if( trim(line) == "HIERARCHY" )
                loadHierarchy(file);
            break;
        }
        file.close();
    }

    // Create Collision body
    rootJoint->create_link();
}
void Bvh::loadHierarchy(std::istream& stream)
{
    std::string tmp;
    while( stream.good() )
    {
        stream >> tmp;
        if( trim(tmp) == "ROOT" )
            rootJoint = loadJoint(stream);
        else if( trim(tmp) == "MOTION" )
            loadMotion(stream);
    }
}
void Bvh::loadMotion(std::istream& stream)
{
    std::string tmp;
    while( stream.good() )
    {
        stream >> tmp;
        if( trim(tmp) == "Frames:" )
        {
            stream >> motionData.num_frames;
        }
        else if( trim(tmp) == "Frame" )
        {
            float frame_time;
            stream >> tmp >> frame_time;
            int num_frames   = motionData.num_frames;
            int num_channels = motionData.num_motion_channels;
            motionData.data = new float[num_frames * num_channels];
            for( int frame = 0; frame < num_frames; frame++ )
            {
                for( int channel = 0; channel < num_channels; channel++)
                {
                    float x;
                    std::stringstream ss;
                    stream >> tmp;
                    ss << tmp;
                    ss >> x;
                    int index = frame * num_channels + channel;
                    motionData.data[index] = x;     
                }
            }
        }
    }
}

JOINT* Bvh::loadJoint(std::istream& stream, JOINT* parent)
{
    JOINT* joint = new JOINT;
    joint->parent = parent;

    std::string* name = new std::string;
    stream >> *name;
    joint->name = name->c_str();

    std::string tmp;
    joint->matrix = glm::mat4(1.0);

    static int _channel_start = 0;
    unsigned channel_order_index = 0;
    while( stream.good() )
    {
        stream >> tmp;
        tmp = trim(tmp);

        char c = tmp.at(0);
        if( c == 'X' || c == 'Y' || c == 'Z' )
        {
            if( tmp == "Xposition" )
            {
                joint->channels_order[channel_order_index++] = Xposition;
            }
            if( tmp == "Yposition" )
            {
                joint->channels_order[channel_order_index++] = Yposition;
            }
            if( tmp == "Zposition" )
            {
                joint->channels_order[channel_order_index++] = Zposition;
            }

            if( tmp == "Xrotation" )
            {
                joint->channels_order[channel_order_index++] = Xrotation;
            }
            if( tmp == "Yrotation" )
            {
                joint->channels_order[channel_order_index++] = Yrotation;
            }
            if( tmp == "Zrotation" )
            {
                joint->channels_order[channel_order_index++] = Zrotation;
            }
        }

        if( tmp == "OFFSET" )
        {
            stream  >> joint->offset.x
                    >> joint->offset.y
                    >> joint->offset.z;
        }
        else if( tmp == "CHANNELS" )
        {
            stream >> joint->num_channels;
            motionData.num_motion_channels += joint->num_channels;
            joint->channel_start = _channel_start;
            _channel_start += joint->num_channels;
            joint->channels_order = new short[joint->num_channels];
        }
        else if( tmp == "JOINT" )
        {
            JOINT* tmp_joint = loadJoint(stream, joint);
            tmp_joint->parent = joint;
            joint->children.push_back(tmp_joint);
        }
        else if( tmp == "End" )
        {
            stream >> tmp >> tmp;

            JOINT* tmp_joint = new JOINT;

            tmp_joint->parent = joint;
            tmp_joint->num_channels = 0;
            tmp_joint->name = "EndSite";
            joint->children.push_back(tmp_joint);

            stream >> tmp;
            if( tmp == "OFFSET" )
                stream >> tmp_joint->offset.x
                       >> tmp_joint->offset.y
                       >> tmp_joint->offset.z;
            stream >> tmp;
        }
        else if( tmp == "}" )
            return joint;

    }
}

void Bvh::draw_bvh(int frame_num){
    int frame_start_num = frame_num * motionData.num_motion_channels;
    rootJoint->draw_joint(frame_start_num, &motionData);
}

void JOINT::draw_joint(int frame_starts_index, MOTION* motion){
    int start_index = frame_starts_index + this->channel_start;
    this->matrix = glm::translate(glm::mat4(1.0),
                                   glm::vec3(this->offset.x,
                                             this->offset.y,
                                             this->offset.z));
    for(int i = 0; i < this->num_channels; i++)
    {
        const short& channel = this->channels_order[i];

        float value = motion->data[start_index + i];
        if( channel & Xposition )
        {
            this->matrix = glm::translate(this->matrix, glm::vec3(value, 0, 0));
        }
        if( channel & Yposition )
        {
            this->matrix = glm::translate(this->matrix, glm::vec3(0, value, 0));
        }
        if( channel & Zposition )
        {
            this->matrix = glm::translate(this->matrix, glm::vec3(0, 0, value));
        }

        if( channel & Xrotation )
        {
            this->matrix = glm::rotate(this->matrix, float(3.141592*(value)/180), glm::vec3(1, 0, 0));
        }
        if( channel & Yrotation )
        {
            this->matrix = glm::rotate(this->matrix, float(3.141592*(value)/180), glm::vec3(0, 1, 0));
        }
        if( channel & Zrotation )
        {
            this->matrix = glm::rotate(this->matrix, float(3.141592*(value)/180), glm::vec3(0, 0, 1));
        }
    }

    if( this->parent != NULL )
        this->matrix = this->parent->matrix * this->matrix;
    
    if(this->parent !=NULL){
        glm::mat4 trans = this->parent->matrix;

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();

        // For Debug
        glPushMatrix();
        glMultMatrixf((float*)glm::value_ptr(trans));
        glMultMatrixf((float*)glm::value_ptr(glm::scale(glm::mat4(1.0f), glm::vec3(10.0f,10.0f,10.0f))));
        draw_axes();
        glPopMatrix();

        //Draw sth
        // this->setTransform((double*)glm::value_ptr(this->matrix));
        // this->draw();

        // Link

        trans = glm::translate(trans, glm::vec3(this->offset.x/2, this->offset.y/2, this->offset.z/2));

        glMultMatrixf((float*)glm::value_ptr(trans));

        if(this->offset.x/2 != 0){
            //this->setSize(this->offset.x/2, 2, 2);
            this->setTransform((float*)glm::value_ptr(trans));
            this->draw();
            //draw_box(btVector3(this->offset.x/2, 2, 2));
        } else if(this->offset.y/2 != 0){
            //this->setSize(2, this->offset.y/2, 2);
            this->setTransform((float*)glm::value_ptr(trans));
            this->draw();
            //draw_box(btVector3(2, this->offset.y/2, 2));
        } else if(this->offset.z/2 != 0){
            //this->setSize(2, 2, this->offset.z/2);
            this->setTransform((float*)glm::value_ptr(trans));
            this->draw();
            //draw_box(btVector3(2, 2, this->offset.z/2));
        }
        glPopMatrix();
    }

    for(auto& child : this->children)
        child->draw_joint(frame_starts_index, motion);

    //glPopMatrix();
    // ourShader.setMat4("model", joint->matrix);
    // glDrawArrays(GL_TRIANGLES, 0, 36);
}

void JOINT::create_link(){
    printf("creating...\n");
    if(this->parent){
        //glm::vec4 parent_world_pos = this->parent->matrix 
        glm::mat4 trans = glm::mat4(1.0f);
        trans = glm::translate(glm::mat4(1.0f), glm::vec3(this->offset.x/2, this->offset.y/2, this->offset.z/2));

        if(this->offset.x/2 != 0){
            this->createCollisionObject((float*)glm::value_ptr(trans),this->offset.x/2, 2, 2);
        } else if(this->offset.y/2 != 0){
            this->createCollisionObject((float*)glm::value_ptr(trans), 2, this->offset.y/2, 2);
        } else if(this->offset.z/2 != 0){
            this->createCollisionObject((float*)glm::value_ptr(trans), 2, 2, this->offset.z/2);
        }
    }

    for(auto& child : this->children)
        child->create_link();
}