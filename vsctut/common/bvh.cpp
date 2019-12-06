#include <bvh.h>


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

