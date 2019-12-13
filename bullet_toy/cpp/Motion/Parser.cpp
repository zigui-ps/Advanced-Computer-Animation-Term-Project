#include<tinyxml.h>
#include<Eigen/Dense>
#include<iostream>
#include<fstream>
#include<sstream>
#include<string>
#include<vector>
#include "Motion/Parser.h"

const double PI = acos(-1);
extern double scale;

using BVHNode = Parser::BVHNode;

std::string v3toString(Eigen::Vector3d vec){
	std::stringstream ss; ss << vec[0] << " " << vec[1] << " " << vec[2] << " ";
	return ss.str();
}

void Parser::BVHNode::readFile(std::ifstream &file, SkeletonPosePtr pose){
	if(channelList.size() == 0) return;
	Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
	for(std::string channel : channelList){
		double value;
		file >> value;
		if(channel.substr(1) == "rotation"){
			Eigen::Vector3d axis;
			switch(channel[0]){
				case 'X': axis = Eigen::Vector3d::UnitX(); break;
				case 'Y': axis = Eigen::Vector3d::UnitY(); break;
				case 'Z': axis = Eigen::Vector3d::UnitZ(); break;
			}
			rotation = rotation * Eigen::AngleAxisd(value * PI / 180, axis);
		}
		else if(channel.substr(1) == "position"){
			int idx;
			switch(channel[0]){
				case 'X': idx = 0; break;
				case 'Y': idx = 1; break;
				case 'Z': idx = 2; break;
			}
			pose->location[idx] = value / 100.0 * scale;
		}
	}
	pose->joint.push_back(rotation);
}

static void BVHToXML(BVHNode* node, TiXmlElement* xml){
	TiXmlElement* sphere = new TiXmlElement("Sphere");
	sphere->SetAttribute("radius", "0.05");
	sphere->SetAttribute("translate", "0 0 0");

	xml->LinkEndChild(sphere);

	for(BVHNode* c : node->child){
		TiXmlElement* child = new TiXmlElement("Joint");
		xml->LinkEndChild(child);
		child->SetAttribute("name", c->name);
		child->SetAttribute("translate", v3toString(Eigen::Vector3d(c->offset)));

		BVHToXML(c, child);
	}
}

static TiXmlDocument* BVHToFile(BVHNode* root){
	TiXmlDocument* doc = new TiXmlDocument("character/gen.xml");
	std::vector<TiXmlElement*> list;

	TiXmlElement* skel = new TiXmlElement("Skeleton");
	doc->LinkEndChild(skel);
	skel->SetAttribute("name", "Humanoid");

	TiXmlElement* xml = new TiXmlElement("Joint");
	skel->LinkEndChild(xml);
	xml->SetAttribute("name", root->name);

	BVHToXML(root, xml);

	doc->SaveFile();

	return doc;
}

static BVHNode* endParser(std::ifstream &file){
	BVHNode* current_node = new BVHNode();
	std::string tmp;
	file >> current_node->name;
	file >> tmp; assert(tmp == "{");
	file >> tmp; assert(tmp == "OFFSET");
	for(int i = 0; i < 3; i++){
		file >> current_node->offset[i];
		current_node->offset[i] /= 100.0;
	}
	file >> tmp; assert(tmp == "}");

	return current_node;
}

static BVHNode* nodeParser(std::ifstream &file, int &channels, std::vector<BVHNode*> &nodeList)
{
	std::string command, tmp;
	int sz;
	BVHNode* current_node = new BVHNode();
	nodeList.push_back(current_node);

	file >> current_node->name;
	file >> tmp; assert(tmp == "{");

	while(1){
		file >> command;
		if(command == "OFFSET"){
			for(int i = 0; i < 3; i++){
				file >> current_node->offset[i];
				current_node->offset[i] /= 100.0;
			}
		}
		else if(command == "CHANNELS"){
			file >> sz;
			channels += sz;
			for(int i = 0; i < sz; i++){
				file >> tmp;
				current_node->channelList.push_back(tmp);
			}
		}
		else if(command == "JOINT"){
			current_node->child.push_back(nodeParser(file, channels, nodeList));
		}
		else if(command == "End"){
			current_node->child.push_back(endParser(file));
		}
		else if(command == "}") break;
	}
	return current_node;
}

static void motionParser(std::ifstream &file, int channels, std::vector<SkeletonPosePtr> &motion, std::vector<BVHNode*> nodeList)
{
	double time_interval;
	int frames;
	std::string command;

	file >> command; assert(command == "Frames:");
	file >> frames;
	file >> command; assert(command == "Frame");
	file >> command; assert(command == "Time:");
	file >> time_interval; 
	//		fps = 1. / time_interval; printf("fps: %lf\n", fps);

	for(int t = 0; t < frames; t++){
		// Position
		SkeletonPosePtr pose = SkeletonPosePtr(new SkeletonPose());
		int idx = 0;
		for(auto node : nodeList){
			node->readFile(file, pose);
		}
		motion.push_back(pose);
	}
}

std::vector<SkeletonPosePtr> Parser::Parser(std::string filename, SkeletonPtr &skel)
{
	std::ifstream file(filename);
	std::string command, tmp;
	std::vector<SkeletonPosePtr> motion;
	std::vector<BVHNode*> nodeList;

	int channels;
	channels = 0;

	while(1){
		command = ""; file >> command;
		if(command == "HIERARCHY"){
			file >> tmp; assert(tmp == "ROOT");
			BVHNode* root = nodeParser(file, channels, nodeList);
			skel = SkeletonPtr(new Skeleton(*BVHToFile(root)));
		}
		else if(command == "MOTION") motionParser(file, channels, motion, nodeList);
		else break;
	}
	file.close();
	return motion;
}
	
std::vector<std::vector<SkeletonPosePtr>> Parser::Parser(std::vector<std::string> motionFiles, SkeletonPtr &skel){
	std::vector<std::vector<SkeletonPosePtr>> poses;
	for(std::string str : motionFiles) poses.push_back(Parser(str, skel));
	return poses;
}
