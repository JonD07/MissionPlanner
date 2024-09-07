/*
 * NodeGenerator.h
 *
 * Created by:	Jonathan Diller
 * On: 			Sep 7, 2024
 *
 * Description: Node generator class, used to create node objects. The
 * node generator reads in a node types file and assigns parameters to
 * the generated node based on these parameters.
 *
 */

#pragma once

#include <map>
#include <sstream>
#include <fstream>

#include "defines.h"
#include "Node.h"
#include "FileReader.h"

#define DEBUG_NODE_GEN	DEBUG || 0


class NodeGenerator {
public:
	NodeGenerator(std::string data_path);
	virtual ~NodeGenerator();

	// Takes in a node string and returns a node. The string is expected to be in this format:
	//  x_1 y_1 z_1 zs_1 q_1 t_1
	Node GenerateNode(int id, std::string& node_string);
private:
	// Path to node data file
	std::string data_fileName;
	// File reader, for handling text file input
	FileReader fileReader;

	// List of node parameters
	std::map<int, NodeParameters> nodeParameters;
};
