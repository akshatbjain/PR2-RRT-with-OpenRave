// include guard
#ifndef __RRT_H_INCLUDED__
#define __RRT_H_INCLUDED__

// If a class A in this header file uses a class B, then class B is one of class A's dependencies. Whether it can be forward declared or needs to be included depends on how B is used within A:
// - do nothing if: A makes no references at all to B
// - do nothing if: The only reference to B is in a friend declaration
// - forward declare B if: A contains a B pointer or reference: B* myb;
// - forward declare B if: one or more functions has a B object/pointer/reference as a parementer, or as a return type: B MyFunction(B myb);
// - #include "b.h" if: B is a parent class of A
// - #include "b.h" if: A contains a B object: B myb;

// forward declared dependencies
//

// included dependencies
#include <vector>
#include <cstring> // Includes NULL
#include <math.h>
#include <openrave/plugin.h>
#include <iostream>

// Classes defined in this header file

class RRTNode
{
    std::vector<double> _configuration;
    RRTNode* parent_node;

public:
    // Constructors
    RRTNode();
    RRTNode(std::vector<double> configuration);
    RRTNode(std::vector<double> configuration, RRTNode* parent);

    // Destructor
    ~RRTNode();

    //Functions
    void setParent(RRTNode* parent);
    RRTNode *getParent();
    std::vector<double> getConfiguration();
    void setConfiguration(std::vector<double> config);

};

class NodeTree
{
    std::vector<RRTNode> _nodes;

public:
    // Constructor
    NodeTree();

    // Destructor
    ~NodeTree();

    //Functions
    void addNode(RRTNode node);
    void deleteNode(int node_index);
    std::vector<RRTNode> getNodes();
    RRTNode getLastNode();
    //std::vector<RRTNode> path();

};

int parent_count = 0;

bool biDirectional;
std::vector<double> start_config, goal_config;
double goal_bias, step_size, joint_weights[7];
std::vector<double> lower_joint_limits, upper_joint_limits;
bool reached_goal;

double euclidean_distance(std::vector<double> A, std::vector<double> B);
RRTNode find_nearest_neighbor(NodeTree tree, std::vector<double> q_random);
std::vector<double> random_sample();
bool check_collision(std::vector<double> config, OpenRAVE::EnvironmentBasePtr env_pointer, OpenRAVE::RobotBasePtr robot_pointer);
void init_Tree();
int extend(NodeTree tree, std::vector<double> q, OpenRAVE::EnvironmentBasePtr env_pointer, OpenRAVE::RobotBasePtr robot_pointer);
bool RRTConnect(OpenRAVE::EnvironmentBasePtr env_pointer);
void print_config(std::vector<double> config);

std::vector<RRTNode *> getPath(NodeTree* tree);

#endif
