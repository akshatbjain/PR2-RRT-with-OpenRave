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

// Classes defined in this header file

class RRTNode
{
    std::vector<float> _configuration;
    RRTNode* parent_node;

public:
    // Constructors
    RRTNode();
    RRTNode(std::vector<float> configuration);
    RRTNode(std::vector<float> configuration, RRTNode* parent);

    // Destructor
    ~RRTNode();

    //Functions
    void setParent(RRTNode* parent);
    RRTNode* getParent();
    std::vector<float> getConfiguration();

};

class NodeTree
{
    std::vector<RRTNode> _nodes;
    std::vector<float> DOFWeights;

public:
    // Constructor
    NodeTree();

    // Destructor
    ~NodeTree();

    //Functions
    void addNode(RRTNode node);
    void deleteNode(int node_index);
    std::vector<RRTNode> getNodes();
    std::vector<RRTNode> path();
    float euclidean_distance(std::vector<float> A, std::vector<float> B);
    RRTNode find_nearest_neighbor(std::vector<RRTNode> tree, std::vector<float> q_random);

};

std::vector<float> lower_joint_limits, upper_joint_limits;
void getJointLimits(std::vector<float> lower, std::vector<float> upper);
std::vector<float> random_sample();



#endif
