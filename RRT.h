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
    std::vector<RRTNode*> _nodes;

public:
    void addNodes(std::vector<float> node_values, RRTNode* parents);
    void deleteNodes();
    std::vector<RRTNode*> getNodes();
    std::vector<RRTNode*> path();
    RRTNode* find_nearest_neighbor(std::vector<float> tree);
};

#endif
