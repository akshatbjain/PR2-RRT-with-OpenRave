#include "RRT.h"

// ******************************************************************************* //
// RRTNode
// ******************************************************************************* //

RRTNode::RRTNode()
{
    parent_node = NULL;
}

RRTNode::RRTNode(std::vector<float> configuration)
{
    _configuration = configuration;
    parent_node = NULL;
}

RRTNode::RRTNode(std::vector<float> configuration, RRTNode *parent)
{
    _configuration = configuration;
    parent_node = parent;
}

RRTNode::~RRTNode()
{
    _configuration.clear();
    parent_node = NULL;
}

void RRTNode::setParent(RRTNode* parent)
{
    parent_node = parent;
}

RRTNode* RRTNode::getParent()
{
    return parent_node;
}

std::vector<float> RRTNode::getConfiguration()
{
    return _configuration;
}

// ******************************************************************************* //
// NodeTree
// ******************************************************************************* //
NodeTree::NodeTree()
{

}

NodeTree::~NodeTree()
{

}

void NodeTree::addNode(RRTNode node)
{
    _nodes.push_back(node);
}

void NodeTree::deleteNode(int node_index)
{
    _nodes.erase(_nodes.begin() + node_index);
}

std::vector<RRTNode> NodeTree::getNodes()
{
    return _nodes;
}

std::vector<RRTNode> NodeTree::path()
{

}

float NodeTree::euclidean_distance(std::vector<float> A, std::vector<float> B)
{
    float sum = 0.0;
    for(int i = 0; i<7; i++)
    {
        sum = sum + pow(DOFWeights[i]*(A[i] - B[i]),2);
    }

    return sqrt(sum);
}

RRTNode NodeTree::find_nearest_neighbor(std::vector<RRTNode> tree, std::vector<float> q_random)
{
    float distance, minimum_distance = -1;
    int node_index;

    for(int i = 0; i<int(tree.size()); i++)
    {
        distance = euclidean_distance(tree[i].getConfiguration(), q_random);
        if(distance < minimum_distance || minimum_distance == -1)
        {
            minimum_distance = distance;
            node_index = i;
        }
    }

    return tree[node_index];
}

void getJointLimits(std::vector<float> lower, std::vector<float> upper)
{
    lower_joint_limits = lower;
    upper_joint_limits = upper;
}

std::vector<float> random_sample()
{
    std::vector<float> q_rand;
    for(int i=0; i<7; i++)
    {
        q_rand.push_back(lower_joint_limits[i] + static_cast<float> (rand()) / (static_cast<float> (RAND_MAX/(upper_joint_limits[i] - lower_joint_limits[i]))));
    }
    return q_rand;
}
