#include "RRT.h"

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
