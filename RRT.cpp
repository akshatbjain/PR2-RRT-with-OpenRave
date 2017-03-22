#include "RRT.h"

// ******************************************************************************* //
// RRTNode
// ******************************************************************************* //

RRTNode::RRTNode()
{
    parent_node = NULL;
}

RRTNode::RRTNode(std::vector<double> configuration)
{
    _configuration = configuration;
    parent_node = NULL;
}

RRTNode::RRTNode(std::vector<double> configuration, RRTNode *parent)
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

std::vector<double> RRTNode::getConfiguration()
{
    return _configuration;
}

void RRTNode::setConfiguration(std::vector<double> config)
{
    _configuration = config;
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

RRTNode NodeTree::getLastNode()
{
    return _nodes.back();
}

std::vector<RRTNode> NodeTree::path()
{

}

double euclidean_distance(std::vector<double> A, std::vector<double> B)
{
    double sum = 0.0;
    for(int i = 0; i<7; i++)
    {
        sum = sum + pow(joint_weights[i]*(A[i] - B[i]),2);
    }

    return sqrt(sum);
}

RRTNode find_nearest_neighbor(NodeTree tree, std::vector<double> q_random)
{
    double distance, minimum_distance = -1;
    int node_index;
    std::vector<RRTNode> nodes = tree.getNodes();

    for(int i = 0; i<int(nodes.size()); i++)
    {
        distance = euclidean_distance(nodes[i].getConfiguration(), q_random);
        if(distance < minimum_distance || minimum_distance == -1)
        {
            minimum_distance = distance;
            node_index = i;
        }
    }

    return nodes[node_index];
}

std::vector<double> random_sample()
{
    std::vector<double> q_rand;
    srand(time(NULL));
    for(int i=0; i<7; i++)
    {
        q_rand.push_back(lower_joint_limits[i] + static_cast<double> (rand()) / (static_cast<double> (RAND_MAX/(upper_joint_limits[i] - lower_joint_limits[i]))));
    }
    return q_rand;
}

bool check_collision(std::vector<double> config)
{
    robot_pointer->SetActiveDOFValues(config);
    return env_pointer->CheckCollision(robot_pointer) || robot_pointer->CheckSelfCollision();
}

void init_Tree(NodeTree tree)
{
    tree.addNode(start_config);
}

int extend(NodeTree tree, std::vector<double> q)
{
    RRTNode q_near, q_new, temp_node;
    std::vector<double> temp, temp1;
    double distance;

    q_near.setConfiguration(find_nearest_neighbor(tree, q).getConfiguration());
    temp1 = q_near.getConfiguration();
    distance = euclidean_distance(q, q_near.getConfiguration());

    if(distance > step_size)
    {
        for(int i = 0; i<7; i++)
        {
            temp.push_back(step_size*(q[i]-temp1[i])/euclidean_distance(q, temp1));
        }
        q_new.setConfiguration(temp);
        if(q_new.getConfiguration() == q)
        {
            reached_goal = 1;
            return 0; // Reached
        }
        if(!check_collision(q_new.getConfiguration()))
        {
            tree.addNode(q_new);
            temp_node = tree.getLastNode();
            temp_node.setParent(q_near.getParent());
            return 1; // Extend
        }

    }
    return 2; // Trapped

}

void RRTConnect()
{
    robot_pointer = env_pointer->GetRobot("PR2");

    NodeTree tree;
    init_Tree(tree);

    reached_goal = 0;

    std::vector<double> q_rand;

    while(reached_goal == 0)
    {
        q_rand = random_sample();
        while(extend(tree,q_rand) != 2)
        {

        }

    }
}
