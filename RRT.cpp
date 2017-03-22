#include "RRT.h"

// ******************************************************************************* //
// RRTNode
// ******************************************************************************* //

RRTNode::RRTNode()
{
    parent_node = 0;
}

RRTNode::RRTNode(std::vector<double> configuration)
{
    _configuration = configuration;
    parent_node = 0;
}

RRTNode::RRTNode(std::vector<double> configuration, int parent)
{
    _configuration = configuration;
    parent_node = parent;
}

RRTNode::~RRTNode()
{
    _configuration.clear();
    parent_node = 0;
}

void RRTNode::setParent(int parent)
{
    parent_node = parent;
}

int RRTNode::getParent()
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
    if (rand() / double(RAND_MAX) < goal_bias)
    {
        return goal_config;
    }
    for(int i=0; i<7; i++)
    {
        q_rand.push_back(lower_joint_limits[i] + ((static_cast<double> (rand()) / static_cast<double> (RAND_MAX)) * (upper_joint_limits[i] - lower_joint_limits[i])));
    }
    return q_rand;
}

bool check_collision(std::vector<double> config, OpenRAVE::EnvironmentBasePtr env_pointer, OpenRAVE::RobotBasePtr robot_pointer)
{
    robot_pointer->SetActiveDOFValues(config);
    return env_pointer->CheckCollision(robot_pointer) || robot_pointer->CheckSelfCollision();
}

void init_Tree(NodeTree tree)
{
    tree.addNode(start_config);
}

int extend(NodeTree* tree, std::vector<double> q, OpenRAVE::EnvironmentBasePtr env_pointer, OpenRAVE::RobotBasePtr robot_pointer)
{
    RRTNode q_near, q_new, temp_node;
    std::vector<double> temp, temp1;
    double distance;

    q_near.setConfiguration(find_nearest_neighbor(*tree, q).getConfiguration());
    temp1 = q_near.getConfiguration();

    distance = euclidean_distance(q, q_near.getConfiguration());

    if(distance > step_size)
    {
        for(int i = 0; i<7; i++)
        {
            temp.push_back(temp1[i] + (step_size*(q[i]-temp1[i])/euclidean_distance(q, temp1)));
        }

        if(!check_collision(temp, env_pointer, robot_pointer))
        {
            print_config(temp);
            q_new.setConfiguration(temp);
            q_new.setParent(parent_count++);
            tree->addNode(q_new);
            std::cout<<"\nNew node added DG";

            if(temp == q)
            {
                if(q == goal_config)
                    reached_goal = 1;
                return 0; // Reached
            }
            return 1; // Extend
        }

    }
    else
    {
        if(!check_collision(q, env_pointer, robot_pointer))
        {
            print_config(q);
            q_new.setConfiguration(q);
            q_new.setParent(parent_count++);
            tree->addNode(q_new);
            std::cout<<"\nNew node added DL";
            if(q == goal_config)
                reached_goal = 1;
            return 0;

        }
    }
    return 2; // Trapped

}

bool RRTConnect(OpenRAVE::EnvironmentBasePtr env_pointer)
{
    OpenRAVE::RobotBasePtr robot_pointer = env_pointer->GetRobot("PR2");
    NodeTree tree, *treeptr = &tree;
    RRTNode start (start_config);

    int threshold = 10000, i = 0;

    tree.addNode(start);

    reached_goal = 0;

    std::vector<double> q_rand;

    srand(time(0));

    while((reached_goal == 0) && (i < threshold))
    {
        q_rand = random_sample();
        std::cout << "\nNew sample. Loop number: "<< i;
        int ext = 1;
        while(ext == 1)
        {
            ext = extend(treeptr,q_rand, env_pointer, robot_pointer);
        }
        std::cout << std::endl;
        i++;
        if(reached_goal == 1)
        {
            std::cout<< "Reached";
            break;
        }
    }

    std::cout << tree.getNodes().size();
    print_config(tree.getLastNode().getConfiguration());
    std::vector<double> path = getPath(tree);
    return true;

}

void print_config(std::vector<double> config)
{
    std::cout << "\n[";
    for(int i = 0; i<7; i++)
    {
        std::cout << config[i] << ", ";
    }
    std::cout <<"]";
}

std::vector<double> getPath(NodeTree tree)
{
    std::vector<RRTNode> nodes = tree.getNodes();
    std::vector<double> path;
    for(int i = 0; i<nodes.size(); i++)
    {

    }
    return path;
}
