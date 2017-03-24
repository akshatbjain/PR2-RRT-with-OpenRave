#include "RRT.h"

// ******************************************************************************* //
// RRTNode
// ******************************************************************************* //

RRTNode::RRTNode()
{

}

RRTNode::RRTNode(std::vector<double> configuration, int parent, int self)
{
    _configuration = configuration;
    parent_node = parent;
    _self = self;
}

RRTNode::~RRTNode()
{
    _configuration.clear();
    parent_node = 0;
    _self = 0;
}

void RRTNode::setParent(int parent)
{
    parent_node = parent;
}

void RRTNode::setSelf(int self)
{
    _self = self;
}

int RRTNode::getParent()
{
    return parent_node;
}

int RRTNode::getSelf()
{
    return _self;
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

// ******************************************************************************* //
// Functions
// ******************************************************************************* //

double euclidean_distance(std::vector<double> A, std::vector<double> B)
{
    double sum = 0.0;
    for(int i = 0; i<7; i++)
    {
        sum = sum + pow((A[i] - B[i]),2);
    }

    return sqrt(sum);
}

double weighted_distance(std::vector<double> A, std::vector<double> B)
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
        distance = weighted_distance(nodes[i].getConfiguration(), q_random);
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

bool in_limits(std::vector<double> a)
{
    for(int i = 0; i<7; i++)
    {
        if((a[i]<lower_joint_limits[i]) || (a[i]>upper_joint_limits[i]))
            return false;
    }
    return true;
}

bool check_collision(std::vector<double> config, OpenRAVE::EnvironmentBasePtr env_pointer, OpenRAVE::RobotBasePtr robot_pointer)
{
    robot_pointer->SetActiveDOFValues(config);
    return env_pointer->CheckCollision(robot_pointer) || robot_pointer->CheckSelfCollision();
}

int extend(NodeTree* tree, std::vector<double> q, OpenRAVE::EnvironmentBasePtr env_pointer, OpenRAVE::RobotBasePtr robot_pointer)
{
    RRTNode q_near, q_new, temp_node;
    std::vector<double> temp, temp1;
    double distance;

    q_near = find_nearest_neighbor(*tree, q);
    temp1 = q_near.getConfiguration();

    distance = weighted_distance(q, temp1);

    if(distance > step_size)
    {
        for(int i = 0; i<7; i++)
        {
            temp.push_back(temp1[i] + (step_size*(q[i]-temp1[i])/distance));
        }

        if((!check_collision(temp, env_pointer, robot_pointer)) && in_limits(temp))
        {
            //print_config(temp);
            ++node_counter;
            q_new.setConfiguration(temp);
            q_new.setParent(q_near.getSelf());
            q_new.setSelf(node_counter);
            tree->addNode(q_new);
            //std::cout<<"\nNew node added DG";

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
        if((!check_collision(q, env_pointer, robot_pointer)) && in_limits(q))
        {
            //print_config(q);
            ++node_counter;
            q_new.setConfiguration(q);
            q_new.setParent(q_near.getSelf());
            q_new.setSelf(node_counter);
            tree->addNode(q_new);
            //std::cout<<"\nNew node added DL";
            if(q == goal_config)
                reached_goal = 1;
            return 0;

        }
    }
    return 2; // Trapped

}

std::vector<std::vector<double> > RRTConnect(OpenRAVE::EnvironmentBasePtr env_pointer)
{
    OpenRAVE::RobotBasePtr robot_pointer = env_pointer->GetRobot("PR2");
    NodeTree tree, *treeptr = &tree;
    RRTNode start (start_config, 0, -1);
    node_counter = 0;
    //int threshold = 10000;
    int i = 0;

    tree.addNode(start);

    reached_goal = 0;

    std::vector<double> q_rand;

    srand(time(0));

    while(reached_goal == 0) // && (i < threshold))
    {
        q_rand = random_sample();
        //std::cout << "\nNew sample. Loop number: "<< i;
        int ext = 1;
        while(ext == 1)
        {
            ext = extend(treeptr,q_rand, env_pointer, robot_pointer);
        }
        //std::cout << std::endl;
        if(i%1000 == 0)
            std::cout << i << "\n";

        i++;
        if(reached_goal == 1)
        {
            std::cout<< "Reached";
            break;
        }
    }

    std::cout << tree.getNodes().size();
    //print_config(tree.getLastNode().getConfiguration());
    std::cout<<"\nGetting path now";
    std::vector<std::vector<double> > path = getPath(treeptr);
    std::cout<<"\nPath successfully found";
    return path;

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

std::vector<std::vector<double> > getPath(NodeTree* tree)
{
    std::vector<RRTNode> nodes = tree->getNodes();
    std::vector<std::vector<double> > path;
    RRTNode temp = tree->getLastNode();
    int current_parent = temp.getParent();
    int size = nodes.size() - 1, i = 0;
    path.push_back(temp.getConfiguration());
    //print_config(path[i]);
    ++i;
    --size;

    while(size!=-1)
    {
        temp = nodes[size];
        if(temp.getSelf() == current_parent)
        {
            path.push_back(temp.getConfiguration());
            //print_config(path[i]);
            //++i;
            current_parent = temp.getParent();
        }
        --size;
        //std::cout<<size<<"\n";
        if(temp.getSelf() == 0)
        {
            //std::cout<<"\nBreaking now";
            break;
        }
    }

//    for(int i = node_counter-2; i>=0; i--)
//    {
//        temp = nodes[i];
//        if(temp.getParent() == parent_count - 1)
//        {
//            path.push_back(temp.getConfiguration());
//            parent_count--;
//            print_config(path[j]);
//            j++;
//        }

//    }
    std::cout<<"\nReversing now";
    reverse(path.begin(), path.end());
    std::cout<<"\nReversing done";
    return path;
}

bool connect_smooth(std::vector<double> a, std::vector<double> b, OpenRAVE::EnvironmentBasePtr env_pointer, OpenRAVE::RobotBasePtr robot_pointer)
{
    double distance = weighted_distance(a, b);
    for(int i=0; i<=int(distance/step_size); i++)
    {
        //std::cout<<"In loop";
        for(int i = 0; i<7; i++)
        {
            a[i] = a[i] + (step_size*(a[i]-b[i])/weighted_distance(a,b));
        }
        if(!in_limits(a))
            return false;
        if(check_collision(a, env_pointer, robot_pointer))
            return false;

    }
    return true;
}

std::vector<std::vector<double> > smooth_path(std::vector<std::vector<double> > path, OpenRAVE::EnvironmentBasePtr env_pointer)
{
    OpenRAVE::RobotBasePtr robot_pointer = env_pointer->GetRobot("PR2");
    std::vector<double> temp1, temp2;
    int length;

    for(int k = 0; k<200; k++)
    {
        //std::cout<<k<<"\n";
        length = path.size();

        if(length<10)
            break;

        int element1 = 0, element2 = 0;
        while(abs(element1 - element2) <= 1)
        {
            element1 = rand() % length;
            element2 = rand() % length;
            if(element1 > element2)
            {
                element1 = element1 + element2; // a = 3, b = 5, a = 8
                element2 = element1 - element2; // b = 8-5 = 3
                element1 = element1 - element2; // a = 8-3 = 5
            }
        }

        temp1 = path[element1];
        temp2 = path[element2];
        //std::cout<<"\n"<<element1<<" "<<element2;

        if(connect_smooth(temp1, temp2, env_pointer, robot_pointer))
        {
            path.erase(path.begin() + element1 + 1, path.begin() + element2);
            //std::cout<<"\n"<<path.size();
        }
    }
    return path;
}

std::vector<std::vector<double> > BiRRTConnect(OpenRAVE::EnvironmentBasePtr env_pointer)
{
    OpenRAVE::RobotBasePtr robot_pointer = env_pointer->GetRobot("PR2");
    NodeTree treeA, treeB, *treeptr1 = &treeA, *treeptr2 = &treeB, *temp_ptr;
    RRTNode start (start_config, 0, -1);
    RRTNode goal (goal_config, 0, -1);
    node_counterA = 0;
    node_counterB = 0;
    int i = 0, ext;

    treeA.addNode(start);
    treeB.addNode(goal);

    reached_goal = 0;

    std::vector<double> q_rand;

    srand(time(0));

    while(reached_goal != 0)
    {
        q_rand = random_sample();

        ext = 1;
        while (ext == 1)
        {
            ext = extend(treeptr1, q_rand, env_pointer, robot_pointer);
        }
        q_rand = treeA.getLastNode().getConfiguration();
        ext = 1;
        while (ext == 1)
        {
            ext = extend(treeptr2, q_rand, env_pointer, robot_pointer);
        }
        if(treeA.getLastNode().getConfiguration() == treeB.getLastNode().getConfiguration())
        {
            reached_goal = 1;
            break;
        }
        else
        {
            *temp_ptr = *treeptr1;
            *treeptr1 = *treeptr2;
            *treeptr2 = *temp_ptr;
        }

        if(i%100 == 0)
            std::cout<<i<<"\n";
        ++i;

    }

    *treeptr1 = treeA;
    *treeptr2 = treeB;

    std::vector<std::vector<double> > pathA = getPath(treeptr1);
    std::vector<std::vector<double> > pathB = getPath(treeptr2);
    reverse(pathB.begin(), pathB.end());

    for(int i = 0; i<int(pathA.size()); i++)
    {
        for(int j = 0; j<7; j++)
        {
            std::cout<<pathA[i][j];
            if(j<6)
                std::cout<<",";
        }
        std::cout<<std::endl;
    }
    for(int i = 0; i<int(pathA.size()); i++)
    {
        for(int j = 0; j<7; j++)
        {
            std::cout<<pathA[i][j];
            if(j<6)
                std::cout<<",";
        }
        std::cout<<std::endl;
    }
    return pathA;
}
