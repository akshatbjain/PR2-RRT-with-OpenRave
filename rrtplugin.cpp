#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <vector>
#include <string>
#include "RRT.cpp"

using namespace OpenRAVE;

class rrtmodule : public ModuleBase
{
public:
    rrtmodule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
    RegisterCommand("MyCommand",boost::bind(&rrtmodule::MyCommand,this,_1,_2),
                        "This is an example command");
    RegisterCommand("RRT", boost::bind(&rrtmodule::RRT,this,_1,_2),
            "This command sends info from python to c++");
    RegisterCommand("Start",boost::bind(&rrtmodule::Start,this,_1,_2),
                    "TP");
    }
    virtual ~rrtmodule() {}
    
    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        sout << "output";
        return true;
    }

    bool RRT(std::ostream& sout, std::istream& sinput)
    {
        // Format in which command is received:
        // RRTConnect goal goal_bias weights
        std::string input; // Variable to read incoming stream of characters

        sout << "\nBidirectional: ";
        sinput >> biDirectional;
        sout << biDirectional;

        sout << "\nStart configuration: ";
        for(int i = 0; i<7; i++)
        {
            sinput >> input;
            input.erase(std::remove(input.begin(), input.end(), ','), input.end());
            start_config.push_back(strtof(input.c_str(),0));
            sout << start_config[i] << " ";
        }

        sout << "\nGoal configuration: ";
        for(int i = 0; i<7; i++)
        {
            sinput >> input;
            input.erase(std::remove(input.begin(), input.end(), ','), input.end());
            goal_config.push_back(strtof(input.c_str(),0));
            sout << goal_config[i] << " ";
        }

        sout << "\nGoal bias: ";
        sinput >> goal_bias;
        sout << goal_bias;

        sout << "\nStep size: ";
        sinput >> step_size;
        sout << step_size;

        sout << "\nWeights: ";
        for(int i = 0; i<7; i++)
        {
            sinput >> input;
            input.erase(std::remove(input.begin(), input.end(), ','), input.end());
            joint_weights[i] = strtof(input.c_str(),0);
            sout << joint_weights[i] << " ";

        }

        sout << "\nLower and Upper Joint Limits: \n";
        for(int i = 0; i<14; i++)
        {
            sinput >> input;
            input.erase(std::remove(input.begin(), input.end(), ','), input.end());
            if(i==7)
                sout << "\n";
            if(i<7)
            {
                lower_joint_limits.push_back(strtof(input.c_str(),0));
                sout << lower_joint_limits[i] << " ";
            }
            else
            {
                upper_joint_limits.push_back(strtof(input.c_str(),0));
                sout << upper_joint_limits[i-7] << " ";
            }

        }

        sout << "\nRandom Sample: \n";
        std::vector<double> q_rand = random_sample();
        for(int i = 0; i<7; i++)
        {
            sout << q_rand[i] << " ";
        }

        return true;
    }

    bool Start(std::ostream& sout, std::istream& sinput)
    {
        std::vector<std::vector<double> > path = RRTConnect(GetEnv());
        for(int i = 0; i<path.size(); i++)
        {
            for(int j = 0; j<7; j++)
            {
                sout << path[i][j];
                if(j<6)
                    sout << ",";
            }
            if(i < path.size() - 1)
                sout << std::endl;
        }
        return true;
    }
};

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "rrtmodule" ) {
        return InterfaceBasePtr(new rrtmodule(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("rrtmodule");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

