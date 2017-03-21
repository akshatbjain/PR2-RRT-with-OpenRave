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
    RegisterCommand("RRTConnect", boost::bind(&rrtmodule::RRTConnect,this,_1,_2),
            "This command sends info from python to c++");
    }
    virtual ~rrtmodule() {}
    
    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        sout << "output";
        return true;
    }

    bool RRTConnect(std::ostream& sout, std::istream& sinput)
    {
        // Format in which command is received:
        // RRTConnect goal goal_bias weights
        std::string input; // Variable to read incoming stream of characters

        int biDirectional;
        std::vector<float> config, lower_limit, upper_limit;
        float goal_bias, weights[7]; // Variables for goal_bias and weights

        sout << "\nBidirectional: ";
        sinput >> biDirectional;
        sout << biDirectional;

        sout << "\nGoal configuration: ";
        for(int i = 0; i<7; i++)
        {
            sinput >> input;
            input.erase(std::remove(input.begin(), input.end(), ','), input.end());
            config.push_back(strtof(input.c_str(),0));
            sout << config[i] << " ";
        }

        sout << "\nGoal bias: ";
        sinput >> input;
        goal_bias = strtof(input.c_str(),0);
        sout << goal_bias;

        sout << "\nWeights: ";
        for(int i = 0; i<7; i++)
        {
            sinput >> input;
            input.erase(std::remove(input.begin(), input.end(), ','), input.end());
            weights[i] = strtof(input.c_str(),0);
            sout << weights[i] << " ";

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
                lower_limit.push_back(strtof(input.c_str(),0));
                sout << lower_limit[i] << " ";
            }
            else
            {
                upper_limit.push_back(strtof(input.c_str(),0));
                sout << upper_limit[i-7] << " ";
            }

        }

        getJointLimits(lower_limit, upper_limit);

        sout << "\nRandom Sample: \n";
        std::vector<float> q_rand = random_sample();
        for(int i = 0; i<7; i++)
        {
            sout << q_rand[i] << " ";
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

