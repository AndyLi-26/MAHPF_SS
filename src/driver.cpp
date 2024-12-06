#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include "LNS.h"
#include "MAHPF.h"
#include "AnytimeBCBS.h"
#include "AnytimeEECBS.h"
//#include "PIBT/pibt.h"


/* Main function */
int main(int argc, char** argv)
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")

		// params for the input instance and experiment settings
		("map,m", po::value<string>()->required(), "input file for map")
		("agents,a", po::value<string>()->required(), "input file for agents")
		//("humans,h", po::value<string>()->required(), "input file for humans")
		("robotNum,r", po::value<int>()->default_value(0), "number of agents")
		("humanNum,h", po::value<int>()->default_value(1), "number of human")
        ("output,o", po::value<string>(), "output file")
		("cutoffTime,t", po::value<double>()->default_value(7200), "cutoff time (seconds)")
		("screen,s", po::value<int>()->default_value(0),
		        "screen option (0: none; 1: LNS results; 2:LNS detailed results; 3: MAPF detailed results)")
		("stats", po::value<string>(), "output stats file")

		// solver
		("solver", po::value<string>()->default_value("LNS"), "solver (LNS, A-BCBS, A-EECBS)")

        // params for LNS
        ("neighborSize", po::value<int>()->default_value(5), "Size of the neighborhood")
        ("maxIterations", po::value<int>()->default_value(1000000), "maximum number of iterations")
        ("initAlgo", po::value<string>()->default_value("OPTIMAL"),
                "MAPF algorithm for finding the initial solution (OPTIMAL,Sub_OPTIMAL)")
        ("mergeAlgo", po::value<string>()->default_value("MCP"),
                "MAPF algorithm for finding the merge solution (MCP,LNS,PP)")
        ("replanAlgo", po::value<string>()->default_value("PP"),
                "MAPF algorithm for replanning (EECBS, CBS, PP)")
        ("destoryStrategy", po::value<string>()->default_value("Adaptive"),
                "Heuristics for finding subgroups (Random, RandomWalk, Intersection, Adaptive)")
        ("pibtWindow", po::value<int>()->default_value(5),
             "window size for winPIBT")
        ("winPibtSoftmode", po::value<bool>()->default_value(true),
             "winPIBT soft mode")
		;
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		cout << desc << endl;
		return 1;
	}

    /*
    PIBTPPS_option pipp_option;
    pipp_option.windowSize = vm["pibtWindow"].as<int>();
    pipp_option.winPIBTSoft = vm["winPibtSoftmode"].as<bool>();
    */
    po::notify(vm);

	srand((int)time(0));

    string map=vm["map"].as<string>(),agents=vm["agents"].as<string>();
    int h=vm["humanNum"].as<int>(),a=vm["robotNum"].as<int>();
	Instance instance(map, agents,h,a);

    double time_limit = vm["cutoffTime"].as<double>();
    int screen = vm["screen"].as<int>();
	srand(0);
    string initAlgo=vm["initAlgo"].as<string>(),mergeAlgo=vm["mergeAlgo"].as<string>();
    MAHPF mahpf(instance, time_limit, initAlgo, mergeAlgo, screen);
    if (mahpf.getInitialSolution())
    {
        mahpf.logStats(0);
        //mahpf.logPath("init.log");
        //mahpf.logTrackerPath("init_path.csv");
        if (mahpf.merge())
        {
            cout<<"successfully merged"<<endl;
            mahpf.logStats(1);
            //mahpf.logPath("merged.log");
            //mahpf.logTrackerPath("final_path.csv");
        }
        else {
            mahpf.logStats(1);
            cout<<"merge failed, choice another merge algo"<<endl;
        };
    }
    else {
        cout<<"no initial solution found"<<endl;
    }

    if (vm.count("stats"))
    {
        string statsFn=vm["stats"].as<string>();
        mahpf.logExpStats(statsFn,map,agents,a,h,initAlgo,mergeAlgo);
    }

    return 0;
}
