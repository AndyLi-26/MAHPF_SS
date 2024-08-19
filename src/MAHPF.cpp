#include "MAHPF.h"
MAHPF::MAHPF(const Instance& instance, double time_limit,
                string init_algo_name, string merge_algo, int screen):
    instance(instance), time_limit(time_limit),
    init_algo_name(init_algo_name), merge_algo(merge_algo), screen(screen),
    path_table(instance.map_size)
{
    int N = instance.getDefaultNumberOfAgents(AgentType::ROBOT);
    agents.reserve(N);
    for (int i = 0; i < N; i++)
        agents.emplace_back(instance, AgentID(i,AgentType::ROBOT));
    int M=instance.getDefaultNumberOfAgents(AgentType::HUMAN);
    humans.reserve(M);
    for (int i = 0; i < M; i++)
        humans.emplace_back(instance, AgentID(i,AgentType::HUMAN));
}

bool MAHPF::getInitialSolution() {
    cout<<"seraching for initial solution"<<endl;
    if (init_algo_name == "OPTIMAL") {
        runHuman();
        cout<<"path after return"<<endl;
        cout<<humans[0].path<<endl;

        runCBS();
    } else if (init_algo_name == "Sub-OPTIMAL") {
        // to be implemented
    } else {
        cout << "Invalid initial solution algorithm name" << endl;
        return false;
    }
}

bool MAHPF::merge()
{
    list<AgentID> confAgents;
    checkConflict(confAgents);
    if (confAgents.empty())
    {
        cout<<"no conflict already, no need to merge"<<endl;
        return true;
    }
    if (merge_algo == "OPTIMAL") {
        // to be implemented
    } else if (merge_algo == "Sub-OPTIMAL") {
        // to be implemented
    } else if (merge_algo == "MCP") {
        printPaths(1);
        cout<<humans[0].path<<endl;
        cout<<"start merging with MCP"<<endl;
        return mergeSuperMCP();

        // to be implemented
    }
}

bool MAHPF::mergeSubOPTIMAL()
{
    mergePP();
}

bool MAHPF::mergePP()
{
    Path p=humans[0].path_planner->findOptimalPath(path_table);
    if (!p.empty())
    {
        humans[0].path=p;
        return true;
    }
    high_resolution_clock::time_point start_time=Time::now();
    double run_time=0;
    while (run_time<time_limit)
    {
        list<AgentID> failedAgents;
        run_time = ((fsec)(Time::now() - start_time)).count();
    }
}

bool MAHPF::runHuman()
{
    for (int h=0;h<humans.size();h++)
    {
        Path p=humans[h].path_planner->findOptimalPath(path_table);
        if (p.empty())
        {
            cout << "No path found for human:"<<humans[h].id << endl;
            return false;
        }
        humans[h].path=p;
    }
    return true;
}

bool MAHPF::runCBS()
{
    if (screen >= 2)
        cout << "initing with CBS " << endl;
    vector<SingleAgentSolver*> search_engines;
    search_engines.reserve(agents.size());
    for (int i = 0; i < agents.size(); i++)
    {
        search_engines.push_back(agents[i].path_planner);
    }

    CBS cbs(search_engines, path_table, screen - 1);
    cbs.setPrioritizeConflicts(true);
    cbs.setDisjointSplitting(false);
    cbs.setBypass(true);
    cbs.setRectangleReasoning(true);
    cbs.setCorridorReasoning(true);
    cbs.setHeuristicType(heuristics_type::WDG, heuristics_type::ZERO);
    cbs.setTargetReasoning(true);
    cbs.setMutexReasoning(false);
    cbs.setConflictSelectionRule(conflict_selection::EARLIEST);
    cbs.setNodeSelectionRule(node_selection::NODE_CONFLICTPAIRS);
    cbs.setSavingStats(false);
    cbs.setHighLevelSolver(high_level_solver_type::ASTAR, 1);
    bool succ = cbs.solve(60, 0);

    if (succ)
    {
        for (size_t i = 0; i < agents.size(); i++)
        {
            agents[i].path = *cbs.paths[i];
            path_table.insertPath(agents[i].id, agents[i].path);
        }
        /*
        if (sum_of_costs_lowerbound < 0)
            sum_of_costs_lowerbound = cbs.getLowerBound();*/
    }
    return succ;
}

void MAHPF::checkConflict(list<AgentID> confAgents)
{
    for (Agent h : humans)
    {
        for (int t=0;t<h.path.size();t++)
        {
            AgentID confAgent=intersect(t,h.path[t].location);
            if (confAgent)
                confAgents.push_back(confAgent);
        }
    }
}

bool MAHPF::mergeSuperMCP()
{
    for (Agent h : humans)
    {
        for (int t=0;t<h.path.size();t++)
        {
            //cout<<"t:"<<t<<endl;
            while (1)
            {
                if(intersect(t,h.path[t].location))
                {
                    if(!delayRobots(t)) return false;
                }
                else
                {
                    break;
                }

            }
        }
    }
    return true;
}
AgentID MAHPF::intersect(int t, int loc){
    for (Agent R: agents)
    {
        if (t<R.path.size() && R.path[t].location==loc)
        {
            //cout<<"id: "<<R.id.id<<endl;
            return R.id;
        }
    }
    return {-1,AgentType::NONE};
}

bool MAHPF::delayRobots(int t)
{
    cout<<"interference detected"<<endl;
    for (Agent R: agents)
    {
        if (R.path.size()==1000) return false;
        R.path.resize(R.path.size()+1);
        for (int i=R.path.size()-1;i>t-1;i--)
        {
            R.path[i]=R.path[i-1];
        }

    }
}


void MAHPF::printPaths(bool only_conf)
{
    for (int t=0;t<humans[0].path.size();t++)
    {
        bool prted=false;
        if (!only_conf)
            cout<<"t: "<<t<<endl;
        for (Agent R: agents)
        {
            if (R.path.size()>t)
                if (R.path[t].location==humans[0].path[t].location)
                {
                    if (!prted)
                    {
                        cout<<"t: "<<t<<endl;
                        prted=true;
                    }
                    cout<<"["<<R.id.id<<","<<R.path[t].location<<"]  ";
                }
                else if(!only_conf)
                    cout<<"("<<R.id.id<<","<<R.path[t].location<<")  ";
        }
        cout<<endl;
    }
}
