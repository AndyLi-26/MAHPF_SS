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
    getInitialSolution();
}

bool MAHPF::getInitialSolution() {
    if (init_algo_name == "OPTIMAL") {
        runHuman();
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
    if (merge_algo == "OPTIMAL") {
        // to be implemented
    } else if (merge_algo == "Sub-OPTIMAL") {
        // to be implemented
    } else if (merge_algo == "MCP") {
        mergeMCP();
        // to be implemented
    }
}

bool MAHPF::runHuman()
{
    for (Agent h : humans)
    {
        Path p=h.path_planner->findOptimalPath(path_table);
        if (p.empty())
        {
            cout << "No path found for human:"<<h.id << endl;
            return false;
        }
        cout<<p<<endl;
    }
    //path_table.insertPath(-1, p);
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
    cout<<"cbs inited"<<endl;
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
    instance.printAgents(AgentType::ROBOT);

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

bool MAHPF::mergeMCP()
{
    // to be implemented
}
