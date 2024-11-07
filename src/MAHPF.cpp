#include "MAHPF.h"
MAHPF::MAHPF(const Instance& instance, double time_limit,
                string init_algo_name, string merge_algo, int screen):
    instance(instance), time_limit(time_limit),
    init_algo_name(init_algo_name), merge_algo(merge_algo), screen(screen),
    path_table(instance.map_size)
{
    int N = instance.getDefaultNumberOfAgents(AgentType::ROBOT);
    robots.reserve(N);
    for (int i = 0; i < N; i++)
        robots.emplace_back(instance, AgentID(i,AgentType::ROBOT));
    int M=instance.getDefaultNumberOfAgents(AgentType::HUMAN);
    humans.reserve(M);
    for (int i = 0; i < M; i++)
        humans.emplace_back(instance, AgentID(i,AgentType::HUMAN));
    time_limit=time_limit;
    cur_Soc=0;
    run_time=0;
    init_conf=0;
}

bool MAHPF::getInitialSolution() {
    cout<<"seraching for initial solution"<<endl;
    bool succ;
    if (init_algo_name == "OPTIMAL") {
        runHuman();
        cout<<"path after return"<<endl;
        cout<<humans[0].path<<endl;

        succ=runCBS();
        if (screen>0)
        {
            cout<<"initial path"<<endl;
            printPathsA();
        }
    } else if (init_algo_name == "Sub-OPTIMAL") {
        // to be implemented
    } else {
        cout << "Invalid initial solution algorithm name" << endl;
        return false;
    }
    for (Agent r:robots)
        init_sol.Soc+=r.path.size();
    init_sol.makespan=std::max(path_table.makespan,(int)humans[0].path.size());

    cur_Soc=init_sol.Soc;
    return succ;
}

bool MAHPF::merge()
{
    list<AgentID> confAgents;
    checkConflict(confAgents);
    if (confAgents.empty())
    {
        cout<<"no conflict already, no need to merge"<<endl;
        //logPath("path.log");
        return true;
    }
    cout<<"confAgents: ";
    for (AgentID id:confAgents)
        cout<<id<<", ";
    cout<<endl;
    init_conf=confAgents.size();

    if(merge_algo== "OPTIMAL")
    {
        cout<<"start merging with OPTIMAL"<<endl;
    }
    // to be implemented
    else if(merge_algo== "Sub-OPTIMAL")
    {
        cout<<"start merging with Sub-OPTIMAL"<<endl;
        mergeSubOPTIMAL(false);
    }
    else if(merge_algo== "Sub-OPTIMAL-P1")
    {
        cout<<"start merging with Sub-OPTIMAL-P1"<<endl;
        mergeSubOPTIMAL(true);
    }
    else if(merge_algo== "superMCP")
    {
        cout<<"start merging with SuperMCP"<<endl;
        if (!mergeSuperMCP()) return false;
    }
    else if(merge_algo== "MCP")
    {
        cout<<"start merging with MCP"<<endl;
        if(!mergeMCP()) return false;
    }
    else if(merge_algo== "push")
    {
        cout<<"start merging with pushing"<<endl;
        if(!mergePush()) return false;
    }
    else if(merge_algo== "stop")
    {
        cout<<"start merging with stopping the world"<<endl;
        if(!mergeStop()) return false;
    }
    else
    {
        cout<<"wrong merge method"<<endl;
        return false;
    }

    final_sol.Soc=humans[0].path.size();
    for (Agent r:robots)
        final_sol.Soc+=r.path.size();
    final_sol.makespan=std::max(path_table.makespan,(int)humans[0].path.size());
    cur_Soc=final_sol.Soc;
    return true;
}

bool MAHPF::mergeStop()
{
    clock_t start = clock();
    for (int h=0;h<humans.size();h++)
    {
        list<int> conf;
        for (int r=0;r<robots.size();r++)
        {
            for (int t=0;t<humans[h].path.size();t++)
            {
                int temp=robots[r].path_planner.start_location;
                if (humans[h].path[t].location==temp)
                {
                    conf.push_back(temp);
                    break;
                }
            }
        }

        humans[h].path_planner.setObs(conf);
        Path p=humans[h].path_planner.findOptimalPath(path_table);
        humans[h].path_planner.unsetObs();

        if (!p.empty())
        {
            humans[h].path=p;
            int Soc=humans[h].path.size();
            for (int r=0;r<robots.size();r++)
            {
                robots[r].path.resize(robots[r].path.size()+Soc-1);
                for (int t=robots[r].path.size()-1;t>=0;t--)
                {
                    if (t>Soc-1)
                        robots[r].path[t]=robots[r].path[t-Soc+1];
                    else
                        robots[r].path[t]=robots[r].path[0];
                }
            }

            if (screen>0)
            {
                cout<<"finished fixing"<<endl;
                printPathsA();
            }
            run_time += (double)(clock() - start) / CLOCKS_PER_SEC;
            return true;
        }
        else
        {
            if (screen>0)
                cout<<"failed at fixing"<<endl;
            run_time += (double)(clock() - start) / CLOCKS_PER_SEC;
            return false;
        }
    }
}

bool MAHPF::mergeOPTIMAL()
{

}
bool MAHPF::mergeSubOPTIMAL(bool fix_human)
{
    mergePP(fix_human);
}

bool MAHPF::mergePush()
{
    PathPool robot_path;
    for (Agent r:robots)
        robot_path.emplace_back(r.path);

    for (Agent h:humans)
    {
        for (PathEntry pe:h.path)
        {

        }
    }
}

bool MAHPF::mergePP(bool fix_human)
{
    clock_t start = clock();
    double loop_time=0;
    if (!fix_human)
    {
        Path p=humans[0].path_planner.findOptimalPath(path_table,SpaceTimeAStar::Cost::CONF);
        if (!p.empty())
        {
            humans[0].path=p;
            run_time += (double)(clock() - start) / CLOCKS_PER_SEC;
            return true;
        }
    }
    if (screen>0)
    {
        cout<<"solution before merge"<<endl;
        printPathsA();
    }

    list<AgentID> failedAgents;
    checkConflict(failedAgents);
    for (AgentID id:failedAgents)
        path_table.deletePath(id,robots[id.id].path);
    path_table.insertPath(humans[0].id,humans[0].path);

    vector<int> idx_in;
    vector<int> idx_out;
    for (int i=1;i<robots.size();i++)
        idx_out.push_back(i);
    for (AgentID id:failedAgents)
    {
        idx_in.push_back(id.id);
        idx_out.erase(std::remove(idx_out.begin(), idx_out.end(), id.id), idx_out.end());
    }

    clock_t start_loop=clock();
    while (run_time+loop_time<time_limit)
    {
        loop_time = (double)(clock() - start_loop) / CLOCKS_PER_SEC;
        bool fail=true;
        while(loop_time+run_time<time_limit && fail)
        {
            loop_time = (double)(clock() - start_loop) / CLOCKS_PER_SEC;
            cout<<"printing here"<<endl;
            bool succ;
            std::random_shuffle(idx_in.begin(), idx_in.end());
            std::random_shuffle(idx_out.begin(), idx_out.end());
            if (idx_out.size()>0)
            {
                int new_id=idx_out.front();
                idx_in.push_back(new_id);
                idx_out.erase(idx_out.begin());
                path_table.deletePath({new_id,AgentType::ROBOT},robots[new_id].path);
            }

            fail=false;
            for (int id:idx_in)
            {
                cout<<"id: "<<id<<endl;
                Path p=robots[id].path_planner.findOptimalPath(path_table);
                if (!p.empty())
                {
                    path_table.insertPath({id,AgentType::ROBOT},p);
                    robots[id].path=p;
                }
                else
                {
                    fail=true;
                    break;
                    for (int tmpid:idx_in)
                    {
                        path_table.deletePath({id,AgentType::ROBOT},robots[id].path);
                        if (tmpid==id) break;
                    }
                }
            }
        }

        if (!fix_human)
        {

            Path p=humans[0].path_planner.findOptimalPath(path_table,SpaceTimeAStar::Cost::CONF);
            if (!p.empty())
            {
                humans[0].path=p;
                if (screen>0)
                {
                    cout<<"finished fixing"<<endl;
                    printPathsA();
                }
                run_time += (double)(clock() - start) / CLOCKS_PER_SEC;
                return true;
            }
        }

        checkConflict(failedAgents);
        if (failedAgents.empty())
        {
            if (screen>0)
                printPathsA();
            run_time += (double)(clock() - start) / CLOCKS_PER_SEC;
            return true;
        }

    }
}

bool MAHPF::runHuman()
{
    clock_t start = clock();
    for (int h=0;h<humans.size();h++)
    {
        Path p=humans[h].path_planner.findOptimalPath(path_table);
        if (p.empty())
        {
            cout << "No path found for human:"<<humans[h].id << endl;
            return false;
        }
        humans[h].path=p;
        init_sol.Soc=humans[h].path.size();
    }
    run_time += (double)(clock() - start) / CLOCKS_PER_SEC;
    return true;
}

bool MAHPF::runCBS()
{
    if (screen >= 2)
        cout << "initing with CBS " << endl;
    vector<SingleAgentSolver*> search_engines;
    search_engines.reserve(robots.size());
    for (int i = 0; i < robots.size(); i++)
    {
        search_engines.push_back(&robots[i].path_planner);
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
    clock_t start = clock();
    bool succ = cbs.solve(time_limit-run_time, 0);
    run_time += (double)(clock() - start) / CLOCKS_PER_SEC;

    if (succ)
    {
        for (size_t i = 0; i < robots.size(); i++)
        {
            robots[i].path = *cbs.paths[i];
            path_table.insertPath(robots[i].id, robots[i].path);
        }
        /*
           if (sum_of_costs_lowerbound < 0)
           sum_of_costs_lowerbound = cbs.getLowerBound();*/
    }
    return succ;
}

void MAHPF::checkConflict(list<AgentID> &confAgents)
{
    confAgents.clear();
    for (Agent h : humans)
    {
        for (int t=0;t<h.path.size();t++)
        {
            AgentID confAgent(-1,AgentType::NONE);
            for (Agent r:robots)
                if (t<r.path.size() && r.path[t].location==h.path[t].location)
                {
                    confAgent.id=r.id.id;
                    confAgent.type=r.id.type;
                    break;
                }
            if (confAgent && find(confAgents.begin(),confAgents.end(),confAgent)==confAgents.end())
            {
                confAgents.push_back(confAgent);
            }
        }
    }
}

bool MAHPF::mergeMCP()
{
    cout<<"starting merge"<<endl;
    clock_t start = clock();
    double loop_time=0;
    PathPool robot_path;
    for (Agent r:robots)
        robot_path.emplace_back(r.path);

    clock_t start_loop=clock();
    for (Agent h : humans)
    {
        for (int t=0;t<h.path.size();t++)
        {
            loop_time = (double)(clock() - start_loop) / CLOCKS_PER_SEC;
            while (run_time+loop_time<time_limit)
            {
                if(intersect(t,h.path[t].location,robot_path)!=-1)
                {
                    if(!delayAgents(t,robot_path))
                    {
                        run_time += (double)(clock() - start) / CLOCKS_PER_SEC;
                        return false;
                    }
                }
                else
                    break;
            }
        }
    }

    path_table.reset();
    for (int i=0;i<robot_path.size();i++)
    {
        robots[i].path=robot_path[i];
        path_table.insertPath(robots[i].id,robot_path[i]);
    }
    run_time += (double)(clock() - start) / CLOCKS_PER_SEC;
    return true;
}

bool MAHPF::mergeSuperMCP()
{
    cout<<"starting super merge"<<endl;
    clock_t start = clock();
    int max_lim=path_table.makespan;
    for (Agent h : humans)
    {
        for (int i=0;i<max_lim;i++)
        {
            if (mergeMCP()) {cout<<"return A"<<endl;return true;}
            for (int t=h.path.size()-1;t>i-1;t--)
                h.path[i]=h.path[i-1];
            list<AgentID> confAgents;
            checkConflict(confAgents);
            if (confAgents.empty()) {cout<<"return B"<<endl;return true;}
        }
    }
    run_time += (double)(clock() - start) / CLOCKS_PER_SEC;
    return true;
}

int MAHPF::intersect(int t, int loc, PathPool& P){
    for (int i=0;i<P.size();i++)
        if (t<P[i].size() && P[i][t].location==loc)
            return i;
    return -1;
}

bool MAHPF::delayAgents(int t, PathPool &P)
{
    for (int i=0;i<P.size();i++)
    {
        if (P[i].size()>1000) return false;
        if (t>=P[i].size()-1) continue;
        P[i].resize(P[i].size()+1);
        for (int j=P[i].size()-1;j>t-1;j--)
            P[i][j]=P[i][j-1];

    }
    return true;
}

void MAHPF::printPathsA()
{
    for (Agent h:humans)
    {
        cout<<h.id<<": @"<<h.path;
    }
    cout<<"-------"<<endl;
    for (Agent r:robots)
    {
        cout<<r.id<<": @"<<r.path;
    }
}

void MAHPF::printPathsT(bool only_conf)
{
    for (int t=0;t<humans[0].path.size();t++)
    {
        bool prted=false;
        if (!only_conf)
            cout<<"t: "<<t<<" h: "<<humans[0].path[t].location<<endl;
        for (Agent R: robots)
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

void MAHPF::logPath(string fn)
{
    ofstream fw(fn, std::ios::out);
    if (!fw.is_open()){
        std::cerr<<"did not open result file properly" << std::endl<<std::flush;
        exit(1);
    }

    fw<<"human: "<<endl;
    for (int h=0;h<humans.size();h++)
    {
        for (PathEntry pe: humans[h].path)
        {
            pair<int, int> coord=instance.getCoordinate(pe.location);
            fw<<coord.first<<","<<coord.second<<";";
        }
        fw<<endl;
    }

    fw<<"robot: "<<endl;
    for (int a=0;a<robots.size();a++)
    {
        for (PathEntry pe: robots[a].path)
        {
            pair<int, int> coord=instance.getCoordinate(pe.location);
            fw<<coord.first<<","<<coord.second<<";";
        }
        fw<<endl;
    }
    fw.close();
}

void MAHPF::logTrackerPath(string fn)
{
    ofstream fw(fn, std::ios::out);
    if (!fw.is_open()){
        std::cerr<<"did not open result file properly" << std::endl<<std::flush;
        exit(1);
    }

    fw<<"\"agents\",\"lower_cost\",\"lower_date\",\"solution_cost\",\"solution_date\",\"path\" "<<endl;
    fw<<"\""<< (humans.size()+robots.size()) <<" \", \"3\", \"2022-11-29\",\"";
    fw<<cur_Soc<<"\",\"2022-11-29\",\"";


    int ncols=instance.num_of_cols;

    for (int r=0;r<robots.size();r++)
    {
        int next, prev;
        for (int t=1;t<robots[r].path.size();t++)
        {
            next=robots[r].path[t].location;
            prev=robots[r].path[t-1].location;
            if (next - prev == 1) {
                fw << "r";
            } else if (next - prev == -1) {
                fw << "l";
            } else if (next - prev == 0) {
                fw << "w";
            } else if (next - prev == ncols) {
                fw << "d";
            } else if (next - prev == -ncols) {
                fw << "u";
            } else {
                std::cerr << "invalid path" << std::endl << std::flush;
            }
        }
        fw<<endl;
    }


    for (int h=0;h<humans.size();h++)
    {
        int next, prev;
        for (int t=1;t<humans[h].path.size();t++)
        {
            next=humans[h].path[t].location;
            prev=humans[h].path[t-1].location;
            if (next - prev == 1) {
                fw << "r";
            } else if (next - prev == -1) {
                fw << "l";
            } else if (next - prev == 0) {
                fw << "w";
            } else if (next - prev == ncols) {
                fw << "d";
            } else if (next - prev == -ncols) {
                fw << "u";
            } else {
                std::cerr << "invalid path" << std::endl << std::flush;
            }
        }
        fw<<endl;
    }
    fw<<"\" ";
}

void MAHPF::logStats(int n)
{
        if (n==0)
        {
            cout<<init_sol;
            cout<<"init time"<<run_time<<endl;
        }
        else if (n==1)
        {
            cout<<final_sol;
            cout<<"merge time"<<run_time<<endl;
        }
    }

    void MAHPF::logExpStats(const string& statsFn, const string& map, const string& instance, int r, int h,
            const string& initAlgo, const string& mergeAlgo)
    {
        std::ofstream file(statsFn, std::ios::app);

        if (file.is_open()) {
            file<<map<<", "<<instance<<", "<<r<<", "<<h<<", "<<run_time
                <<", "<<initAlgo<<", "<<init_sol.makespan<<", "<<init_sol.Soc
                <<", "<<init_conf
                <<", "<<mergeAlgo<<", "<<final_sol.makespan<<", "<<final_sol.Soc
                <<", "<<endl;
            file.close();     // Close the file
        } else {
            std::cerr << "Error: Could not open file for appending." << std::endl;
        }

    }

