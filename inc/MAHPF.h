#pragma once
#include "ECBS.h"
#include "SpaceTimeAStar.h"
#include "LNS.h"

class MAHPF
{
    public:
        MAHPF(const Instance& instance, double time_limit,
                string init_algo_name, string merge_algo, int screen);

        bool getInitialSolution();
        bool merge();

        Stats init_sol;
        Stats final_sol;
        int cur_Soc;

        void logStats(int n);
        void logExpStats(const string& statsFn, const string& map, const string& instance, int r, int h,
                const string& initAlgo, const string& mergeAlgo);
        void logPath(string fn);
        void logTrackerPath(string fn);
        void printPathsT(bool only_conf);
        void printPathsA();

    private:
        vector<Agent> robots;
        vector<Agent> humans;
        bool runCBS(bool init);
        bool runHuman();
        bool mergeSuperMCP();
        bool mergeMCP();
        bool mergeStop();
        bool mergePP(bool fix_human);
        bool mergePush();
        bool mergeSubOPTIMAL(bool fix_human);
        bool mergeOPTIMAL();
        int intersect(int t, int loc, PathPool& P);
        void checkConflict(list<AgentID> &confAgents);
        bool delayAgents(int t, PathPool &P);
        string init_algo_name;
        string merge_algo;
        const Instance& instance;
        double time_limit;
        double run_time = 0;
        int init_conf;
        int screen;
        bool logP;
        PathTable path_table; // 1. stores the paths of all agents in a time-space table;
};
