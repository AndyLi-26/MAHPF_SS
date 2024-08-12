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
        vector<Agent> agents;
        vector<Agent> humans;

    private:
        bool runCBS();
        bool runHuman();
        bool mergeMCP();
        string init_algo_name;
        string merge_algo;
        const Instance& instance;
        double time_limit;
        int screen;
        PathTable path_table; // 1. stores the paths of all agents in a time-space table;
};
