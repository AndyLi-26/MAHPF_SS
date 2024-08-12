#pragma once
#include <tuple>
#include <list>
#include <vector>
#include <set>
#include <ctime>
#include <fstream>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision
#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

using boost::heap::pairing_heap;
using boost::heap::compare;
using boost::unordered_map;
using boost::unordered_set;
using std::vector;
using std::list;
using std::set;
using std::get;
using std::tuple;
using std::make_tuple;
using std::pair;
using std::make_pair;
using std::tie;
using std::min;
using std::max;
using std::shared_ptr;
using std::make_shared;
using std::clock;
using std::cout;
using std::endl;
using std::ofstream;
using std::cerr;
using std::string;


// #define NDEBUG

#define MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES INT_MAX / 2


enum AgentType
{
    HUMAN,
    ROBOT,
    NONE
};

struct PathEntry
{
	int location = -1;
	PathEntry(int loc = -1) { location = loc; }
};

struct AgentID
{
    int id;
    AgentType type;
    AgentID(int id, AgentType type) : id(id), type(type) {}
    bool operator==(const AgentID& other) const
    {
        return id == other.id && type == other.type;
    }
    bool operator!=(const AgentID& other) const
    {
        return !(*this == other);
    }

    operator bool() const
    {
        return id != -1 && type != NONE;
    }

};

typedef vector<PathEntry> Path;
std::ostream& operator<<(std::ostream& os, const Path& path);
bool isSamePath(const Path& p1, const Path& p2);

struct IterationStats
{
    int sum_of_costs;
    double runtime;
    int num_of_agents;
    string algorithm;
    int sum_of_costs_lowerbound;
    IterationStats(int num_of_agents, int sum_of_costs, double runtime, string algorithm,
            int sum_of_costs_lowerbound = 0) :
            num_of_agents(num_of_agents), sum_of_costs(sum_of_costs), runtime(runtime),
            sum_of_costs_lowerbound(sum_of_costs_lowerbound), algorithm(algorithm) {}
};

struct PIBTPPS_option{
    int windowSize ;
    bool winPIBTSoft ;
    int timestepLimit ;
};

// Only for three-tuples of std::hash-able types for simplicity.
// You can of course template this struct to allow other hash functions
/*struct three_tuple_hash {
    template <class T1, class T2, class T3>
    std::size_t operator () (const std::tuple<T1, T2, T3> &p) const {
        auto h1 = std::hash<T1>{}(get<0>(p));
        auto h2 = std::hash<T2>{}(get<1>(p));
        auto h3 = std::hash<T3>{}(get<2>(p));
        // Mainly for demonstration purposes, i.e. works but is overly simple
        // In the real world, use sth. like boost.hash_combine
        return h1 ^ h2 ^ h3;
    }
};*/

