#pragma once
#include "SingleAgentSolver.h"
#include "PathTable.h"


class AStarNode: public LLNode
{
public:
	// define a typedefs for handles to the heaps (allow up to quickly update a node in the heap)
	typedef pairing_heap< AStarNode*, compare<LLNode::compare_node> >::handle_type dis_open_handle_t;
	typedef pairing_heap< AStarNode*, compare<LLNode::secondary_compare_node> >::handle_type conf_open_handle_t;
	dis_open_handle_t dis_open_handle;
	conf_open_handle_t conf_open_handle;
	conf_open_handle_t dis_focal_handle;


	AStarNode() : LLNode() {}

	AStarNode(int loc, int g_val, int h_val, LLNode* parent, int timestep, int num_of_conflicts = 0, bool in_openlist = false) :
		LLNode(loc, g_val, h_val, parent, timestep, num_of_conflicts, in_openlist) {}


	~AStarNode() {}

	// The following is used by for generating the hash value of a nodes
	struct NodeHasher
	{
		size_t operator()(const AStarNode* n) const
		{
			size_t loc_hash = std::hash<int>()(n->location);
			size_t timestep_hash = std::hash<int>()(n->timestep);
			return (loc_hash ^ (timestep_hash << 1));
		}
	};

	// The following is used for checking whether two nodes are equal
	// we say that two nodes, s1 and s2, are equal if
	// both are non-NULL and agree on the id and timestez
	struct eqnode
	{
		bool operator()(const AStarNode* s1, const AStarNode* s2) const
		{
			return (s1 == s2) || (s1 && s2 &&
                        s1->location == s2->location &&
                        s1->timestep == s2->timestep &&
						s1->wait_at_goal == s2->wait_at_goal);
		}
	};
};


class SpaceTimeAStar: public SingleAgentSolver
{
public:
    enum Cost {
        CONF,
        DIS
    };

    // find path by time-space A* search
    // Returns a shortest path that does not collide with paths in the path table
    Path findOptimalPath(const PathTable& path_table, Cost obj);
    Path findOptimalPath(const PathTable& path_table){findOptimalPath(path_table,Cost::DIS);}
    //Path findLeastCollisionPath(const PathTable& path_table)
	// find path by time-space A* search
	// Returns a shortest path that satisfies the constraints of the give node  while
	// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
	// lowerbound is an underestimation of the length of the path in order to speed up the search.
	Path findOptimalPath(const HLNode& node, const ConstraintTable& initial_constraints,
						const vector<Path*>& paths, int agent, int lower_bound);
	pair<Path, int> findSuboptimalPath(const HLNode& node, const ConstraintTable& initial_constraints,
		const vector<Path*>& paths, int agent, int lowerbound, double w);  // return the path and the lowerbound

	int getTravelTime(int start, int end, const ConstraintTable& constraint_table, int upper_bound);

	string getName() const { return "AStar"; }

	SpaceTimeAStar(const Instance& instance, AgentID id):
		SingleAgentSolver(instance, id) {}

private:
	// define typedefs and handles for heap
	typedef pairing_heap< AStarNode*, compare<AStarNode::compare_node> > dis_open_t;
	typedef pairing_heap< AStarNode*, compare<AStarNode::secondary_compare_node> > conf_open_t;
	dis_open_t dis_open_list;
	conf_open_t conf_open_list;
	conf_open_t dis_focal_list;

	// define typedef for hash_map
	typedef unordered_set<AStarNode*, AStarNode::NodeHasher, AStarNode::eqnode> hashtable_t;
	hashtable_t allNodes_table;

	// Updates the path datamember
	void updatePath(const LLNode* goal, vector<PathEntry> &path);
	void updateFocalList();
	inline AStarNode* popNode(Cost obj);
	inline void pushNode(AStarNode* node, Cost obj);
	void releaseNodes();

};
