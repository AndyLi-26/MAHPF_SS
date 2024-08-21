
#pragma once
#ifndef AGENT_H
#define AGENT_H

#include "common.h"
//#include "Instance.h"


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

#include "SpaceTimeAStar.h"
#include "humanSingle.h"

struct Agent
{
    AgentID id;
    SpaceTimeAStar path_planner; // start, goal, and heuristics are stored in the path planner
    Path path;

    Agent(const Instance& instance, AgentID id) : id(id), path_planner(instance,id){}
    int getNumOfDelays() const { return (int) path.size() - 1 - path_planner.my_heuristic[path_planner.start_location]; }
};

struct Human
{
    int id;
    HumanSingle path_planner; // start, goal, and heuristics are stored in the path planner
    Path path;

    Human(const Instance& instance, int id) : id(id), path_planner(instance,{id,AgentType::HUMAN}){}
    int getNumOfDelays() const { return (int) path.size() - 1 - path_planner.my_heuristic[path_planner.start_location]; }
    AgentID getID() { return {id,AgentType::HUMAN};}
};

#endif
