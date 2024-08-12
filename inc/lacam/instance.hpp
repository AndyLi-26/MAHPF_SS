/*
 * instance definition
 */
#pragma once
#include <random>

#include "graph.hpp"
#include "utils.hpp"

struct lacamInstance {
  const Graph G;  // graph
  Config starts;  // initial configuration
  Config goals;   // goal configuration
  const uint N;   // number of agents

  // for testing
  lacamInstance(const std::string& map_filename,
           const std::vector<int>& start_indexes,
           const std::vector<int>& goal_indexes);
  // for MAPF benchmark
  lacamInstance(const std::string& scen_filename, const std::string& map_filename,
           const int _N = 1);
  // random instance generation
  lacamInstance(const std::string& map_filename, std::mt19937* MT, const int _N = 1);
  lacamInstance(Graph _G, Config _starts, Config _goals): G(_G), N(_starts.size()) {
      for (int i = 0; i < N; i++){
          starts.push_back(_starts[i]);
          goals.push_back(_goals[i]);
      }
  };
  void checkSanity(){
      std::cout<<"N: "<<N<<std::endl;
      std::cout<<"G: "<<G.V.size()<<","<<G.U.size()<<std::endl;
      for (int v=0;v<G.V.size();v++){
          std::cout<<"V: "<<G.V[v]->id<<std::endl;
      }

      std::cout<<"Starts: "<<starts.size()<<std::endl;
      std::cout<<"goals: "<<goals.size()<<std::endl;
      for (int i = 0; i < N; i++){
          std::cout<<"Starts: "<<starts[i]->id<<std::endl;
          std::cout<<"goals: "<<goals[i]->id<<std::endl;
      }

  }
  ~lacamInstance() {}

  // simple feasibility check of instance
  bool is_valid(const int verbose = 0) const;
};

// solution: a sequence of configurations
using Solution = std::vector<Config>;
