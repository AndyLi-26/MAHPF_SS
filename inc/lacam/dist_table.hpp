/*
 * distance table with lazy evaluation, using BFS
 */
#pragma once

#include "graph.hpp"
#include "instance.hpp"
#include "utils.hpp"

struct DistTable {
  const int K;  // number of vertices
  std::vector<std::vector<int> >
      table;  // distance table, index: agent-id & vertex-id
  std::vector<std::queue<Vertex*> > OPEN;  // search queue

  int get(int i, int v_id);   // agent, vertex-id
  int get(int i, Vertex* v);  // agent, vertex

  DistTable(const lacamInstance& ins);
  DistTable(const lacamInstance* ins);

  void setup(const lacamInstance* ins);  // initialization
};
