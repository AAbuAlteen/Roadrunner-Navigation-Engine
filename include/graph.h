#pragma once
#include <vector>
#include "types.h" // Holds your Node/Edge structs we made earlier

struct Graph {
    // 1. The Geometry: Where is Node 'i' located?
    // We access this as: graph.nodes[i].lat
    std::vector<Node> nodes;

    // 2. The Connectivity: Who is connected to Node 'i'?
    // adj_list[i] returns a vector of Edges (target_id, weight)
    std::vector<std::vector<Edge>> adj_list;

    // Helper to reserve memory (Optimization)
    // If we know we have 500k nodes, we reserve space to prevent resizing.
    void resize(size_t n) {
        nodes.reserve(n);
        adj_list.resize(n);
    }

    // Helper to add a connection
    // We add the edge in ONE direction (u -> v).
    // The Handler will call this twice if the road is two-way.
    void add_edge(int u, int v, double weight) {
        adj_list[u].push_back({v, weight});
    }
};