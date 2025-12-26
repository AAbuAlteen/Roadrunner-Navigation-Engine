#pragma once
#include <vector>
#include "types.h"

struct Graph {
    // 1. Geometry
    std::vector<Node> nodes;

    // 2. Forward Connectivity (Start -> End)
    // adj_list[u] = edges leaving u
    std::vector<std::vector<Edge>> adj_list;

    // 3. Backward Connectivity (End -> Start) -> NEW!
    // reverse_adj_list[v] = edges entering v
    std::vector<std::vector<Edge>> reverse_adj_list;

    void resize(size_t n) {
        nodes.reserve(n);
        adj_list.resize(n);
        reverse_adj_list.resize(n); // Resize the new vector
    }

    void add_edge(int u, int v, double weight) {

        // --- DEBUG: Bounds Check ---
        if (u >= adj_list.size() || v >= reverse_adj_list.size()) {
            std::cerr << "CRASH IMMINENT! Index out of bounds." << std::endl;
            std::cerr << "Trying to access Node: " << u << " (or " << v << ")" << std::endl;
            std::cerr << "Max Size: " << adj_list.size() << " / " << reverse_adj_list.size() << std::endl;
            exit(1); // Stop immediately
        }

        // Forward edge: u -> v
        adj_list[u].push_back({v, weight});

        // Reverse edge: v -> u
        reverse_adj_list[v].push_back({u, weight});
    }
};