#pragma once
#include "graph.h"
#include <queue>
#include <limits>
#include <algorithm>
#include <vector>

// {Distance from Start, NodeID}
using PQElement = std::pair<double, int>;

class DijkstraRouter {
    const Graph& graph;

public:
    DijkstraRouter(const Graph& g) : graph(g) {}

    // Standard Dijkstra Search
    double find_path(int start, int end, std::vector<int>& path) {
        int n = graph.nodes.size();

        // g_score: The exact cost from start to current node
        std::vector<double> g_score(n, std::numeric_limits<double>::infinity());

        // parent: To reconstruct path
        std::vector<int> parent(n, -1);

        // Min-Priority Queue (orders by Distance)
        std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq;

        // Initialize Start
        g_score[start] = 0.0;
        pq.push({0.0, start});

        while(!pq.empty()) {
            double current_dist = pq.top().first;
            int u = pq.top().second;
            pq.pop();

            // 1. Target Reached?
            if (u == end) break;

            // 2. Lazy Deletion (Optimization)
            // If we found a shorter path to 'u' already, skip this stale entry
            if (current_dist > g_score[u]) continue;

            // 3. Explore Neighbors
            for (const auto& edge : graph.adj_list[u]) {
                int v = edge.target;
                double weight = edge.weight;
                double new_dist = g_score[u] + weight;

                if (new_dist < g_score[v]) {
                    // Found a better path to v!
                    parent[v] = u;
                    g_score[v] = new_dist;

                    // Push with Distance only (No Heuristic)
                    pq.push({new_dist, v});
                }
            }
        }

        // Path Reconstruction
        if (g_score[end] == std::numeric_limits<double>::infinity()) return -1.0;

        int curr = end;
        while (curr != -1) {
            path.push_back(curr);
            curr = parent[curr];
        }
        std::reverse(path.begin(), path.end());

        return g_score[end];
    }
};