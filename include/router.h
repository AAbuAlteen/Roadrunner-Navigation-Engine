#pragma once
#include "graph.h"
#include "utils.h" // Needed for haversine_distance
#include <queue>
#include <limits>
#include <algorithm>
#include <vector>

// {F-Score (g + h), NodeID}
using PQElement = std::pair<double, int>;

class Router {
    const Graph& graph;

public:
    Router(const Graph& g) : graph(g) {}

    // A* Search
    double find_path(int start, int end, std::vector<int>& path) {
        int n = graph.nodes.size();

        // g_score: The exact cost from start to current node
        std::vector<double> g_score(n, std::numeric_limits<double>::infinity());

        // parent: To reconstruct path
        std::vector<int> parent(n, -1);

        // Min-Priority Queue (orders by F-Score)
        std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq;

        // Initialize Start
        g_score[start] = 0.0;

        // Calculate initial heuristic
        double h_start = haversine_distance(
            graph.nodes[start].lat, graph.nodes[start].lon,
            graph.nodes[end].lat, graph.nodes[end].lon
        );

        pq.push({0.0 + h_start, start});

        while(!pq.empty()) {
            double current_f = pq.top().first;
            int u = pq.top().second;
            pq.pop();

            // 1. Target Reached?
            if (u == end) break;

            // 2. Lazy Deletion (Optimization)
            // If we found a better path to 'u' already, skip this stale entry
            // current_f is (g + h), so we compare g part roughly or just check g_score
            // A safer check for A*: if g_score[u] is better than what implies this f_score
            // But standard Dijkstra check is fine:
            // logic: f = g + h. If g_score[u] + h < current_f, we skip.
            double u_h = haversine_distance(
                graph.nodes[u].lat, graph.nodes[u].lon,
                graph.nodes[end].lat, graph.nodes[end].lon
            );
            if (g_score[u] + u_h < current_f) continue;

            // 3. Explore Neighbors
            for (const auto& edge : graph.adj_list[u]) {
                int v = edge.target;
                double weight = edge.weight;
                double tentative_g = g_score[u] + weight;

                if (tentative_g < g_score[v]) {
                    // Found a better path to v!
                    parent[v] = u;
                    g_score[v] = tentative_g;

                    // Heuristic: Straight line distance from V to END
                    double h = haversine_distance(
                        graph.nodes[v].lat, graph.nodes[v].lon,
                        graph.nodes[end].lat, graph.nodes[end].lon
                    );

                    // Push with F-Score (g + h)
                    pq.push({tentative_g + h, v});
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