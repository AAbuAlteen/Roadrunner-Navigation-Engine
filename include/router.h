#pragma once
#include "graph.h"
#include <queue>
#include <limits>
#include <algorithm> // for std::reverse
#include <vector>

// We store {distance, node_id} in the queue
using PQElement = std::pair<double, int>;

class Router {
    const Graph& graph;

public:
    // Constructor just stores the reference
    Router(const Graph& g) : graph(g) {}

    // Returns distance in meters. Fills 'path' vector with Node IDs.
    double find_path(int start_node, int end_node, std::vector<int>& path) {
        int n = graph.nodes.size();
        
        // 1. Initialization
        // 'dist' array: Holds the shortest known distance to every node
        std::vector<double> dist(n, std::numeric_limits<double>::infinity());
        
        // 'parent' array: To reconstruct the path later (who did we come from?)
        std::vector<int> parent(n, -1);
        
        // Min-Priority Queue
        std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq;

        // 2. Start
        dist[start_node] = 0;
        pq.push({0, start_node}); // Distance 0 to start

        // 3. The Main Loop
        while(!pq.empty()) {
            // Get the closest node
            double current_dist = pq.top().first;
            int u = pq.top().second;
            pq.pop();

            // Optimization: If we found the target, stop early!
            if (u == end_node) break;

            // Optimization: Lazy Deletion
            // If we found a shorter path to 'u' earlier, ignore this old entry
            if (current_dist > dist[u]) continue;

            // Explore neighbors
            for (const auto& edge : graph.adj_list[u]) {
                int v = edge.target;
                double weight = edge.weight;

                // Relaxation Step
                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    parent[v] = u; // Record path
                    pq.push({dist[v], v});
                }
            }
        }

        // 4. Reconstruct Path
        if (dist[end_node] == std::numeric_limits<double>::infinity()) {
            return -1.0; // No path found
        }

        // Backtrack from End -> Start
        int curr = end_node;
        while (curr != -1) {
            path.push_back(curr);
            curr = parent[curr];
        }
        // Reverse it to get Start -> End
        std::reverse(path.begin(), path.end());

        return dist[end_node];
    }
};