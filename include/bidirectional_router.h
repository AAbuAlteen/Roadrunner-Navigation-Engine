#pragma once
#include "graph.h"
#include <queue>
#include <vector>
#include <limits>
#include <algorithm>

using PQElement = std::pair<double, int>;

class BidirectionalRouter {
    const Graph& graph;

    // Memory Pools
    std::vector<double> dist_fwd;
    std::vector<double> dist_rev;
    std::vector<int> parent_fwd;
    std::vector<int> parent_rev;

    // Optimization: Track which nodes we dirtied to clean them fast
    std::vector<int> touched_nodes;

public:
    BidirectionalRouter(const Graph& g) : graph(g) {
        int n = graph.nodes.size();
        dist_fwd.resize(n, std::numeric_limits<double>::infinity());
        dist_rev.resize(n, std::numeric_limits<double>::infinity());
        parent_fwd.resize(n, -1);
        parent_rev.resize(n, -1);
        touched_nodes.reserve(n); // Pre-allocate just in case
    }

    double find_path(int start, int end, std::vector<int>& path) {
        double inf = std::numeric_limits<double>::infinity();

        // 1. LAZY CLEANUP (Only clean what we used last time)
        for (int node : touched_nodes) {
            dist_fwd[node] = inf;
            dist_rev[node] = inf;
            parent_fwd[node] = -1;
            parent_rev[node] = -1;
        }
        touched_nodes.clear();

        // 2. Initialization
        dist_fwd[start] = 0;
        dist_rev[end] = 0;

        // Mark start/end as touched so we clean them next time
        touched_nodes.push_back(start);
        touched_nodes.push_back(end);

        std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq_fwd;
        std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq_rev;

        pq_fwd.push({0, start});
        pq_rev.push({0, end});

        double mu = inf;
        int meeting_node = -1;

        // 3. The Loop
        while (!pq_fwd.empty() && !pq_rev.empty()) {

            if (pq_fwd.top().first + pq_rev.top().first >= mu) break;

            expand(graph.adj_list, pq_fwd, dist_fwd, parent_fwd, dist_rev, mu, meeting_node);
            expand(graph.reverse_adj_list, pq_rev, dist_rev, parent_rev, dist_fwd, mu, meeting_node);
        }

        // 4. Reconstruct Path
        if (meeting_node == -1) return -1.0;

        int curr = meeting_node;
        while (curr != -1) {
            path.push_back(curr);
            curr = parent_fwd[curr];
        }
        std::reverse(path.begin(), path.end());

        curr = parent_rev[meeting_node];
        while (curr != -1) {
            path.push_back(curr);
            curr = parent_rev[curr];
        }

        return mu;
    }

private:
    void expand(const std::vector<std::vector<Edge>>& adj,
                std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>>& pq,
                std::vector<double>& dist,
                std::vector<int>& parent,
                const std::vector<double>& other_dist,
                double& mu,
                int& meeting_node) {

        if (pq.empty()) return;

        double d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (d > dist[u]) return;

        for (const auto& edge : adj[u]) {
            int v = edge.target;
            double weight = edge.weight;

            if (dist[u] + weight < dist[v]) {
                // If this is the first time we touch v, add to clean list
                if (dist[v] == std::numeric_limits<double>::infinity()) {
                    touched_nodes.push_back(v);
                }

                dist[v] = dist[u] + weight;
                parent[v] = u;
                pq.push({dist[v], v});

                if (other_dist[v] != std::numeric_limits<double>::infinity()) {
                    double new_path = dist[v] + other_dist[v];
                    if (new_path < mu) {
                        mu = new_path;
                        meeting_node = v;
                    }
                }
            }
        }
    }
};