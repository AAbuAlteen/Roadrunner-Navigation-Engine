#include <iostream>
#include <osmium/io/pbf_input.hpp>
#include <osmium/visitor.hpp>
#include "osm_handler.h"
#include "graph.h"
#include "dijkstra_router.h"
#include "a_star_router.h"
#include "bidirectional_router.h"

int main() {
    // 1. Setup File Path
    std::string pbf_file = "../data/israel.pbf"; // Ensure this matches your folder

    // 2. Setup Data Structures
    Graph my_graph;
    // Reserve memory to prevent re-allocations (Estimation: 500k nodes)
    my_graph.resize(500000);

    // 3. Setup Osmium Handlers
    // Index: Stores coordinates in RAM
    IndexType index;
    // LocationHandler: Fills the index
    LocationHandler location_handler(index);
    // GraphBuilder: Builds our graph using the index
    GraphBuilder builder(my_graph);

    std::cout << "Reading OSM Data from " << pbf_file << "..." << std::endl;

    // 4. Run the Reader
    // We request Nodes (for location) and Ways (for edges)
    osmium::io::Reader reader(pbf_file, osmium::osm_entity_bits::node | osmium::osm_entity_bits::way);

    // Apply BOTH handlers at once.
    // Libosmium is smart: it passes Nodes to location_handler first, then Ways to builder.
    osmium::apply(reader, location_handler, builder);

    reader.close();

    // 5. Output Stats
    std::cout << "---------------------------------" << std::endl;
    std::cout << "Graph Build Complete!" << std::endl;
    std::cout << "Nodes: " << my_graph.nodes.size() << std::endl;

    long long edge_count = 0;
    for(const auto& edges : my_graph.adj_list) edge_count += edges.size();
    std::cout << "Edges: " << edge_count << std::endl;
    std::cout << "---------------------------------" << std::endl;

// --- Define Nodes Once ---
    int start_node = 0;
    int end_node = 1000;
    if (end_node >= my_graph.nodes.size()) end_node = my_graph.nodes.size() - 1;

    std::cout << "\n=== BENCHMARK ROUND (Node " << start_node << " -> " << end_node << ") ===" << std::endl;

    // ---------------------------------------------------------
    // 1. Standard Dijkstra
    // ---------------------------------------------------------
    {
        std::cout << "Running Standard Dijkstra..." << std::endl;
        auto start_time = std::chrono::high_resolution_clock::now();

        DijkstraRouter router(my_graph); // Ensure class is named DijkstraRouter in header
        std::vector<int> path;
        double distance = router.find_path(start_node, end_node, path);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        std::cout << "  -> Distance: " << distance / 1000.0 << " km" << std::endl;
        std::cout << "  -> Time: " << duration << " ms" << std::endl;
    }

    // ---------------------------------------------------------
    // 2. A* Search
    // ---------------------------------------------------------
    {
        std::cout << "Running A* Search..." << std::endl;
        auto start_time = std::chrono::high_resolution_clock::now();

        AStarRouter router(my_graph); // Ensure class is named AStarRouter in header
        std::vector<int> path;
        double distance = router.find_path(start_node, end_node, path);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        std::cout << "  -> Distance: " << distance / 1000.0 << " km" << std::endl;
        std::cout << "  -> Time: " << duration << " ms" << std::endl;
    }

    // ---------------------------------------------------------
    // 3. Bidirectional Dijkstra
    // ---------------------------------------------------------
    {
        std::cout << "Running Bidirectional Dijkstra..." << std::endl;
        auto start_time = std::chrono::high_resolution_clock::now();

        BidirectionalRouter router(my_graph);
        std::vector<int> path;
        double distance = router.find_path(start_node, end_node, path);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        std::cout << "  -> Distance: " << distance / 1000.0 << " km" << std::endl;
        std::cout << "  -> Time: " << duration << " ms" << std::endl;
    }

    return 0;
}