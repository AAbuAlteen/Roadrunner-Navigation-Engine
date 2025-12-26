#pragma once
#include <osmium/handler.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/tags/tags_filter.hpp>
#include <osmium/index/map/flex_mem.hpp> // Stores coordinates in RAM
#include <osmium/handler/node_locations_for_ways.hpp> // Looks up coordinates
#include <unordered_map>
#include <string>
#include "graph.h"
#include "utils.h"

// Define the Index Type (Maps OSM ID -> Lat/Lon)
using IndexType = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;
// Define the Handler that fills this index
using LocationHandler = osmium::handler::NodeLocationsForWays<IndexType>;

class GraphBuilder : public osmium::handler::Handler {
    Graph& graph;

    // MAPPING: OSM ID (Huge) -> Internal ID (0, 1, 2...)
    std::unordered_map<int64_t, int> id_map;
    int internal_id_counter = 0;

public:
    explicit GraphBuilder(Graph& g) : graph(g) {}

    int get_or_create_node(int64_t osm_id, const osmium::Location& loc) {
        if (id_map.find(osm_id) == id_map.end()) {
            // New Node found!
            id_map[osm_id] = internal_id_counter;

            // Add to our Graph's Node Vector
            graph.nodes.emplace_back(internal_id_counter, loc.lat(), loc.lon());

            internal_id_counter++;

            // Dynamic resizing for BOTH Adjacency Lists
            if (graph.adj_list.size() <= internal_id_counter) {
                size_t new_size = internal_id_counter + 10000; // Grow in chunks

                graph.adj_list.resize(new_size);
                graph.reverse_adj_list.resize(new_size); // <--- ADD THIS LINE
            }
        }
        return id_map[osm_id];
    }

    // The Main Event: Processing a Road
    void way(const osmium::Way& way) {
        const char* highway = way.tags()["highway"];
        if (!highway) return; // Not a road

        // Filter: Only allow cars (ignore footways, cycleways)
        std::string type = highway;
        if (type != "motorway" && type != "trunk" && type != "primary" &&
            type != "secondary" && type != "tertiary" && type != "residential" &&
            type != "unclassified") {
            return;
        }

        // Iterate through the nodes in this road
        for (size_t i = 0; i < way.nodes().size() - 1; ++i) {
            const auto& n1 = way.nodes()[i];
            const auto& n2 = way.nodes()[i+1];

            // Ensure both nodes have valid coordinates (looked up from Index)
            if (n1.location().valid() && n2.location().valid()) {

                // 1. Get Integers IDs
                int u = get_or_create_node(n1.ref(), n1.location());
                int v = get_or_create_node(n2.ref(), n2.location());

                // 2. Calculate Distance (Weight)
                double dist = haversine_distance(n1.location().lat(), n1.location().lon(),
                                               n2.location().lat(), n2.location().lon());

                // 3. Add Edges (Explicitly Bidirectional)
                // Forward: u -> v
                graph.add_edge(u, v, dist);
                // Backward: v -> u
                graph.add_edge(v, u, dist);
            }
        }
    }
};