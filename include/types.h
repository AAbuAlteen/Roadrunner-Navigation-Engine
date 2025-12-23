#pragma once
#include <cstdint>

// Use strong types for IDs so we don't mix them up
using NodeID = int64_t; // OSM IDs are huge (e.g., 4592012)
using InternalID = int32_t; // Our internal array index (0, 1, 2...)

struct Node {
    double lat;
    double lon;
    InternalID id;

    // Constructor
    Node(InternalID i, double x, double y) : id(i), lat(x), lon(y) {}
};

struct Edge {
    InternalID target;
    double weight; // Travel time in seconds
};