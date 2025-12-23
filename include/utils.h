#pragma once
#include <cmath>

// Earth radius in meters
constexpr double R = 6371000.0;
constexpr double TO_RAD = M_PI / 180.0;

// Returns distance in meters
inline double haversine_distance(double lat1, double lon1, double lat2, double lon2) {
    // Convert degrees to radians
    double dLat = (lat2 - lat1) * TO_RAD;
    double dLon = (lon2 - lon1) * TO_RAD;
    
    // The Formula
    double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
               std::cos(lat1 * TO_RAD) * std::cos(lat2 * TO_RAD) *
               std::sin(dLon / 2) * std::sin(dLon / 2);
    
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    
    return R * c;
}