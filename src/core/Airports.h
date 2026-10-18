// The airports around the flight, so takeoff and landing clips can say where they were.
#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>

namespace br {

// What the sim's facilities cache reported (the airports within some distance of the aircraft),
// kept for the whole flight: a takeoff clip is saved minutes after liftoff, when the departure
// airport may have dropped out of the cache.
class AirportList {
public:
    // The farthest an airport's reference point is from a liftoff or touchdown on it. The end of
    // a long runway is 2 to 3 km from it.
    static constexpr double kMaxKm = 8.0;
    // Idents that aren't four letters (heliports and private strips: "34CA", "WA12", "0S9") count
    // as this much further away, so a heliport by the runway doesn't win over the airport.
    static constexpr double kLocalIdentPenaltyKm = 3.0;

    void add(const std::string& ident, const std::string& region, double latDeg, double lonDeg);
    void clear() { airports_.clear(); }
    size_t size() const { return airports_.size(); }

    // The ident of the airport at (lat, lon), or "" if none is near enough.
    std::string at(double latDeg, double lonDeg) const;

private:
    struct Entry {
        std::string ident;
        double lat;
        double lon;
    };
    std::unordered_map<std::string, Entry> airports_;  // by region and ident
};

}  // namespace br
