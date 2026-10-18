#include "Airports.h"

#include <cmath>

namespace br {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusKm = 6371.0;

// Close enough at a few kilometres: the flat-earth distance.
double distanceKm(double lat1, double lon1, double lat2, double lon2) {
    double dLon = lon2 - lon1;
    if (dLon > 180.0) dLon -= 360.0;
    if (dLon < -180.0) dLon += 360.0;
    const double x = dLon * std::cos((lat1 + lat2) * 0.5 * kPi / 180.0);
    const double y = lat2 - lat1;
    return std::sqrt(x * x + y * y) * kPi / 180.0 * kEarthRadiusKm;
}

bool icaoLike(const std::string& ident) {
    if (ident.size() != 4) return false;
    for (const char c : ident) {
        if (c < 'A' || c > 'Z') return false;
    }
    return true;
}

}  // namespace

void AirportList::add(const std::string& ident, const std::string& region, double latDeg, double lonDeg) {
    if (ident.empty() || !std::isfinite(latDeg) || !std::isfinite(lonDeg)) return;
    airports_[region + ":" + ident] = Entry{ident, latDeg, lonDeg};
}

std::string AirportList::at(double latDeg, double lonDeg) const {
    if (!std::isfinite(latDeg) || !std::isfinite(lonDeg)) return std::string();
    const Entry* best = nullptr;
    double bestScore = 0.0;
    for (const auto& kv : airports_) {
        const Entry& a = kv.second;
        const double d = distanceKm(latDeg, lonDeg, a.lat, a.lon);
        if (d > kMaxKm) continue;
        const double score = d + (icaoLike(a.ident) ? 0.0 : kLocalIdentPenaltyKm);
        if (!best || score < bestScore || (score == bestScore && a.ident < best->ident)) {
            best = &a;
            bestScore = score;
        }
    }
    return best ? best->ident : std::string();
}

}  // namespace br
