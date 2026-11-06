// CoreTests.exe --fit-trace <trace.csv> [from to]: StickFit on a replay trace (kPlaybackTrace), the
// same as a replay fits it, for each surface the stick loop steered; prints each model. For seeding
// an aircraft's figures from its replays, and for checking the fit against them.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "../src/core/StickFit.h"

namespace {

std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) out.push_back(cell);
    return out;
}

double num(const std::vector<std::string>& row, int col) {
    if (col < 0 || col >= static_cast<int>(row.size()) || row[col].empty()) return NAN;
    char* end = nullptr;
    const double v = std::strtod(row[col].c_str(), &end);
    return end == row[col].c_str() ? NAN : v;
}

}  // namespace

int traceFitMain(int argc, char** argv) {
    if (argc < 3) {
        std::printf("usage: CoreTests.exe --fit-trace <trace.csv> [from to]\n");
        return 2;
    }
    const double from = argc > 4 ? std::atof(argv[3]) : -1e9;
    const double to = argc > 4 ? std::atof(argv[4]) : 1e9;
    std::ifstream in(argv[2]);
    if (!in) {
        std::printf("can't open %s\n", argv[2]);
        return 2;
    }
    std::string line;
    std::getline(in, line);
    const std::vector<std::string> head = split(line);
    std::map<std::string, int> col;
    for (size_t i = 0; i < head.size(); ++i) col[head[i]] = static_cast<int>(i);
    const auto c = [&](const char* name) {
        const auto it = col.find(name);
        return it == col.end() ? -1 : it->second;
    };
    struct Axis {
        const char* name;
        const char* on;
        const char* stick;
        const char* surface;
    };
    const Axis axes[] = {
        {"elevator", "stick_on", "stick", "elev_live"},
        {"ailerons", "ail_on", "ail_stick", "ail_l_live"},
        {"rudder", "rud_on", "rud_stick", "rud_live"},
    };
    std::vector<br::StickFit> fits(3, br::StickFit(1e9));
    const double kDegPerRad = 57.29577951308232;
    while (std::getline(in, line)) {
        const std::vector<std::string> row = split(line);
        if (num(row, c("playing")) != 1.0) continue;
        const double dt = num(row, c("wall_dt_ms")) / 1000.0;
        if (!(dt > 0.0)) continue;  // the loop didn't step
        const double t = num(row, c("clip_t"));
        for (size_t k = 0; k < fits.size(); ++k) {
            const Axis& a = axes[k];
            br::StickSample s;
            s.dt = dt;
            s.stick = num(row, c(a.stick));
            s.surface = num(row, c(a.surface)) * kDegPerRad;
            s.use = num(row, c(a.on)) == 1.0 && num(row, c("ongnd_live")) < 0.5 && num(row, c("jumped")) != 1.0 &&
                    std::fabs(s.stick) < 0.97 && t >= from && t <= to;
            fits[k].add(s);
        }
    }
    for (size_t k = 0; k < fits.size(); ++k) {
        const br::StickPlant p = fits[k].fit();
        std::printf("%-9s %5.0f s usable: ", axes[k].name, fits[k].usableSec());
        if (!p.valid()) {
            std::printf("no fit\n");
            continue;
        }
        std::printf("%-6s %6.1f %s, %.2f s late, settling %.2f s, fit %.2f\n", br::stickPlantForm(p.form), p.gain,
                    p.form == br::StickPlant::Form::Rate ? "deg/s" : "deg  ", p.lagSec, p.settleSec, p.fit);
    }
    return 0;
}
