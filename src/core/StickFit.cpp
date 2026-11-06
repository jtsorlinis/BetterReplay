#include "StickFit.h"

#include <algorithm>
#include <cmath>

namespace br {

const char* stickPlantForm(StickPlant::Form f) {
    switch (f) {
        case StickPlant::Form::Rate: return "rate";
        case StickPlant::Form::Direct: return "direct";
        case StickPlant::Form::None: break;
    }
    return "none";
}

namespace {

constexpr int kSlope = 2;  // the surface's rate: from 2 frames either side

bool solve2(double m00, double m01, double m11, double v0, double v1, double& x0, double& x1) {
    const double det = m00 * m11 - m01 * m01;
    if (!(std::fabs(det) > 1e-12 * (m00 * m11 + 1e-30))) return false;
    x0 = (v0 * m11 - v1 * m01) / det;
    x1 = (m00 * v1 - m01 * v0) / det;
    return true;
}

}  // namespace

void StickFit::add(const StickSample& s) {
    Row r;
    const double dt = std::isfinite(s.dt) && s.dt > 0.0 ? std::fmin(s.dt, 0.25) : 1.0 / 30.0;
    r.t = rows_.empty() ? 0.0 : rows_.back().t + dt;
    r.s = s.stick;
    r.d = s.surface;
    r.use = s.use && std::isfinite(s.stick) && std::isfinite(s.surface);
    rows_.push_back(r);
    // The oldest go a few seconds' worth at a time.
    const double keepFrom = r.t - keepSec_;
    if (rows_.front().t < keepFrom - 5.0) {
        const auto it = std::find_if(rows_.begin(), rows_.end(), [keepFrom](const Row& x) { return x.t >= keepFrom; });
        rows_.erase(rows_.begin(), it);
    }
}

double StickFit::usableSec() const {
    double sec = 0.0;
    for (size_t i = 1; i < rows_.size(); ++i) {
        if (rows_[i].use) sec += rows_[i].t - rows_[i - 1].t;
    }
    return sec;
}

StickPlant StickFit::fit() const {
    StickPlant none;
    const size_t n = rows_.size();
    if (n < 100 || usableSec() < kMinSec) return none;
    std::vector<double> dts;
    dts.reserve(n);
    for (size_t i = 1; i < n; ++i) dts.push_back(rows_[i].t - rows_[i - 1].t);
    std::nth_element(dts.begin(), dts.begin() + dts.size() / 2, dts.end());
    const double dtm = std::fmax(dts[dts.size() / 2], 1e-3);
    const int half = std::max(3, static_cast<int>(std::lround(kSmoothSec / dtm)));
    const int maxLag = std::min(static_cast<int>(std::lround(kMaxLagSec / dtm)), 48);
    const int window = std::max(10, static_cast<int>(std::lround(kWindowSec / dtm)));

    // The quick movement: each less its average over `half` frames either side, where all of those
    // are usable (ok[i]).
    std::vector<double> qs(n, 0.0), qd(n, 0.0);
    std::vector<char> ok(n, 0);
    {
        double ss = 0.0, sd = 0.0;
        int bad = 0;
        const int w = 2 * half + 1;
        for (size_t i = 0; i < n; ++i) {
            ss += rows_[i].use ? rows_[i].s : 0.0;
            sd += rows_[i].use ? rows_[i].d : 0.0;
            bad += rows_[i].use ? 0 : 1;
            if (i >= static_cast<size_t>(w)) {
                const Row& out = rows_[i - w];
                ss -= out.use ? out.s : 0.0;
                sd -= out.use ? out.d : 0.0;
                bad -= out.use ? 0 : 1;
            }
            if (i + 1 < static_cast<size_t>(w)) continue;
            const size_t mid = i - half;
            if (bad != 0) continue;
            qs[mid] = rows_[mid].s - ss / w;
            qd[mid] = rows_[mid].d - sd / w;
            ok[mid] = 1;
        }
    }
    double sq = 0.0;
    int used = 0;
    for (size_t i = 0; i < n; ++i) {
        if (!ok[i]) continue;
        sq += qs[i] * qs[i];
        ++used;
    }
    if (used < 100 || std::sqrt(sq / used) < kMinQuickStick) return none;

    StickPlant best;
    best.fit = -1e9;
    const size_t first = static_cast<size_t>(maxLag + kSlope);
    for (int form = 0; form < 2; ++form) {
        const bool rate = form == 0;
        for (int lag = 0; lag <= maxLag; ++lag) {
            const auto usable = [&](size_t i) {
                if (!ok[i] || !ok[i - lag]) return false;
                return !rate || (i + kSlope < n && ok[i - kSlope] && ok[i + kSlope]);
            };
            double m00 = 0.0, m01 = 0.0, m11 = 0.0, v0 = 0.0, v1 = 0.0;
            int rows = 0;
            for (size_t i = first; i + kSlope < n; ++i) {
                if (!usable(i)) continue;
                const double x0 = qs[i - lag];
                if (rate) {
                    const double span = rows_[i + kSlope].t - rows_[i - kSlope].t;
                    if (!(span > 0.0)) continue;
                    const double y = (qd[i + kSlope] - qd[i - kSlope]) / span;
                    const double x1 = qd[i];
                    m00 += x0 * x0;
                    m01 += x0 * x1;
                    m11 += x1 * x1;
                    v0 += x0 * y;
                    v1 += x1 * y;
                } else {
                    m00 += x0 * x0;
                    v0 += x0 * qd[i];
                }
                ++rows;
            }
            if (rows < 60 || !(m00 > 0.0)) continue;
            double a = 0.0, b = 0.0;
            if (rate) {
                if (!solve2(m00, m01, m11, v0, v1, a, b)) continue;
            } else {
                a = v0 / m00;
            }
            if (!(a > 0.5) || a > (rate ? 300.0 : 120.0) || b > 1.0 || b < -60.0) continue;
            // How well it reproduces the quick movement from where each stretch begins.
            double err = 0.0, var = 0.0;
            for (size_t i0 = first; i0 + window + kSlope < n; i0 += window / 2) {
                bool good = true;
                for (size_t i = i0; i < i0 + window && good; ++i) good = ok[i] && ok[i - lag];
                if (!good) continue;
                double sim = qd[i0];
                for (size_t i = i0; i < i0 + window; ++i) {
                    if (!rate) sim = a * qs[i - lag];
                    err += (sim - qd[i]) * (sim - qd[i]);
                    var += qd[i] * qd[i];
                    if (rate) sim += (rows_[i + 1].t - rows_[i].t) * (a * qs[i - lag] + b * sim);
                }
            }
            if (!(var > 0.0)) continue;
            const double fit = 1.0 - std::sqrt(err / var);
            if (fit <= best.fit) continue;
            best.fit = fit;
            best.lagSec = lag * dtm;
            if (rate && b < 0.0 && -1.0 / b < kDirectBelowSec) {
                best.form = StickPlant::Form::Direct;  // settles at a position, quickly
                best.gain = -a / b;
                best.settleSec = -1.0 / b;
            } else {
                best.form = rate ? StickPlant::Form::Rate : StickPlant::Form::Direct;
                best.gain = a;
                best.settleSec = 0.0;
            }
        }
    }
    if (!(best.fit >= kMinFit) || !best.valid()) return none;
    best.seconds = usableSec();
    return best;
}

}  // namespace br
