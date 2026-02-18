#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <limits>
#include <string>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <unordered_map>
#include <algorithm>

// ---------- geo helpers ----------
static inline double deg2rad(double d) { return d * M_PI / 180.0; }

static double haversine_m(double lat1, double lon1, double lat2, double lon2) {
    constexpr double R = 6371000.0;
    double dlat = deg2rad(lat2 - lat1);
    double dlon = deg2rad(lon2 - lon1);
    double a = std::sin(dlat/2)*std::sin(dlat/2) +
               std::cos(deg2rad(lat1))*std::cos(deg2rad(lat2)) *
               std::sin(dlon/2)*std::sin(dlon/2);
    double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return R * c;
}

// ---------- CSR ----------
struct CSR {
    uint64_t N = 0;
    uint64_t M = 0;
    std::vector<uint64_t> node_ids;   // idx -> OSM node id
    std::vector<double>   lat;        // idx -> lat
    std::vector<double>   lon;        // idx -> lon
    std::vector<uint64_t> off;        // size N+1
    std::vector<uint32_t> to;         // size M
    std::vector<float>    w;          // size M  (IMPORTANT: assumes SECONDS)
};

static bool load_csr_v2(const std::string& path, CSR& g) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;

    in.read(reinterpret_cast<char*>(&g.N), sizeof(g.N));
    in.read(reinterpret_cast<char*>(&g.M), sizeof(g.M));
    if (!in) return false;

    g.node_ids.resize(g.N);
    g.lat.resize(g.N);
    g.lon.resize(g.N);
    g.off.resize(g.N + 1);
    g.to.resize(g.M);
    g.w.resize(g.M);

    in.read(reinterpret_cast<char*>(g.node_ids.data()), sizeof(uint64_t) * g.N);
    in.read(reinterpret_cast<char*>(g.lat.data()),      sizeof(double)   * g.N);
    in.read(reinterpret_cast<char*>(g.lon.data()),      sizeof(double)   * g.N);

    in.read(reinterpret_cast<char*>(g.off.data()),      sizeof(uint64_t) * (g.N + 1));
    in.read(reinterpret_cast<char*>(g.to.data()),       sizeof(uint32_t) * g.M);
    in.read(reinterpret_cast<char*>(g.w.data()),        sizeof(float)    * g.M);

    if (!in) return false;
    if (g.off.empty() || g.off.back() != g.M) return false;
    return true;
}

static std::unordered_map<uint64_t, uint32_t> build_osm_to_idx(const CSR& g) {
    std::unordered_map<uint64_t, uint32_t> mp;
    mp.reserve(static_cast<size_t>(g.N * 1.3));
    for (uint32_t i = 0; i < static_cast<uint32_t>(g.N); ++i) mp[g.node_ids[i]] = i;
    return mp;
}

// ---------- A* ----------
struct PQItem {
    double f;
    uint32_t v;
    bool operator>(const PQItem& o) const { return f > o.f; }
};

// Returns: true if found path. dist_out is total COST (seconds).
static bool astar_seconds(
    const CSR& g,
    uint32_t s,
    uint32_t t,
    std::vector<uint32_t>& parent_out,
    double& dist_out_seconds
) {
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> gscore(g.N, INF);
    std::vector<uint32_t> parent(g.N, UINT32_MAX);

    // Heuristic must be in SAME UNITS as edge weights.
    // Your w[] is seconds, so h() should return seconds.
    // We approximate by straight-line meters / max_speed_mps.
    constexpr double MAX_SPEED_MPS = 33.33; // ~120 km/h (admissible-ish upper bound)

    auto h = [&](uint32_t v) -> double {
        double meters = haversine_m(g.lat[v], g.lon[v], g.lat[t], g.lon[t]);
        return meters / MAX_SPEED_MPS;
    };

    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;

    gscore[s] = 0.0;
    pq.push({h(s), s});

    while (!pq.empty()) {
        auto cur = pq.top();
        pq.pop();
        uint32_t v = cur.v;

        if (v == t) break;

        uint64_t begin = g.off[v];
        uint64_t end   = g.off[v + 1];

        for (uint64_t ei = begin; ei < end; ++ei) {
            uint32_t u = g.to[ei];
            double cand = gscore[v] + static_cast<double>(g.w[ei]); // seconds
            if (cand < gscore[u]) {
                gscore[u] = cand;
                parent[u] = v;
                pq.push({cand + h(u), u});
            }
        }
    }

    if (!std::isfinite(gscore[t])) return false;
    parent_out = std::move(parent);
    dist_out_seconds = gscore[t];
    return true;
}

static std::vector<uint32_t> reconstruct_path(
    const std::vector<uint32_t>& parent,
    uint32_t s,
    uint32_t t
) {
    std::vector<uint32_t> rev;
    for (uint32_t cur = t; cur != UINT32_MAX; cur = parent[cur]) {
        rev.push_back(cur);
        if (cur == s) break;
    }
    if (rev.empty() || rev.back() != s) return {};
    return std::vector<uint32_t>(rev.rbegin(), rev.rend());
}

static double path_distance_m(const CSR& g, const std::vector<uint32_t>& path) {
    if (path.size() < 2) return 0.0;
    double total = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        uint32_t a = path[i - 1];
        uint32_t b = path[i];
        total += haversine_m(g.lat[a], g.lon[a], g.lat[b], g.lon[b]);
    }
    return total;
}

int main(int argc, char** argv) {
    // Usage:
    //   ./route_graph <start_osm_id> <end_osm_id>
    if (argc != 3) {
        std::cerr << "Usage: ./route_graph <start_osm_id> <end_osm_id>\n";
        return 1;
    }

    uint64_t start_osm = 0, end_osm = 0;
    try {
        start_osm = static_cast<uint64_t>(std::stoull(argv[1]));
        end_osm   = static_cast<uint64_t>(std::stoull(argv[2]));
    } catch (...) {
        std::cerr << "Error: arguments must be integers (OSM node ids).\n";
        return 1;
    }

    const std::string bin = "../data/out/graph_csr.bin";

    CSR g;
    if (!load_csr_v2(bin, g)) {
        std::cerr << "Error: failed to load v2 CSR from " << bin << "\n";
        std::cerr << "Tip: run ./build_graph again, then ./load_graph to verify.\n";
        return 1;
    }

    auto osm2idx = build_osm_to_idx(g);
    auto itS = osm2idx.find(start_osm);
    auto itT = osm2idx.find(end_osm);
    if (itS == osm2idx.end() || itT == osm2idx.end()) {
        std::cerr << "Error: start or end OSM id not found in graph.\n";
        std::cerr << "Tip: use ./load_graph output or your python snippet to grab valid IDs.\n";
        return 1;
    }

    uint32_t s = itS->second;
    uint32_t t = itT->second;

    std::vector<uint32_t> parent;
    double best_time_sec = 0.0;

    auto t0 = std::chrono::steady_clock::now();
    bool ok = astar_seconds(g, s, t, parent, best_time_sec);
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::cout << "N = " << g.N << ", M = " << g.M << "\n";

    if (!ok) {
        std::cout << "No path found.\n";
        std::cout << "Runtime: " << ms << " ms\n";
        return 0;
    }

    auto path = reconstruct_path(parent, s, t);
    if (path.empty()) {
        std::cout << "No path found (reconstruct failed).\n";
        std::cout << "Runtime: " << ms << " ms\n";
        return 0;
    }

    // Compute geometric distance for reporting (km)
    double dist_m = path_distance_m(g, path);
    double dist_km = dist_m / 1000.0;

    // Time reporting
    double time_min = best_time_sec / 60.0;

    std::cout << "Travel time: " << time_min << " minutes\n";
    std::cout << "Distance: " << dist_km << " km\n";
    std::cout << "Hops: " << path.size() << " nodes\n";
    std::cout << "Runtime: " << ms << " ms\n";
    std::cout << "Start OSM: " << start_osm << "\n";
    std::cout << "End OSM:   " << end_osm << "\n";

    // Path preview (don’t spam)
    std::cout << "Path preview (OSM ids):\n";
    size_t preview = std::min<size_t>(path.size(), 15);
    for (size_t i = 0; i < preview; ++i) {
        std::cout << "  " << g.node_ids[path[i]] << "\n";
    }
    if (path.size() > preview) {
        std::cout << "  ... (" << (path.size() - preview) << " more)\n";
    }

    return 0;
}
