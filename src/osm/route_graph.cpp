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
#include <filesystem>
#include <algorithm>

static inline double deg2rad(double d) { return d * M_PI / 180.0; }

// Haversine distance in meters
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

// ---------- CSR v2 (with coords) ----------
struct CSR {
    uint64_t N = 0;
    uint64_t M = 0;
    std::vector<uint64_t> node_ids;   // idx -> OSM
    std::vector<double>   lat;        // idx -> lat
    std::vector<double>   lon;        // idx -> lon
    std::vector<uint64_t> off;        // N+1
    std::vector<uint32_t> to;         // M
    std::vector<float>    w;          // M (meters)
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
    if (g.off.back() != g.M) return false;
    return true;
}

static std::unordered_map<uint64_t, uint32_t> build_osm_to_idx(const CSR& g) {
    std::unordered_map<uint64_t, uint32_t> mp;
    mp.reserve((size_t)(g.N * 1.3));
    for (uint32_t i = 0; i < (uint32_t)g.N; ++i) mp[g.node_ids[i]] = i;
    return mp;
}

// brute-force nearest node (good enough for demo)
// later we can upgrade to a grid/kd-tree if you want
static uint32_t snap_nearest(const CSR& g, double qlat, double qlon, double& out_dist_m) {
    uint32_t best = 0;
    double bestd = std::numeric_limits<double>::infinity();

    for (uint32_t i = 0; i < (uint32_t)g.N; ++i) {
        double d = haversine_m(qlat, qlon, g.lat[i], g.lon[i]);
        if (d < bestd) {
            bestd = d;
            best = i;
        }
    }
    out_dist_m = bestd;
    return best;
}

// ---------- A* ----------
struct PQItem {
    double f;
    uint32_t v;
    bool operator>(const PQItem& o) const { return f > o.f; }
};

static bool astar(
    const CSR& g,
    uint32_t s,
    uint32_t t,
    std::vector<uint32_t>& parent_out,
    double& dist_out
) {
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> gscore(g.N, INF);
    std::vector<uint32_t> parent(g.N, UINT32_MAX);

    auto h = [&](uint32_t v) {
        return haversine_m(g.lat[v], g.lon[v], g.lat[t], g.lon[t]);
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
            double cand = gscore[v] + (double)g.w[ei]; // meters
            if (cand < gscore[u]) {
                gscore[u] = cand;
                parent[u] = v;
                pq.push({cand + h(u), u});
            }
        }
    }

    if (!std::isfinite(gscore[t])) return false;
    parent_out = std::move(parent);
    dist_out = gscore[t];
    return true;
}

static std::vector<uint32_t> reconstruct_path(const std::vector<uint32_t>& parent, uint32_t s, uint32_t t) {
    std::vector<uint32_t> rev;
    for (uint32_t cur = t; cur != UINT32_MAX; cur = parent[cur]) {
        rev.push_back(cur);
        if (cur == s) break;
    }
    if (rev.empty() || rev.back() != s) return {};
    return std::vector<uint32_t>(rev.rbegin(), rev.rend());
}

static bool write_geojson(
    const std::string& out_path,
    const CSR& g,
    const std::vector<uint32_t>& path,
    uint32_t s, uint32_t t,
    double dist_m,
    double runtime_ms
) {
    std::filesystem::create_directories(std::filesystem::path(out_path).parent_path());

    std::ofstream out(out_path);
    if (!out) return false;

    // FeatureCollection with:
    //  - LineString route
    //  - Point start
    //  - Point end
    out << "{\n";
    out << "  \"type\": \"FeatureCollection\",\n";
    out << "  \"features\": [\n";

    // Route LineString
    out << "    {\n";
    out << "      \"type\": \"Feature\",\n";
    out << "      \"properties\": {\n";
    out << "        \"distance_m\": " << dist_m << ",\n";
    out << "        \"distance_km\": " << (dist_m / 1000.0) << ",\n";
    out << "        \"hops\": " << path.size() << ",\n";
    out << "        \"runtime_ms\": " << runtime_ms << ",\n";
    out << "        \"start_osm\": " << g.node_ids[s] << ",\n";
    out << "        \"end_osm\": " << g.node_ids[t] << "\n";
    out << "      },\n";
    out << "      \"geometry\": {\n";
    out << "        \"type\": \"LineString\",\n";
    out << "        \"coordinates\": [\n";

    for (size_t i = 0; i < path.size(); ++i) {
        uint32_t v = path[i];
        // GeoJSON expects [lon, lat]
        out << "          [" << g.lon[v] << ", " << g.lat[v] << "]";
        out << (i + 1 == path.size() ? "\n" : ",\n");
    }

    out << "        ]\n";
    out << "      }\n";
    out << "    },\n";

    // Start point
    out << "    {\n";
    out << "      \"type\": \"Feature\",\n";
    out << "      \"properties\": {\"kind\": \"start\", \"osm\": " << g.node_ids[s] << "},\n";
    out << "      \"geometry\": {\"type\": \"Point\", \"coordinates\": [" << g.lon[s] << ", " << g.lat[s] << "]}\n";
    out << "    },\n";

    // End point
    out << "    {\n";
    out << "      \"type\": \"Feature\",\n";
    out << "      \"properties\": {\"kind\": \"end\", \"osm\": " << g.node_ids[t] << "},\n";
    out << "      \"geometry\": {\"type\": \"Point\", \"coordinates\": [" << g.lon[t] << ", " << g.lat[t] << "]}\n";
    out << "    }\n";

    out << "  ]\n";
    out << "}\n";

    return true;
}

int main(int argc, char** argv) {
    // Usage:
    //   OSM ids:
    //     ./route_graph <start_osm_id> <end_osm_id>
    //
    //   Lat/Lon:
    //     ./route_graph <start_lat> <start_lon> <end_lat> <end_lon>

    const std::string bin = "../data/out/graph_csr.bin";
    CSR g;

    if (!load_csr_v2(bin, g)) {
        std::cerr << "Error: failed to load v2 CSR from " << bin << "\n";
        std::cerr << "Tip: run ./build_graph then ./load_graph.\n";
        return 1;
    }

    uint32_t s = UINT32_MAX, t = UINT32_MAX;
    bool used_latlon = false;

    double snap_s_m = 0.0, snap_t_m = 0.0;

    if (argc == 3) {
        // OSM ids mode
        uint64_t start_osm = 0, end_osm = 0;
        try {
            start_osm = (uint64_t)std::stoull(argv[1]);
            end_osm   = (uint64_t)std::stoull(argv[2]);
        } catch (...) {
            std::cerr << "Error: arguments must be integers (OSM node ids).\n";
            return 1;
        }

        auto osm2idx = build_osm_to_idx(g);
        auto itS = osm2idx.find(start_osm);
        auto itT = osm2idx.find(end_osm);
        if (itS == osm2idx.end() || itT == osm2idx.end()) {
            std::cerr << "Error: start or end OSM id not found in graph.\n";
            std::cerr << "Tip: run ./load_graph to see a valid node0 OSM id.\n";
            return 1;
        }
        s = itS->second;
        t = itT->second;

    } else if (argc == 5) {
        // Lat/Lon mode
        used_latlon = true;
        double slat, slon, tlat, tlon;
        try {
            slat = std::stod(argv[1]);
            slon = std::stod(argv[2]);
            tlat = std::stod(argv[3]);
            tlon = std::stod(argv[4]);
        } catch (...) {
            std::cerr << "Error: lat/lon args must be numbers.\n";
            return 1;
        }

        s = snap_nearest(g, slat, slon, snap_s_m);
        t = snap_nearest(g, tlat, tlon, snap_t_m);

        std::cout << "Snap start: (" << slat << ", " << slon << ") -> idx " << s
                  << " OSM " << g.node_ids[s] << " (snap " << snap_s_m << " m)\n";
        std::cout << "Snap end:   (" << tlat << ", " << tlon << ") -> idx " << t
                  << " OSM " << g.node_ids[t] << " (snap " << snap_t_m << " m)\n";

    } else {
        std::cerr << "Usage:\n";
        std::cerr << "  ./route_graph <start_osm_id> <end_osm_id>\n";
        std::cerr << "  ./route_graph <start_lat> <start_lon> <end_lat> <end_lon>\n";
        return 1;
    }

    std::vector<uint32_t> parent;
    double dist_m = 0.0;

    auto t0 = std::chrono::steady_clock::now();
    bool ok = astar(g, s, t, parent, dist_m);
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::cout << "N = " << g.N << ", M = " << g.M << "\n";

    if (!ok) {
        std::cout << "No path found.\n";
        std::cout << "Runtime: " << ms << " ms\n";
        return 0;
    }

    auto path = reconstruct_path(parent, s, t);

    std::cout << "Cost (sum w): " << dist_m << " meters (" << (dist_m / 1000.0) << " km)\n";
    std::cout << "Hops: " << path.size() << " nodes\n";
    std::cout << "Runtime: " << ms << " ms\n";
    std::cout << "Start OSM: " << g.node_ids[s] << "\n";
    std::cout << "End OSM:   " << g.node_ids[t] << "\n";

    // Preview
    std::cout << "Path preview (OSM ids):\n";
    size_t preview = std::min<size_t>(path.size(), 15);
    for (size_t i = 0; i < preview; ++i) {
        std::cout << "  " << g.node_ids[path[i]] << "\n";
    }
    if (path.size() > preview) std::cout << "  ... (" << (path.size() - preview) << " more)\n";

    // GeoJSON export
    const std::string geo_out = "../data/out/route.geojson";
    if (write_geojson(geo_out, g, path, s, t, dist_m, ms)) {
        std::cout << "Wrote GeoJSON: " << geo_out << "\n";
        if (used_latlon) {
            std::cout << "(Includes snapped start/end points)\n";
        }
    } else {
        std::cerr << "Warning: failed to write GeoJSON to " << geo_out << "\n";
    }

    return 0;
}
