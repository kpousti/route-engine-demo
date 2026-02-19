#include <osmium/io/any_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/visitor.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/node.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <stdexcept>

// ---------------- geo helpers ----------------
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

// ---------------- road filters ----------------
static bool is_drivable_highway(const osmium::Way& w) {
    const char* h = w.tags()["highway"];
    if (!h) return false;
    const std::string hs(h);
    return (hs == "motorway" ||
            hs == "trunk" ||
            hs == "primary" ||
            hs == "secondary" ||
            hs == "tertiary" ||
            hs == "unclassified" ||
            hs == "residential" ||
            hs == "living_street" ||
            hs == "service");
}

// ---------------- speed + oneway helpers ----------------
// minimal, practical parser: grabs the first number in "35", "35 mph", "50 km/h", "signals", etc.
static double parse_maxspeed_to_mps(const char* s) {
    if (!s) return -1.0;
    std::string v(s);
    // common junk values
    if (v == "signals" || v == "none" || v == "walk" || v == "variable") return -1.0;

    // find first digit
    size_t i = 0;
    while (i < v.size() && !(v[i] >= '0' && v[i] <= '9')) i++;
    if (i == v.size()) return -1.0;

    // parse number (possibly with decimal)
    size_t j = i;
    while (j < v.size() && ((v[j] >= '0' && v[j] <= '9') || v[j] == '.')) j++;
    double num = 0.0;
    try {
        num = std::stod(v.substr(i, j - i));
    } catch (...) {
        return -1.0;
    }

    // unit detection
    // if it mentions mph -> mph, else assume km/h (OSM default is usually km/h unless explicitly mph)
    bool mph = (v.find("mph") != std::string::npos) || (v.find("mp/h") != std::string::npos);
    if (mph) return num * 0.44704;       // mph -> m/s
    return num * (1000.0 / 3600.0);      // km/h -> m/s
}

static double default_speed_mps_from_highway(const osmium::Way& w) {
    const char* h = w.tags()["highway"];
    if (!h) return 13.89; // 50 km/h fallback
    const std::string hs(h);

    // simple, sane defaults (tweak later)
    if (hs == "motorway")      return 30.56; // 110 km/h
    if (hs == "trunk")         return 25.00; // 90 km/h
    if (hs == "primary")       return 22.22; // 80 km/h
    if (hs == "secondary")     return 19.44; // 70 km/h
    if (hs == "tertiary")      return 16.67; // 60 km/h
    if (hs == "unclassified")  return 13.89; // 50 km/h
    if (hs == "residential")   return 11.11; // 40 km/h
    if (hs == "living_street") return  5.56; // 20 km/h
    if (hs == "service")       return  8.33; // 30 km/h
    return 13.89;
}

static double speed_mps_for_way(const osmium::Way& w) {
    // if maxspeed exists, use it; otherwise highway default
    double ms = parse_maxspeed_to_mps(w.tags()["maxspeed"]);
    if (ms > 0.0) return ms;
    return default_speed_mps_from_highway(w);
}

// Return: 0 = bidirectional, +1 = forward only, -1 = reverse only
static int oneway_dir(const osmium::Way& w) {
    const char* o = w.tags()["oneway"];
    if (!o) return 0;

    std::string v(o);
    for (auto& c : v) c = (char)std::tolower(c);

    if (v == "yes" || v == "true" || v == "1") return +1;
    if (v == "-1" || v == "reverse") return -1;
    if (v == "no" || v == "false" || v == "0") return 0;

    return 0;
}

// ---------------- pass 1 ----------------
struct CollectNodeIDs : public osmium::handler::Handler {
    std::unordered_set<osmium::object_id_type> needed;
    void way(const osmium::Way& w) {
        if (!is_drivable_highway(w)) return;
        for (const auto& nr : w.nodes()) needed.insert(nr.ref());
    }
};

// ---------------- pass 2 ----------------
struct Coord { double lat, lon; };

struct StoreCoords : public osmium::handler::Handler {
    const std::unordered_set<osmium::object_id_type>& needed;
    std::unordered_map<osmium::object_id_type, Coord> coords;

    explicit StoreCoords(const std::unordered_set<osmium::object_id_type>& n) : needed(n) {
        coords.reserve(needed.size());
    }

    void node(const osmium::Node& n) {
        auto id = n.id();
        if (needed.find(id) == needed.end()) return;
        if (!n.location()) return;
        coords[id] = { n.location().lat(), n.location().lon() };
    }
};

// ---------------- pass 3 (CSR build) ----------------
struct EdgeRec { uint32_t from, to; float w_sec; };

int main() {
    try {
        const std::string pbf_path = "../data/region.osm.pbf";
        const std::string out_path = "../data/out/graph_csr.bin";

        // PASS 1
        osmium::io::Reader r1(pbf_path);
        CollectNodeIDs pass1;
        osmium::apply(r1, pass1);
        r1.close();
        std::cout << "Pass1: needed node IDs = " << pass1.needed.size() << "\n";

        // PASS 2
        osmium::io::Reader r2(pbf_path);
        StoreCoords pass2(pass1.needed);
        osmium::apply(r2, pass2);
        r2.close();
        std::cout << "Pass2: coords stored = " << pass2.coords.size() << "\n";

        // Build compact index using ONLY nodes that have coords
        std::vector<uint64_t> node_ids;
        node_ids.reserve(pass2.coords.size());
        for (const auto& kv : pass2.coords) node_ids.push_back((uint64_t)kv.first);
        std::sort(node_ids.begin(), node_ids.end());

        std::unordered_map<osmium::object_id_type, uint32_t> id_to_idx;
        id_to_idx.reserve((size_t)(node_ids.size() * 1.3));
        for (uint32_t i = 0; i < (uint32_t)node_ids.size(); ++i) {
            id_to_idx[(osmium::object_id_type)node_ids[i]] = i;
        }

        const uint64_t N = node_ids.size();
        std::cout << "Index: N = " << N << "\n";

        // Build lat/lon arrays aligned with node_ids[]
        std::vector<double> lat(N), lon(N);
        for (uint64_t i = 0; i < N; ++i) {
            auto it = pass2.coords.find((osmium::object_id_type)node_ids[i]);
            if (it == pass2.coords.end()) throw std::runtime_error("Internal: coord missing for node id");
            lat[i] = it->second.lat;
            lon[i] = it->second.lon;
        }

        // PASS 3: build directed edges with weight = travel_time_seconds
        std::vector<uint64_t> deg(N, 0);
        std::vector<EdgeRec> edges;
        edges.reserve(3'000'000);

        struct Builder : public osmium::handler::Handler {
            const std::unordered_map<osmium::object_id_type, Coord>& coords;
            const std::unordered_map<osmium::object_id_type, uint32_t>& id_to_idx;
            std::vector<uint64_t>& deg;
            std::vector<EdgeRec>& edges;

            uint64_t ways_used = 0;
            uint64_t directed_edges_added = 0;

            Builder(const std::unordered_map<osmium::object_id_type, Coord>& c,
                    const std::unordered_map<osmium::object_id_type, uint32_t>& m,
                    std::vector<uint64_t>& d,
                    std::vector<EdgeRec>& e)
                : coords(c), id_to_idx(m), deg(d), edges(e) {}

            void add_dir(uint32_t a, uint32_t b, float sec) {
                deg[a]++;
                edges.push_back({a, b, sec});
                directed_edges_added++;
            }

            void way(const osmium::Way& w0) {
                if (!is_drivable_highway(w0)) return;
                if (w0.nodes().size() < 2) return;

                ++ways_used;

                const int od = oneway_dir(w0);         // 0, +1, -1
                const double speed = speed_mps_for_way(w0); // m/s

                auto it = w0.nodes().begin();
                osmium::object_id_type prev_id = it->ref();
                ++it;

                for (; it != w0.nodes().end(); ++it) {
                    osmium::object_id_type cur_id = it->ref();

                    auto p = coords.find(prev_id);
                    auto q = coords.find(cur_id);
                    if (p != coords.end() && q != coords.end()) {
                        auto ip = id_to_idx.find(prev_id);
                        auto iq = id_to_idx.find(cur_id);

                        if (ip != id_to_idx.end() && iq != id_to_idx.end()) {
                            double meters = haversine_m(p->second.lat, p->second.lon,
                                                       q->second.lat, q->second.lon);
                            double sec = meters / std::max(0.1, speed); // avoid divide by 0
                            float wsec = (float)sec;

                            // od meaning:
                            // 0  => both directions
                            // +1 => follow node order prev->cur only
                            // -1 => reverse only cur->prev
                            if (od == 0) {
                                add_dir(ip->second, iq->second, wsec);
                                add_dir(iq->second, ip->second, wsec);
                            } else if (od == +1) {
                                add_dir(ip->second, iq->second, wsec);
                            } else { // -1
                                add_dir(iq->second, ip->second, wsec);
                            }
                        }
                    }

                    prev_id = cur_id;
                }
            }
        };

        osmium::io::Reader r3(pbf_path);
        Builder builder(pass2.coords, id_to_idx, deg, edges);
        osmium::apply(r3, builder);
        r3.close();

        const uint64_t M = edges.size();
        std::cout << "Pass3: ways used = " << builder.ways_used << "\n";
        std::cout << "Graph: directed edges added = " << builder.directed_edges_added << "\n";

        // Build offsets
        std::vector<uint64_t> offsets(N + 1, 0);
        for (uint64_t i = 0; i < N; ++i) offsets[i + 1] = offsets[i] + deg[i];
        if (offsets[N] != M) throw std::runtime_error("Degree sum mismatch vs edges.size()");

        // Pack CSR
        std::vector<uint64_t> cursor = offsets;
        std::vector<uint32_t> csr_to(M);
        std::vector<float> csr_w(M);

        for (const auto& e : edges) {
            uint64_t pos = cursor[e.from]++;
            csr_to[pos] = e.to;
            csr_w[pos]  = e.w_sec;
        }

        double avg_deg = (N == 0) ? 0.0 : (double)M / (double)N;
        std::cout << "Graph: N=" << N << " M=" << M << " avg out-deg=" << avg_deg << "\n";

        std::filesystem::create_directories("../data/out");

        // Write v2 binary WITH coords (same layout your route_graph loader expects)
        {
            std::ofstream out(out_path, std::ios::binary);
            if (!out) throw std::runtime_error("Failed to open output: " + out_path);

            auto write_u64 = [&](uint64_t v){ out.write((const char*)&v, sizeof(v)); };

            write_u64(N);
            write_u64(M);

            out.write((const char*)node_ids.data(), sizeof(uint64_t) * N);
            out.write((const char*)lat.data(),     sizeof(double)   * N);
            out.write((const char*)lon.data(),     sizeof(double)   * N);

            out.write((const char*)offsets.data(), sizeof(uint64_t) * (N + 1));
            out.write((const char*)csr_to.data(),  sizeof(uint32_t) * M);
            out.write((const char*)csr_w.data(),   sizeof(float)    * M);

            if (!out) throw std::runtime_error("Write failed (disk?)");
        }

        std::cout << "Wrote CSR graph (+coords, time weights) to: " << out_path << "\n";
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}