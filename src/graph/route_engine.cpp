#include "graph/route_engine.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// -------- geo helpers --------
static inline double deg2rad(double d) { return d * M_PI / 180.0; }

static double haversine_m(double lat1, double lon1, double lat2, double lon2) {
  constexpr double R = 6371000.0;
  const double dlat = deg2rad(lat2 - lat1);
  const double dlon = deg2rad(lon2 - lon1);
  const double a =
      std::sin(dlat / 2) * std::sin(dlat / 2) +
      std::cos(deg2rad(lat1)) * std::cos(deg2rad(lat2)) *
      std::sin(dlon / 2) * std::sin(dlon / 2);
  const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
  return R * c;
}

// -------- CSR loader (v2) --------
static bool load_csr_v2(const std::string& path, RouteEngine::CSR& g) {
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

  in.read(reinterpret_cast<char*>(g.off.data()), sizeof(uint64_t) * (g.N + 1));
  in.read(reinterpret_cast<char*>(g.to.data()),  sizeof(uint32_t) * g.M);
  in.read(reinterpret_cast<char*>(g.w.data()),   sizeof(float)    * g.M);

  if (!in) return false;
  if (g.off.empty() || g.off.back() != g.M) return false;
  return true;
}

static std::unordered_map<uint64_t, uint32_t> build_osm_to_idx(const RouteEngine::CSR& g) {
  std::unordered_map<uint64_t, uint32_t> mp;
  mp.reserve(static_cast<size_t>(g.N * 1.3));
  for (uint32_t i = 0; i < static_cast<uint32_t>(g.N); ++i) mp[g.node_ids[i]] = i;
  return mp;
}

// -------- A* on "seconds" weights --------
struct PQItem {
  double f;
  uint32_t v;
  bool operator>(const PQItem& o) const { return f > o.f; }
};

static bool astar_seconds(
  const RouteEngine::CSR& g,
  uint32_t s,
  uint32_t t,
  std::vector<uint32_t>& parent_out,
  double& dist_out_seconds
) {
  const double INF = std::numeric_limits<double>::infinity();
  std::vector<double> gscore(g.N, INF);
  std::vector<uint32_t> parent(g.N, UINT32_MAX);

  // heuristic in seconds (meters / max_speed)
  constexpr double MAX_SPEED_MPS = 33.33; // ~120 km/h bound

  auto h = [&](uint32_t v) -> double {
    const double meters = haversine_m(g.lat[v], g.lon[v], g.lat[t], g.lon[t]);
    return meters / MAX_SPEED_MPS;
  };

  std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> pq;
  gscore[s] = 0.0;
  pq.push({h(s), s});

  while (!pq.empty()) {
    const auto cur = pq.top();
    pq.pop();
    const uint32_t v = cur.v;

    if (v == t) break;

    const uint64_t begin = g.off[v];
    const uint64_t end   = g.off[v + 1];

    for (uint64_t ei = begin; ei < end; ++ei) {
      const uint32_t u = g.to[ei];
      const double cand = gscore[v] + static_cast<double>(g.w[ei]); // seconds
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

static double path_distance_m(const RouteEngine::CSR& g, const std::vector<uint32_t>& path) {
  if (path.size() < 2) return 0.0;
  double total = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const uint32_t a = path[i - 1];
    const uint32_t b = path[i];
    total += haversine_m(g.lat[a], g.lon[a], g.lat[b], g.lon[b]);
  }
  return total;
}

// -------- grid helpers (for fast snap) --------
static inline uint64_t pack_i32(int32_t x, int32_t y) {
  return (uint64_t(uint32_t(x)) << 32) | uint64_t(uint32_t(y));
}

static inline void latlon_to_xy_m(double lat_deg, double lon_deg, double lat0_rad,
                                 double& out_x, double& out_y) {
  constexpr double R = 6371000.0;
  const double lat = deg2rad(lat_deg);
  const double lon = deg2rad(lon_deg);
  // equirectangular projection around lat0
  out_x = R * lon * std::cos(lat0_rad);
  out_y = R * lat;
}

// -------- RouteEngine --------
RouteEngine::RouteEngine(const std::string& data_dir) {
  const std::string bin = data_dir + "/graph_csr.bin";
  if (!load_csr_v2(bin, g_)) {
    throw std::runtime_error("Failed to load CSR graph: " + bin);
  }
  osm2idx_ = build_osm_to_idx(g_);
  build_grid_index_();
}

void RouteEngine::build_grid_index_() {
  if (g_.N == 0) return;

  // pick a stable reference latitude (mean-ish)
  double sum = 0.0;
  for (uint32_t i = 0; i < (uint32_t)g_.N; ++i) sum += g_.lat[i];
  grid_lat0_rad_ = deg2rad(sum / (double)g_.N);

  grid_.clear();
  grid_.reserve((size_t)(g_.N * 0.25)); // buckets << nodes

  for (uint32_t i = 0; i < (uint32_t)g_.N; ++i) {
    double x, y;
    latlon_to_xy_m(g_.lat[i], g_.lon[i], grid_lat0_rad_, x, y);
    const int32_t gx = (int32_t)std::floor(x / grid_cell_m_);
    const int32_t gy = (int32_t)std::floor(y / grid_cell_m_);
    grid_[pack_i32(gx, gy)].push_back(i);
  }
}

uint32_t RouteEngine::snap_grid_idx_(double qlat, double qlon) const {
  if (g_.N == 0) throw std::runtime_error("Graph is empty");

  double qx, qy;
  latlon_to_xy_m(qlat, qlon, grid_lat0_rad_, qx, qy);
  const int32_t qgx = (int32_t)std::floor(qx / grid_cell_m_);
  const int32_t qgy = (int32_t)std::floor(qy / grid_cell_m_);

  uint32_t best = 0;
  double bestd = std::numeric_limits<double>::infinity();

  // scan rings of neighboring cells (0..MAXR)
  // MAXR=6 => ~1200m search radius with 200m cells
  const int MAXR = 6;

  for (int r = 0; r <= MAXR; ++r) {
    bool any = false;

    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        // ring-only to reduce work: skip interior when r>0
        if (r > 0 && std::abs(dx) != r && std::abs(dy) != r) continue;

        const uint64_t key = pack_i32(qgx + dx, qgy + dy);
        auto it = grid_.find(key);
        if (it == grid_.end()) continue;
        any = true;

        for (uint32_t idx : it->second) {
          const double d = haversine_m(qlat, qlon, g_.lat[idx], g_.lon[idx]);
          if (d < bestd) {
            bestd = d;
            best = idx;
          }
        }
      }
    }

    // early exit: if we found something and r is “large enough”
    // (not perfect math, but works great in practice)
    if (any && bestd < (r + 0.5) * grid_cell_m_) break;
  }

  // if grid is somehow empty, fallback to brute force (safety)
  if (!std::isfinite(bestd)) {
    for (uint32_t i = 0; i < (uint32_t)g_.N; ++i) {
      const double d = haversine_m(qlat, qlon, g_.lat[i], g_.lon[i]);
      if (d < bestd) { bestd = d; best = i; }
    }
  }

  return best;
}

long long RouteEngine::snap(double lat, double lon, double* out_lat, double* out_lon) const {
  const uint32_t idx = snap_grid_idx_(lat, lon);
  if (out_lat) *out_lat = g_.lat[idx];
  if (out_lon) *out_lon = g_.lon[idx];
  return (long long)g_.node_ids[idx];
}

RouteResult RouteEngine::route(long long start_osm, long long end_osm) const {
  RouteResult rr;

  auto itS = osm2idx_.find((uint64_t)start_osm);
  auto itT = osm2idx_.find((uint64_t)end_osm);
  if (itS == osm2idx_.end() || itT == osm2idx_.end()) {
    throw std::runtime_error("start or end OSM id not found in graph");
  }

  const uint32_t s = itS->second;
  const uint32_t t = itT->second;

  std::vector<uint32_t> parent;
  double best_time_sec = 0.0;

  const auto t0 = std::chrono::steady_clock::now();
  const bool ok = astar_seconds(g_, s, t, parent, best_time_sec);
  const auto t1 = std::chrono::steady_clock::now();

  rr.stats.runtime_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  rr.stats.start_osm = start_osm;
  rr.stats.end_osm   = end_osm;

  if (!ok) {
    rr.path_latlon.clear();
    rr.stats.hops = 0;
    rr.stats.distance_km = 0.0;
    rr.stats.travel_time_min = -1.0;
    return rr;
  }

  const auto path = reconstruct_path(parent, s, t);
  rr.stats.hops = (int)path.size();

  rr.path_latlon.reserve(path.size());
  for (uint32_t v : path) rr.path_latlon.push_back({g_.lat[v], g_.lon[v]});

  const double dist_m = path_distance_m(g_, path);
  rr.stats.distance_km = dist_m / 1000.0;
  rr.stats.travel_time_min = best_time_sec / 60.0;

  return rr;
}