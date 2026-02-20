#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <unordered_map>

struct RouteStats {
  double distance_km = 0.0;
  double runtime_ms  = 0.0;
  int hops           = 0;
  long long start_osm = -1;
  long long end_osm   = -1;
  double travel_time_min = -1.0;
};

struct RouteResult {
  std::vector<std::pair<double,double>> path_latlon; // lat,lon
  RouteStats stats;
};

class RouteEngine {
public:
  struct CSR {
    uint64_t N = 0;
    uint64_t M = 0;
    std::vector<uint64_t> node_ids;   // idx -> OSM node id
    std::vector<double>   lat;        // idx -> lat
    std::vector<double>   lon;        // idx -> lon
    std::vector<uint64_t> off;        // size N+1
    std::vector<uint32_t> to;         // size M
    std::vector<float>    w;          // size M (SECONDS)
  };

  explicit RouteEngine(const std::string& data_dir);

  long long snap(double lat, double lon, double* out_lat, double* out_lon) const;
  RouteResult route(long long start_osm, long long end_osm) const;

private:
  // ---- graph ----
  CSR g_;
  std::unordered_map<uint64_t, uint32_t> osm2idx_;

  // ---- snap accel: grid buckets ----
  // grid cell size in meters (tune 100–300m; 200m is a good start)
  double grid_cell_m_ = 200.0;
  double grid_lat0_rad_ = 0.0; // reference latitude for x-scaling

  // key: packed (gx,gy) -> list of node indices
  std::unordered_map<uint64_t, std::vector<uint32_t>> grid_;

  void build_grid_index_();
  uint32_t snap_grid_idx_(double qlat, double qlon) const;
};