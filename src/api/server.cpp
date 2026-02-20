#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>
#include <utility>
#include <stdexcept>
#include <filesystem>
#include <unordered_map>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <limits>
#include <fstream>

#include "third_party/httplib.h"
#include "graph/route_engine.h"

// -----------------------------
// Minimal JSON string escape
// -----------------------------
static std::string json_escape(const std::string& s) {
  std::string out;
  out.reserve(s.size() + 8);
  for (char c : s) {
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"':  out += "\\\""; break;
      case '\n': out += "\\n";  break;
      case '\r': out += "\\r";  break;
      case '\t': out += "\\t";  break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          std::ostringstream oss;
          oss << "\\u00" << std::hex << std::setw(2) << std::setfill('0')
              << (int)(unsigned char)c;
          out += oss.str();
        } else {
          out += c;
        }
    }
  }
  return out;
}

// -----------------------------
// Tiny JSON helper: "key":"value"
// -----------------------------
static std::string json_find_string_value(const std::string& json, const std::string& key) {
  const std::string needle = "\"" + key + "\":\"";
  size_t pos = json.find(needle);
  if (pos == std::string::npos) return "";
  pos += needle.size();

  std::string val;
  val.reserve(128);

  bool escape = false;
  for (size_t i = pos; i < json.size(); i++) {
    char c = json[i];
    if (escape) {
      switch (c) {
        case '"': val.push_back('"'); break;
        case '\\': val.push_back('\\'); break;
        case '/': val.push_back('/'); break;
        case 'n': val.push_back('\n'); break;
        case 'r': val.push_back('\r'); break;
        case 't': val.push_back('\t'); break;
        default: val.push_back(c); break;
      }
      escape = false;
      continue;
    }
    if (c == '\\') { escape = true; continue; }
    if (c == '"') break;
    val.push_back(c);
  }
  return val;
}

// -----------------------------
// Safe param parsing helpers
// -----------------------------
static bool parse_double_param(const httplib::Request& req, const char* key, double& out) {
  if (!req.has_param(key)) return false;
  try {
    out = std::stod(req.get_param_value(key));
    return std::isfinite(out);
  } catch (...) {
    return false;
  }
}

static bool parse_int_param(const httplib::Request& req, const char* key, int& out) {
  if (!req.has_param(key)) return false;
  try {
    const std::string v = req.get_param_value(key);
    size_t idx = 0;
    long long n = std::stoll(v, &idx, 10);
    if (idx != v.size()) return false;
    if (n < std::numeric_limits<int>::min() || n > std::numeric_limits<int>::max()) return false;
    out = static_cast<int>(n);
    return true;
  } catch (...) {
    return false;
  }
}

static bool in_latlon_range(double lat, double lon) {
  return lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0;
}

// -----------------------------
// Reverse geocode via Nominatim (HTTPS w/ fallback)
// -----------------------------
static std::string reverse_geocode_label(double lat, double lon) {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  httplib::SSLClient cli("nominatim.openstreetmap.org", 443);
  cli.set_follow_location(true);
  cli.enable_server_certificate_verification(true);

  namespace fs = std::filesystem;
  const char* candidates[] = {
    "/etc/ssl/cert.pem",
    "/usr/local/etc/openssl@3/cert.pem",
    "/opt/homebrew/etc/openssl@3/cert.pem",
    "/usr/local/etc/openssl/cert.pem",
    "/opt/homebrew/etc/openssl/cert.pem"
  };

  bool set_ca = false;
  for (auto* p : candidates) {
    if (fs::exists(p)) { cli.set_ca_cert_path(p); set_ca = true; break; }
  }
  if (!set_ca) cli.enable_server_certificate_verification(false);
#else
  httplib::Client cli("http://nominatim.openstreetmap.org");
  cli.set_follow_location(true);
#endif

  cli.set_default_headers({
    {"User-Agent", "tritonroute/1.0 (local dev)"},
    {"Accept", "application/json"}
  });

  std::ostringstream path;
  path << "/reverse?format=jsonv2"
       << "&lat=" << std::fixed << std::setprecision(7) << lat
       << "&lon=" << std::fixed << std::setprecision(7) << lon
       << "&zoom=18&addressdetails=1";

  auto res = cli.Get(path.str().c_str());
  if (!res) throw std::runtime_error("reverse geocode failed: no HTTP response");
  if (res->status != 200) {
    std::ostringstream oss;
    oss << "reverse geocode failed: HTTP " << res->status;
    throw std::runtime_error(oss.str());
  }

  std::string display = json_find_string_value(res->body, "display_name");
  if (!display.empty()) return display;

  std::string name = json_find_string_value(res->body, "name");
  if (!name.empty()) return name;

  return "Unknown location";
}

// -----------------------------
// Polyline simplify (RDP) in meters
// Works on lat/lon by projecting to local meters around ref_lat.
// -----------------------------
struct XY { double x, y; };

static inline double deg2rad(double d) { return d * M_PI / 180.0; }

static XY project_m(double lat, double lon, double ref_lat) {
  // equirectangular projection
  constexpr double R = 6371000.0;
  const double x = R * deg2rad(lon) * std::cos(deg2rad(ref_lat));
  const double y = R * deg2rad(lat);
  return {x, y};
}

static double point_seg_dist2(const XY& p, const XY& a, const XY& b) {
  const double vx = b.x - a.x, vy = b.y - a.y;
  const double wx = p.x - a.x, wy = p.y - a.y;
  const double vv = vx*vx + vy*vy;
  double t = (vv > 0) ? (wx*vx + wy*vy) / vv : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  const double px = a.x + t*vx, py = a.y + t*vy;
  const double dx = p.x - px, dy = p.y - py;
  return dx*dx + dy*dy;
}

static std::vector<std::pair<double,double>> simplify_rdp_m(
  const std::vector<std::pair<double,double>>& latlon,
  double eps_m
) {
  if (eps_m <= 0.0 || latlon.size() <= 2) return latlon;

  // reference latitude for projection
  double ref_lat = 0.0;
  for (auto& p : latlon) ref_lat += p.first;
  ref_lat /= (double)latlon.size();

  std::vector<XY> pts;
  pts.reserve(latlon.size());
  for (auto& ll : latlon) pts.push_back(project_m(ll.first, ll.second, ref_lat));

  const double eps2 = eps_m * eps_m;

  std::vector<char> keep(latlon.size(), 0);
  keep.front() = 1;
  keep.back() = 1;

  // iterative stack of segments [i,j]
  std::vector<std::pair<size_t,size_t>> st;
  st.push_back({0, latlon.size()-1});

  while (!st.empty()) {
    auto [i, j] = st.back();
    st.pop_back();
    if (j <= i + 1) continue;

    const XY a = pts[i];
    const XY b = pts[j];

    double best_d2 = -1.0;
    size_t best_k = i;

    for (size_t k = i + 1; k < j; ++k) {
      const double d2 = point_seg_dist2(pts[k], a, b);
      if (d2 > best_d2) { best_d2 = d2; best_k = k; }
    }

    if (best_d2 > eps2) {
      keep[best_k] = 1;
      st.push_back({i, best_k});
      st.push_back({best_k, j});
    }
  }

  std::vector<std::pair<double,double>> out;
  out.reserve(latlon.size());
  for (size_t i = 0; i < latlon.size(); ++i) if (keep[i]) out.push_back(latlon[i]);

  if (out.size() < 2) return latlon; // safety
  return out;
}

// -----------------------------
// GeoJSON FeatureCollection (LineString only) + friendly labels + simp/points
// -----------------------------
static std::string to_geojson_featurecollection(
  const std::vector<std::pair<double,double>>& path_latlon,
  const RouteStats& st,
  const std::string& start_label,
  const std::string& end_label,
  int simp_m
) {
  const int points = static_cast<int>(path_latlon.size());

  std::ostringstream out;
  out << std::fixed << std::setprecision(6);

  out << "{";
  out << "\"type\":\"FeatureCollection\",";
  out << "\"features\":[";

  out << "{"
      << "\"type\":\"Feature\","
      << "\"geometry\":{\"type\":\"LineString\",\"coordinates\":[";
  for (size_t i = 0; i < path_latlon.size(); i++) {
    const auto& [lat, lon] = path_latlon[i];
    out << "[" << lon << "," << lat << "]";
    if (i + 1 < path_latlon.size()) out << ",";
  }
  out << "]},"
      << "\"properties\":{"
      << "\"distance_km\":" << st.distance_km << ","
      << "\"runtime_ms\":"  << st.runtime_ms  << ","
      << "\"hops\":"        << st.hops        << ","
      << "\"start_osm\":"   << st.start_osm   << ","
      << "\"end_osm\":"     << st.end_osm     << ","
      << "\"start_label\":\"" << json_escape(start_label) << "\","
      << "\"end_label\":\""   << json_escape(end_label)   << "\","
      << "\"simp_m\":" << simp_m << ","
      << "\"points\":" << points;

  if (st.travel_time_min >= 0.0) {
    out << ",\"travel_time_min\":" << st.travel_time_min;
  }

  out << "}"
      << "}";

  out << "]}";
  return out.str();
}

// -----------------------------
// tiny static file helper
// -----------------------------
static bool read_file(const std::string& path, std::string& out) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  std::ostringstream ss;
  ss << in.rdbuf();
  out = ss.str();
  return true;
}

int main() {
  httplib::Server svr;
  RouteEngine engine("data/out");

  // ---- in-memory label cache ----
  static std::unordered_map<long long, std::string> label_cache;
  static std::mutex label_mu;

  auto get_label_cached = [&](long long osm_id, double lat, double lon) -> std::string {
    {
      std::lock_guard<std::mutex> lk(label_mu);
      auto it = label_cache.find(osm_id);
      if (it != label_cache.end()) return it->second;
    }
    std::string label = reverse_geocode_label(lat, lon);
    {
      std::lock_guard<std::mutex> lk(label_mu);
      label_cache[osm_id] = label;
    }
    return label;
  };

  // ---- FINAL POLISH: serve map from server ----
  auto serve_map = [&](const httplib::Request&, httplib::Response& res) {
    res.set_header("Cache-Control", "no-store");
    std::string html;
    if (!read_file("data/out/map.html", html)) {
      res.status = 404;
      res.set_content("missing data/out/map.html\n", "text/plain");
      return;
    }
    res.set_content(html, "text/html; charset=utf-8");
  };
  svr.Get("/", serve_map);
  svr.Get("/map.html", serve_map);

  // optional convenience
  svr.Get("/route.geojson", [&](const httplib::Request&, httplib::Response& res) {
    res.set_header("Cache-Control", "no-store");
    std::string body;
    if (!read_file("data/out/route.geojson", body)) {
      res.status = 404;
      res.set_content("missing data/out/route.geojson\n", "text/plain");
      return;
    }
    res.set_content(body, "application/json");
  });

  svr.Get("/health", [](const httplib::Request&, httplib::Response& res) {
    res.set_content("ok\n", "text/plain");
  });

  // GET /reverse?lat=...&lon=...
  svr.Get("/reverse", [&](const httplib::Request& req, httplib::Response& res) {
    res.set_header("Access-Control-Allow-Origin", "*");
    res.set_header("Cache-Control", "no-store");

    double lat=0, lon=0;
    if (!parse_double_param(req, "lat", lat) || !parse_double_param(req, "lon", lon)) {
      res.status = 400;
      res.set_content("{\"error\":\"invalid or missing parameters: lat, lon\"}", "application/json");
      return;
    }
    if (!in_latlon_range(lat, lon)) {
      res.status = 400;
      res.set_content("{\"error\":\"lat/lon out of range\"}", "application/json");
      return;
    }

    try {
      double slat = lat, slon = lon;
      long long osm_id = engine.snap(lat, lon, &slat, &slon);
      std::string label = get_label_cached(osm_id, slat, slon);
      std::string body = std::string("{\"label\":\"") + json_escape(label) + "\"}";
      res.set_content(body, "application/json");
    } catch (const std::exception& e) {
      res.status = 500;
      std::string msg = std::string("{\"error\":\"") + json_escape(e.what()) + "\"}";
      res.set_content(msg, "application/json");
    }
  });

  // GET /route?slat=...&slon=...&elat=...&elon=...&simp_m=...
  svr.Get("/route", [&](const httplib::Request& req, httplib::Response& res) {
    res.set_header("Access-Control-Allow-Origin", "*");
    res.set_header("Cache-Control", "no-store");

    double slat=0, slon=0, elat=0, elon=0;
    if (!parse_double_param(req, "slat", slat) ||
        !parse_double_param(req, "slon", slon) ||
        !parse_double_param(req, "elat", elat) ||
        !parse_double_param(req, "elon", elon)) {
      res.status = 400;
      res.set_content("{\"error\":\"invalid or missing parameters: slat, slon, elat, elon\"}", "application/json");
      return;
    }
    if (!in_latlon_range(slat, slon) || !in_latlon_range(elat, elon)) {
      res.status = 400;
      res.set_content("{\"error\":\"start/end lat/lon out of range\"}", "application/json");
      return;
    }

    int simp_m = 0;
    if (req.has_param("simp_m")) {
      if (!parse_int_param(req, "simp_m", simp_m)) {
        res.status = 400;
        res.set_content("{\"error\":\"invalid simp_m (must be an integer)\"}", "application/json");
        return;
      }
    }
    simp_m = std::clamp(simp_m, 0, 200);

    try {
      double sslat = slat, sslon = slon, eelat = elat, eelon = elon;
      long long s_id = engine.snap(slat, slon, &sslat, &sslon);
      long long e_id = engine.snap(elat, elon, &eelat, &eelon);

      RouteResult rr = engine.route(s_id, e_id);
      if (rr.path_latlon.empty()) {
        res.status = 404;
        res.set_content("{\"error\":\"no route found\"}", "application/json");
        return;
      }

      rr.stats.start_osm = s_id;
      rr.stats.end_osm   = e_id;

      std::string start_label = "Start";
      std::string end_label   = "End";
      try { start_label = get_label_cached(s_id, sslat, sslon); } catch (...) {}
      try { end_label   = get_label_cached(e_id, eelat, eelon); } catch (...) {}

      // ✅ APPLY SIMPLIFICATION HERE (this is the missing piece)
      const auto simplified = simplify_rdp_m(rr.path_latlon, (double)simp_m);

      std::string geojson = to_geojson_featurecollection(
        simplified, rr.stats, start_label, end_label, simp_m
      );
      res.set_content(geojson, "application/json");
    } catch (const std::exception& e) {
      res.status = 500;
      std::string msg = std::string("{\"error\":\"") + json_escape(e.what()) + "\"}";
      res.set_content(msg, "application/json");
    }
  });

  std::cout << "Server running on http://localhost:8080\n";
  svr.listen("0.0.0.0", 8080);
}