# Route Engine (C++)  
Offline A* Routing + Interactive Geospatial Visualization

A high-performance routing engine built from scratch in modern C++.

This project parses OpenStreetMap data into a compressed graph representation, executes A* shortest-path search with spatial snapping, exposes a hardened HTTP API, and renders interactive routes through a Leaflet frontend with live simplification controls.

Built to deeply understand how real-world routing systems work — not just how to call them.

---

## Why I Built This

Routing systems sit at the intersection of:

- Graph theory  
- Performance engineering  
- Memory layout design  
- API architecture  
- Geospatial reasoning  
- Frontend/backend interaction  

Instead of treating routing like a black box, I built the full pipeline:

OSM → Graph Construction → A* Search → HTTP API → GeoJSON → Interactive UI

This forced me to think about:

- Cache efficiency (CSR-style adjacency layout)
- Heuristic-guided search vs naive traversal
- Deterministic path reconstruction
- Coordinate snapping in geographic space
- API validation and error handling
- Payload size vs rendering cost (simplification)
- Preventing stale client requests under rapid parameter changes
- Git history hygiene for large binary artifacts

The goal wasn’t just correctness — it was understanding system behavior end-to-end.

---

## Architecture

OSM Data  
   ↓  
Graph Builder (C++)  
   ↓  
Compressed Sparse Row Graph  
   ↓  
A* Routing Engine  
   ↓  
HTTP Server  
   ↓  
GeoJSON Response  
   ↓  
Leaflet Frontend  

Core capabilities:

- Graph construction from OpenStreetMap data  
- CSR-style adjacency structure for cache-friendly traversal  
- A* search with haversine heuristic  
- Spatial snapping (lat/lon → nearest node)  
- GeoJSON export  
- Configurable polyline simplification (`simp_m`)  
- Reverse geocode caching  
- Debounced frontend rerouting + request canceling  

---

## Example

`simp_m = 0`   → 82 points  
`simp_m = 50`  → 9 points  

Increasing simplification reduces payload size while preserving topology.

---

## API

### Health
GET `/health`

### Reverse Geocode
GET `/reverse?lat=...&lon=...`

### Route
GET `/route?slat=...&slon=...&elat=...&elon=...&simp_m=...`

Returns a GeoJSON `FeatureCollection` including:

- distance_km  
- runtime_ms  
- hops  
- travel_time_min (if available)  
- simp_m  
- points  
- start_label  
- end_label  

Input validation ensures malformed coordinates and invalid parameters are rejected cleanly.

---

## Build & Run

### Prerequisites

- C++17 compatible compiler  
- CMake ≥ 3.16  
- macOS or Linux  
- OpenSSL (optional, for HTTPS reverse geocoding)  
- OpenStreetMap `.pbf` file for graph generation  

### Build

```bash
cmake -S . -B build
cmake --build build -j
```

### Run

```bash
./build/route_server
```

Server runs at:

http://localhost:8080

Open the interactive demo:

`data/out/map.html`

---

## Design Decisions

- **A\*** reduces search space compared to plain Dijkstra  
- **CSR layout** improves memory locality and traversal speed  
- Backend returns only LineString; UI owns marker state  
- Explicit simplification controls payload and rendering cost  
- Reverse geocode responses are cached in memory  
- Client-side caching prevents duplicate route computation  
- Debounced slider prevents request flooding  

---

## Real-World Applications

This architecture mirrors production systems used in:

- Navigation platforms  
- Delivery and fleet routing  
- Infrastructure planning  
- Network topology traversal  
- Backend geospatial services  

The same algorithmic patterns appear anywhere large graphs require fast, deterministic answers.

---

## Future Improvements

- Spatial index (KD-tree / R-tree) for faster snapping  
- Bidirectional A*  
- Contraction hierarchies  
- Turn penalties and road class weighting  
- Benchmark harness  

---

Generated graph binaries are intentionally excluded from version control.
