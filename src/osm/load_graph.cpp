#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>
#include <string>
#include <stdexcept>

struct GraphCSR {
    uint64_t N = 0;
    uint64_t M = 0;
    std::vector<uint64_t> node_ids;   // idx -> OSM node id
    std::vector<double>   lat;        // idx -> lat
    std::vector<double>   lon;        // idx -> lon
    std::vector<uint64_t> offsets;    // size N+1
    std::vector<uint32_t> to;         // size M
    std::vector<float>    w;          // size M
};

static GraphCSR load_csr_v2(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("Failed to open: " + path);

    auto read_u64 = [&](){
        uint64_t v;
        in.read(reinterpret_cast<char*>(&v), sizeof(v));
        return v;
    };

    GraphCSR g;
    g.N = read_u64();
    g.M = read_u64();

    g.node_ids.resize(g.N);
    g.lat.resize(g.N);
    g.lon.resize(g.N);
    g.offsets.resize(g.N + 1);
    g.to.resize(g.M);
    g.w.resize(g.M);

    in.read(reinterpret_cast<char*>(g.node_ids.data()), sizeof(uint64_t) * g.N);
    in.read(reinterpret_cast<char*>(g.lat.data()),      sizeof(double)   * g.N);
    in.read(reinterpret_cast<char*>(g.lon.data()),      sizeof(double)   * g.N);

    in.read(reinterpret_cast<char*>(g.offsets.data()),  sizeof(uint64_t) * (g.N + 1));
    in.read(reinterpret_cast<char*>(g.to.data()),       sizeof(uint32_t) * g.M);
    in.read(reinterpret_cast<char*>(g.w.data()),        sizeof(float)    * g.M);

    if (!in) throw std::runtime_error("Read failed / file truncated?");
    if (g.offsets.back() != g.M) throw std::runtime_error("Bad CSR: offsets[N] != M");

    return g;
}

int main() {
    try {
        const std::string path = "../data/out/graph_csr.bin";
        auto g = load_csr_v2(path);

        std::cout << "Loaded CSR graph (v2 + coords):\n";
        std::cout << "N = " << g.N << "\n";
        std::cout << "M = " << g.M << "\n";
        std::cout << "avg out-degree = " << (g.N ? (double)g.M / (double)g.N : 0.0) << "\n";

        if (g.N > 0) {
            uint64_t deg0 = g.offsets[1] - g.offsets[0];
            std::cout << "deg(node 0) = " << deg0 << "\n";
            std::cout << "node0 OSM id = " << g.node_ids[0] << "\n";
            std::cout << "node0 lat/lon = " << g.lat[0] << ", " << g.lon[0] << "\n";
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
