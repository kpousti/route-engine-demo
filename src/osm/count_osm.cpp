#include <iostream>
#include <string>
#include <chrono>

#include <osmium/io/any_input.hpp>
#include <osmium/visitor.hpp>
#include <osmium/handler.hpp>

class CountHandler : public osmium::handler::Handler {
public:
    size_t node_count = 0;
    size_t way_count = 0;
    size_t relation_count = 0;

    void node(const osmium::Node&) {
        ++node_count;
    }

    // Only count actual roads (ways with "highway" tag)
    void way(const osmium::Way& w) {
    const char* h = w.tags()["highway"];
    if (!h) return;

    // Count mostly drivable road classes (skip footways/paths/cycleways/etc.)
    if (
        std::string(h) == "motorway" ||
        std::string(h) == "trunk" ||
        std::string(h) == "primary" ||
        std::string(h) == "secondary" ||
        std::string(h) == "tertiary" ||
        std::string(h) == "unclassified" ||
        std::string(h) == "residential" ||
        std::string(h) == "living_street"
        // optional: include "service" if you want parking lot roads too
        // || std::string(h) == "service"
    ) {
        ++way_count;
    }
}


    void relation(const osmium::Relation&) {
        ++relation_count;
    }
};

int main() {
    try {
        // IMPORTANT: because we run from build/
        const std::string path = "../data/region.osm.pbf";

        auto t0 = std::chrono::steady_clock::now();

        osmium::io::Reader reader(path);
        CountHandler handler;

        osmium::apply(reader, handler);
        reader.close();

        auto t1 = std::chrono::steady_clock::now();
        std::chrono::duration<double> secs = t1 - t0;

        std::cout << "Nodes: " << handler.node_count << "\n";
        std::cout << "Ways (roads only): " << handler.way_count << "\n";
        std::cout << "Relations: " << handler.relation_count << "\n";

        double roads_per_node =
            static_cast<double>(handler.way_count) /
            static_cast<double>(handler.node_count);

        std::cout << "Road density (roads per node): "
                  << roads_per_node << "\n";

        std::cout << "Time: " << secs.count() << " seconds\n";
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
