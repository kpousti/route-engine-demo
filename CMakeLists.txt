cmake_minimum_required(VERSION 3.16)
project(tritonroute LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# libosmium is header-only in Homebrew (no find_package(osmium))
find_path(OSMIUM_INCLUDE_DIR
  NAMES osmium/io/any_input.hpp
  PATHS /usr/local/include /opt/homebrew/include
  REQUIRED
)

# Common deps for reading .pbf on macOS
find_library(ZLIB_LIB z REQUIRED)
find_library(EXPAT_LIB expat REQUIRED)
find_library(BZ2_LIB bz2 REQUIRED)
find_library(LZ4_LIB lz4 REQUIRED)

# ---- count_osm ----
add_executable(count_osm src/osm/count_osm.cpp)
target_include_directories(count_osm PRIVATE ${OSMIUM_INCLUDE_DIR})
target_link_libraries(count_osm PRIVATE
  ${ZLIB_LIB} ${EXPAT_LIB} ${BZ2_LIB} ${LZ4_LIB}
)

# ---- build_graph ----
add_executable(build_graph src/osm/build_graph.cpp)
target_include_directories(build_graph PRIVATE ${OSMIUM_INCLUDE_DIR})
target_link_libraries(build_graph PRIVATE
  ${ZLIB_LIB} ${EXPAT_LIB} ${BZ2_LIB} ${LZ4_LIB}
)

# ---- load_graph ----
add_executable(load_graph src/osm/load_graph.cpp)
target_include_directories(load_graph PRIVATE ${OSMIUM_INCLUDE_DIR})
target_link_libraries(load_graph PRIVATE
  ${ZLIB_LIB} ${EXPAT_LIB} ${BZ2_LIB} ${LZ4_LIB}
)

# ---- route_graph (only if this file exists) ----
# If you haven't created src/osm/route_graph.cpp yet, comment these 3 lines out.
add_executable(route_graph src/osm/route_graph.cpp)
target_include_directories(route_graph PRIVATE ${OSMIUM_INCLUDE_DIR})
target_link_libraries(route_graph PRIVATE
  ${ZLIB_LIB} ${EXPAT_LIB} ${BZ2_LIB} ${LZ4_LIB}
)
