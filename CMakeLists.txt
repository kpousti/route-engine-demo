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

find_package(Threads REQUIRED)

# ✅ NEW: OpenSSL for HTTPS (cpp-httplib SSLClient)
find_package(OpenSSL REQUIRED)

# ----------------------------
# Executables (OSM tools)
# ----------------------------
add_executable(count_osm src/osm/count_osm.cpp)
target_include_directories(count_osm PRIVATE ${OSMIUM_INCLUDE_DIR})
target_link_libraries(count_osm PRIVATE
  ${ZLIB_LIB} ${EXPAT_LIB} ${BZ2_LIB} ${LZ4_LIB}
)

add_executable(build_graph src/osm/build_graph.cpp)
target_include_directories(build_graph PRIVATE ${OSMIUM_INCLUDE_DIR})
target_link_libraries(build_graph PRIVATE
  ${ZLIB_LIB} ${EXPAT_LIB} ${BZ2_LIB} ${LZ4_LIB}
)

add_executable(load_graph src/osm/load_graph.cpp)
target_include_directories(load_graph PRIVATE ${OSMIUM_INCLUDE_DIR})
target_link_libraries(load_graph PRIVATE
  ${ZLIB_LIB} ${EXPAT_LIB} ${BZ2_LIB} ${LZ4_LIB}
)

# ----------------------------
# shared routing library
# ----------------------------
add_library(route_engine STATIC
  src/graph/route_engine.cpp
)

target_include_directories(route_engine PUBLIC
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/include
)

# ----------------------------
# route_graph (CLI) - only if file exists
# ----------------------------
if (EXISTS "${CMAKE_SOURCE_DIR}/src/osm/route_graph.cpp")
  add_executable(route_graph src/osm/route_graph.cpp)
  target_include_directories(route_graph PRIVATE ${OSMIUM_INCLUDE_DIR})
  target_link_libraries(route_graph PRIVATE
    route_engine
    ${ZLIB_LIB} ${EXPAT_LIB} ${BZ2_LIB} ${LZ4_LIB}
  )
endif()

# ----------------------------
# route_server (HTTP backend)
# ----------------------------
add_executable(route_server src/api/server.cpp)

target_include_directories(route_server PRIVATE
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/include
)

# ✅ Tell httplib to enable OpenSSL + link OpenSSL libs
target_compile_definitions(route_server PRIVATE CPPHTTPLIB_OPENSSL_SUPPORT)

target_link_libraries(route_server PRIVATE
  route_engine
  Threads::Threads
  OpenSSL::SSL
  OpenSSL::Crypto
)