cmake_minimum_required(VERSION 3.17)
project(Roadrunner)

set(CMAKE_CXX_STANDARD 17)

# 1. Standard Dependencies (You still need ZLIB and Threads)
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)

# 2. Tell CMake where the downloaded libraries are
# We point to the 'include' folder inside the folders you just extracted
include_directories(include)
include_directories(libs/libosmium/include)
include_directories(libs/protozero/include)

# 3. Build Setup
add_executable(Roadrunner src/main.cpp
        include/utils.h
        include/graph.h
        include/router.h)
target_link_libraries(Roadrunner Threads::Threads ZLIB::ZLIB)