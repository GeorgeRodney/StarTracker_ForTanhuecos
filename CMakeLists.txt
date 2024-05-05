cmake_minimum_required(VERSION 3.22)
project(run)

# Find the Eigen3 package
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Define source and header files
set(sources
    TrackFileMgr.cpp
)
set(headers 
    TrackFileMgr.hpp
    TrkUtility.hpp
)

# Add executable for 'run'
add_executable(run run.cpp ${sources} ${headers})
# Link Eigen to 'run'
target_link_libraries(run Eigen3::Eigen)

# Add executable for 'writeState'
add_executable(writeState writeState.cpp)

# Since writeState doesn't link against Eigen, we need to explicitly include Eigen directories if needed
target_include_directories(writeState PRIVATE ${EIGEN3_INCLUDE_DIR})
