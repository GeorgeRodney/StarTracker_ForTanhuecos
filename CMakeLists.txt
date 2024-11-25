cmake_minimum_required(VERSION 3.22)
project(run)

# Set the build type to Debug
set(CMAKE_BUILD_TYPE Debug)

# Find the Eigen3 package
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system)

# Define source and header files
set(sources
    src/TrackFileMgr.cpp
    DataAssociation/hungarian.cpp
    DataAssociation/auction.cpp
    DataAssociation/binning.cpp
    # DataAssociation/mcmc.cpp // BUSTED
)
set(headers 
    inc/TrackFileMgr.hpp
    inc/TrkUtility.hpp
)

# Add executable for 'run'
add_executable(run run.cpp ${sources} ${headers})
# Link Eigen to 'run'
target_link_libraries(run Eigen3::Eigen)
target_link_libraries(run ${Boost_LIBRARIES})

# Add executable for 'writeState'
add_executable(writeState writeState.cpp)

target_include_directories(run PRIVATE inc)  # Include inc for run
target_include_directories(writeState PRIVATE inc)  # Include inc for writeState

# Since writeState doesn't link against Eigen, we need to explicitly include Eigen directories if needed
target_include_directories(writeState PRIVATE ${EIGEN3_INCLUDE_DIR})
