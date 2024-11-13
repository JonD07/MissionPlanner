#
# This very appreciated template was provided by Matthias Miltenberger.
# To build:
#  > cmake -H. -Bbuild
#  > cmake --build build
#

cmake_minimum_required(VERSION 3.10)

project(holistic-planner)

option(CXX "enable C++ compilation" ON)
if(CXX)
	enable_language(CXX)
endif()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
set(CMAKE_CXX_FLAGS "-O3 -ggdb -g -Wall -Werror")

# Visual Studio compiler with static runtime libraries
if(MSVC AND MT)
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT -O0 -ggdb -g -Wall -Werror")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd -O0 -ggdb -g -Wall -Werror")
	set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MT -O0 -ggdb -g -Wall -Werror")
	set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MTd -O0 -ggdb -g -Wall -Werror")
endif()

# Get Gurobi
find_package(GUROBI REQUIRED)

include_directories(
	${GUROBI_INCLUDE_DIRS}
	${CMAKE_CURRENT_LIST_DIR}/inc
	${CMAKE_CURRENT_LIST_DIR}/inc/MissionPlanner
	${CMAKE_CURRENT_LIST_DIR}/inc/LocalPlanner
)

# list source files here
#set(sources src/MissionPlanner/main.cpp)

add_executable(mission-planner
	src/MissionPlanner/main.cpp
	src/ClusteringAlgorithm.cpp
	src/COptimizer.cpp
	src/Drone.cpp
	src/DroneGenerator.cpp
	src/DummyHeuristic.cpp
	src/FileReader.cpp
	src/HLOptimizer.cpp
	src/Input.cpp
	src/LKH_TSP_Solver.cpp
	src/Node.cpp
	src/NodeGenerator.cpp
	src/Solution.cpp
	src/Solver.cpp
	src/TighteningHeuristic.cpp
	src/TourImprover.cpp
	src/Utilities.cpp
	src/MissionPlanner/Offline_Input.cpp
	src/MissionPlanner/Solver_Baseline.cpp
	src/MissionPlanner/Solver_Greedy.cpp
	src/MissionPlanner/Solver_Opt.cpp
	src/MissionPlanner/Solver_VRP.cpp
)

add_executable(local-planner
	src/LocalPlanner/main.cpp
	src/Drone.cpp
	src/DroneGenerator.cpp
	src/FileReader.cpp
	src/HLOptimizer.cpp
	src/Input.cpp
	src/Node.cpp
	src/NodeGenerator.cpp
	src/Solution.cpp
	src/Solver.cpp
	src/Utilities.cpp
	src/LocalPlanner/Online_Input.cpp
	src/LocalPlanner/Solver_Standard.cpp
)

if(CXX)
	set(CMAKE_CXX_STANDARD 11)
	target_link_libraries(mission-planner optimized ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})
	target_link_libraries(local-planner optimized ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})
endif()

# Link Gurobi
target_link_libraries(mission-planner ${GUROBI_LIBRARY})
target_link_libraries(local-planner ${GUROBI_LIBRARY})

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
	include(FeatureSummary)
	feature_summary(WHAT ALL)
endif()
