#
# This very appreciated template was provided by Matthias Miltenberger.
# To build:
#  > cmake -H. -Bbuild
#  > cmake --build build
#
# If you would like to create a new executable (e.g. test executable), add a new "Test" folder in src. 
# In your new folder, add an "inc" and "src" folder. Then come back here and add the details of your 
# new executable to the areas that have "**" in the comments (e.g. include_directories, add_executable, etc).

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


## Some project defines to keep things clean later
# ** Define the root and source directories of each executable
set(MSPLANNER_DIR ${CMAKE_SOURCE_DIR}/src/MissionPlanner)
set(MSPLANNER_SRC ${MSPLANNER_DIR}/src)
set(LCPLANNER_DIR ${CMAKE_SOURCE_DIR}/src/LocalPlanner)
set(LCPLANNER_SRC ${LCPLANNER_DIR}/src)

# ** Add include directories
include_directories(
	${GUROBI_INCLUDE_DIRS}
	${MSPLANNER_DIR}/inc
	${LCPLANNER_DIR}/inc
)


## Creating and building the Solver library
# Define the source and include solver library directories
set(LIB_SRC_DIR ${CMAKE_SOURCE_DIR}/src/Solver/src)
set(LIB_INC_DIR ${CMAKE_SOURCE_DIR}/src/Solver/inc)

# Gather all .cpp files from src/ directory
file(GLOB SRC_FILES "${LIB_SRC_DIR}/*.cpp")

# Create static solver library from the source files
add_library(solver_library STATIC ${SRC_FILES})

# Specify include directories for the solver library
target_include_directories(solver_library PUBLIC ${LIB_INC_DIR})


## Build executables
# ** Create mission-planner
add_executable(mission-planner
	${MSPLANNER_SRC}/main.cpp
	${MSPLANNER_SRC}/Offline_Input.cpp
	${MSPLANNER_SRC}/Solver_Baseline.cpp
	${MSPLANNER_SRC}/Solver_Greedy.cpp
	${MSPLANNER_SRC}/Solver_Opt.cpp
	${MSPLANNER_SRC}/Solver_VRP.cpp
)

# ** Create local-planner
add_executable(local-planner
	${LCPLANNER_SRC}/main.cpp
	${LCPLANNER_SRC}/Online_Input.cpp
	${LCPLANNER_SRC}/Solver_Standard.cpp
)


## Linking things to your executables
# Link Gurobi library
if(CXX)
	set(CMAKE_CXX_STANDARD 11)
	target_link_libraries(mission-planner optimized ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})
	target_link_libraries(local-planner optimized ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})
endif()

# ** Link Gurobi and solver library
target_link_libraries(mission-planner
	${GUROBI_LIBRARY}
	solver_library
)
target_link_libraries(local-planner 
	${GUROBI_LIBRARY}
	solver_library
)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
	include(FeatureSummary)
	feature_summary(WHAT ALL)
endif()
