include_directories(${mazeSolverRobotProject_SOURCE_DIR}/mazeSolverRobot/src/)
include_directories(${mazeSolverRobotProject_SOURCE_DIR}/mazeSolverRobot/src/DFSSolver)
include_directories(${mazeSolverRobotProject_SOURCE_DIR}/mazeSolverRobot/src/robotPathFinder)
include_directories(${mazeSolverRobotProject_SOURCE_DIR}/mazeGenerator/src/)
add_library(mazeSolverRobot src/main.cpp)

target_link_libraries(mazeSolverRobot mazeGenerator)