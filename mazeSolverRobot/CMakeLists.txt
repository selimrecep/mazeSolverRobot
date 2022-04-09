include_directories(${mazeSolverRobotProject_SOURCE_DIR}/mazeSolverRobot/src/)
include_directories(${mazeSolverRobotProject_SOURCE_DIR}/mazeSolverRobot/src/DFSSolver)
include_directories(${mazeSolverRobotProject_SOURCE_DIR}/mazeSolverRobot/src/robotPathFinder)
include_directories(${mazeSolverRobotProject_SOURCE_DIR}/mazeGenerator/src/)
add_library( mazeSolverRobot_src src/main.cpp src/Path.cpp src/MazeTesting.cpp src/DFSSolver/MazeSolver.cpp src/robotPathFinder/RobotPathFinder.cpp)

add_executable(mazeSolverRobotExe src/main.cpp)
target_link_libraries(mazeSolverRobotExe mazeSolverRobot_src mazeGenerator)