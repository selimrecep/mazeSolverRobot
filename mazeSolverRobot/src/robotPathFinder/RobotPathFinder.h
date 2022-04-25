#pragma once

#include "CellPos.h"
#include "TrueGrid.h"
#include <MazeSolver.h>
#include <functional>

enum class Orientation { up, right, down, left, unkown };

using optional_step_callback_t =
    std::function<void(bool_grid_t&, bool, bool, int)>;
using visits_t = std::vector<std::vector<bool>>;

using decision_callback_t =
    std::function<bool(std::array<CellInfo, 4>&, CellPos, CellInfo&)>;

struct PathMatrixReturn {
  bool_grid_t robotMatrix;
  Path path;
};

class RobotPathFinder {
private:
  static CellInfo getNextPoint(CellPos pos, visits_t& visits,
                               bool_grid_t& binaryMatrix, Orientation rot);
  static CellInfo getNextPointWithPreference(CellPos pos, visits_t& visits,
                                             bool_grid_t& binaryMatrix,
                                             Orientation rot,
                                             CellPos exitPoint);
  static CellInfo getNextPointWithPreference(CellPos pos, visits_t& visits,
                                             bool_grid_t& binaryMatrix,
                                             Orientation rot,
                                             decision_callback_t decider);
  static Orientation getNextRot(Orientation rot);
  static void copyStackToPath(std::stack<CellPos> stack, Path& path);
  static GridCellState getCorrespondingState(Orientation rot);

public:
  static Orientation getRotBetweenPoints(CellPos from, CellPos to);
  static void convertEnterExitPoints(CellPos& entryPoint, CellPos& exitPoint,
                                     int rows, int columns);
  static PathMatrixReturn solveMaze(bool_grid_t& binaryMatrix,
                                    CellPos entryPoint, CellPos exitPoint,
                                    decision_callback_t decider,
                                    bool onlyUseShortestPaths = false,
                                    optional_step_callback_t = nullptr);
  static void readSensorDataToMatrix(bool_grid_t& binaryMatrix,
                                     bool_grid_t& robotMatrix, visits_t& visits,
                                     Orientation rot, CellPos pos);
  static CellPos getNextPointWithRot(CellPos pos, Orientation rot);
};