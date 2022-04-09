#pragma once

#include "TrueGrid.h"
#include <MazeSolver.h>
#include <functional>

enum class Orientation { up, down, left, right, unkown };

using optional_step_callback_t = std::function<void(bool_grid_t&)>;

class RobotPathFinder {
private:
  static CellInfo getNextPoint(CellPos pos, visits_t& visits,
                               bool_grid_t& binaryMatrix, Orientation rot);
  static Orientation getNextRot(Orientation rot);
  static Orientation getRotBetweenPoints(CellPos from, CellPos to);
  static void copyStackToPath(std::stack<CellPos> stack, Path& path);
  static GridCellState getCorrespondingState(Orientation rot);

public:
  static void convertEnterExitPoints(CellPos& entryPoint, CellPos& exitPoint,
                                     int rows, int columns);
  static Path solveMaze(bool_grid_t& binaryMatrix, CellPos entryPoint,
                        CellPos exitPoint, optional_step_callback_t = nullptr);
};