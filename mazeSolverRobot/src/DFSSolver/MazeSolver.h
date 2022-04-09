#pragma once
#include <CellPos.h>
#include <Maze2.h>
#include <Path.h>
#include <stack>

using visits_t = std::vector<std::vector<bool>>;

class MazeSolver {
private:
  static CellInfo getNextPoint(CellPos pos, visits_t& visits, TrueGrid& grid);
  static CellInfo getNextPoint(CellPos pos, visits_t& visits,
                               bool_grid_t& binaryMatrix);

public:
  static Path solveMaze(Maze2& maze);
  static Path solveBinaryMaze(bool_grid_t& binaryMatrix, CellPos bEnterPoint,
                              CellPos bExitPoint);
  static void copyStackToPath(std::stack<CellPos> stack, Path& path);
};