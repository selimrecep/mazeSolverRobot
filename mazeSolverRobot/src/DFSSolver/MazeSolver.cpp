#include <MazeSolver.h>
#include <RobotPathFinder.h>
#include <TrueGrid.h>
#include <cassert>
#include <stack>


Path MazeSolver::solveBinaryMaze(bool_grid_t& binaryMatrix, CellPos bEnterPoint,
                                 CellPos bExitPoint) {
  int rows(binaryMatrix.size());
  int columns(binaryMatrix[0].size());

  RobotPathFinder::convertEnterExitPoints(bEnterPoint, bExitPoint,
                                          (rows - 1) / 2, (columns - 1) / 2);

  visits_t visits;
  visits.resize(rows);
  for (int i{0}; i < rows; ++i) {
    visits[i].resize(columns);
  }

  std::stack<CellPos> stack;
  Path path{rows * columns};

  CellPos currentPoint{bEnterPoint};
  stack.push(currentPoint);

  while (!stack.empty()) {
    currentPoint = stack.top();
    visits[currentPoint.row][currentPoint.column] = true;

    if (currentPoint == bExitPoint) {
      if (path.getLength() == 0 ||
          path.getLength() > static_cast<int>(stack.size())) {
        // Makes a copy of stack :p
        MazeSolver::copyStackToPath(stack, path);
      }
    }
    CellInfo nextPoint{
        MazeSolver::getNextPoint(currentPoint, visits, binaryMatrix)};
    if (nextPoint.valid) {
      stack.push(nextPoint.pos);
    } else {
      stack.pop();
    }
  }
  return path;
}
Path MazeSolver::solveMaze(Maze2& maze) {
  assert(maze.hasEnterAndExit());

  int rows{maze.getTrueGrid().rows};
  int columns{maze.getTrueGrid().columns};
  visits_t visits;

  visits.resize(rows);
  for (int i{0}; i < rows; ++i) {
    visits[i].resize(columns);
  }

  // Hmm.
  Path path{maze.getTotalCellCount()};

  std::stack<CellPos> stack;
  CellPos enterPoint, exitPoint;
  maze.getEnterAndExitPoint(enterPoint, exitPoint);

  CellPos currentPoint{enterPoint};
  stack.push(currentPoint);

  while (!stack.empty()) {
    currentPoint = stack.top();
    visits[currentPoint.row][currentPoint.column] = true;

    if (currentPoint == exitPoint) {
      if (path.getLength() == 0 ||
          path.getLength() > static_cast<int>(stack.size())) {
        // Makes a copy of stack :p
        MazeSolver::copyStackToPath(stack, path);
      }
    }
    CellInfo nextPoint{getNextPoint(currentPoint, visits, maze.getTrueGrid())};
    if (nextPoint.valid) {
      stack.push(nextPoint.pos);
    } else {
      stack.pop();
    }
  }
  return path;
}

CellInfo MazeSolver::getNextPoint(CellPos pos, visits_t& visits,
                                  bool_grid_t& binaryMatrix) {
  int rows(binaryMatrix.size());
  int columns(binaryMatrix[0].size());

  CellInfo info{};

  if (pos.row > 0 &&
      binaryMatrix[pos.row - 1][pos.column] != GridCellState::FILLED &&
      !visits[pos.row - 1][pos.column]) {
    info.valid = true;
    info.pos = {pos.row - 1, pos.column};
    info.cellState = binaryMatrix[pos.row - 1][pos.column];
  } else if (pos.column < columns - 1 &&
             binaryMatrix[pos.row][pos.column + 1] != GridCellState::FILLED &&
             !visits[pos.row][pos.column + 1]) {
    info.valid = true;
    info.pos = {pos.row, pos.column + 1};
    info.cellState = binaryMatrix[pos.row][pos.column + 1];
  } else if (pos.row < rows - 1 &&
             binaryMatrix[pos.row + 1][pos.column] != GridCellState::FILLED &&
             !visits[pos.row + 1][pos.column]) {
    info.valid = true;
    info.pos = {pos.row + 1, pos.column};
    info.cellState = binaryMatrix[pos.row + 1][pos.column];
  } else if (pos.column > 0 &&
             binaryMatrix[pos.row][pos.column - 1] != GridCellState::FILLED &&
             !visits[pos.row][pos.column - 1]) {
    info.valid = true;
    info.pos = {pos.row, pos.column - 1};
    info.cellState = binaryMatrix[pos.row][pos.column - 1];
  }

  return info;
}

CellInfo MazeSolver::getNextPoint(CellPos pos, visits_t& visits,
                                  TrueGrid& grid) {
  walls_t& wallsRows = grid.getRowWalls();
  walls_t& wallsColumns = grid.getColumnWalls();

  CellInfo info{};
  if (pos.row > 0 &&
      wallsRows[pos.row - 1][pos.column] == WallCellState::OPENED) {
    if (!visits[pos.row - 1][pos.column]) {
      info.valid = true;
      info.pos = {pos.row - 1, pos.column};
      info.cellState = grid[pos.row][pos.column];
      return info;
    }
  }

  if (pos.column < grid.columns - 1 &&
      wallsColumns[pos.row][pos.column] == WallCellState::OPENED) {
    if (!visits[pos.row][pos.column + 1]) {
      info.valid = true;
      info.pos = {pos.row, pos.column + 1};
      info.cellState = grid[pos.row][pos.column];
      return info;
    }
  }

  if (pos.row < grid.rows - 1 &&
      wallsRows[pos.row][pos.column] == WallCellState::OPENED) {
    if (!visits[pos.row + 1][pos.column]) {
      info.valid = true;
      info.pos = {pos.row + 1, pos.column};
      info.cellState = grid[pos.row][pos.column];
      return info;
    }
  }

  if (pos.column > 0 &&
      wallsColumns[pos.row][pos.column - 1] == WallCellState::OPENED) {
    if (!visits[pos.row][pos.column - 1]) {
      info.valid = true;
      info.pos = {pos.row, pos.column - 1};
      info.cellState = grid[pos.row][pos.column];
      return info;
    }
  }

  // no valid pos
  return info;
}

void MazeSolver::copyStackToPath(std::stack<CellPos> stack, Path& path) {
  // emptys
  path.empty();

  while (!stack.empty()) {
    path.addStep(stack.top());
    stack.pop();
  }
}