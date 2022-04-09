#include "RobotPathFinder.h"
#include "TrueGrid.h"
#include <functional>

using visits_t = std::vector<std::vector<bool>>;

Path RobotPathFinder::solveMaze(bool_grid_t& binaryMatrix, CellPos entryPoint,
                                CellPos exitPoint,
                                optional_step_callback_t callback) {
  // Very dirty, but...
  int rows(binaryMatrix.size());
  int columns(binaryMatrix[0].size());
  // Very dirty, but...
  int tempOriginalRows((rows - 1) / 2);
  int tempOriginalColumns((columns - 1) / 2);
  convertEnterExitPoints(entryPoint, exitPoint, tempOriginalRows,
                         tempOriginalColumns);

  CellPos currentPoint{entryPoint};
  Orientation robotRot{Orientation::up};

  visits_t visits;
  visits.resize(rows);
  for (int i{0}; i < rows; ++i) {
    visits[i].resize(columns);
  }

  std::stack<CellPos> stack;
  Path path{rows * columns};

  stack.push(currentPoint);

  while (!stack.empty()) {
    currentPoint = stack.top();
    visits[currentPoint.row][currentPoint.column] = true;

    binaryMatrix[currentPoint.row][currentPoint.column] =
        RobotPathFinder::getCorrespondingState(robotRot);

    if (callback) {
      callback(binaryMatrix);
    }

    if (currentPoint == exitPoint) {
      if (path.getLength() == 0 ||
          path.getLength() > static_cast<int>(stack.size())) {
        // Makes a copy of stack :p
        RobotPathFinder::copyStackToPath(stack, path);
      }
    }

    CellInfo nextCellInfo =
        getNextPoint(currentPoint, visits, binaryMatrix, robotRot);
    if (nextCellInfo.valid) {
      binaryMatrix[currentPoint.row][currentPoint.column] =
          GridCellState::BEEN_VISITED;
      robotRot = getRotBetweenPoints(currentPoint, nextCellInfo.pos);
      stack.push(nextCellInfo.pos);
    } else {
      stack.pop();
      if (!stack.empty()) {
        CellPos lastCellOnStack{stack.top()};
        binaryMatrix[currentPoint.row][currentPoint.column] =
            GridCellState::BEEN_LEFT;
        robotRot = getRotBetweenPoints(currentPoint, lastCellOnStack);
      } else {
        if (path.getLength() == 0) {
          // We don't have any path yet
          // Nothing on stack to go back, keep rotating
          stack.push(currentPoint);
          robotRot = getNextRot(robotRot);
        }
      }
    }
  }

  return path;
}
Orientation RobotPathFinder::getNextRot(Orientation rot) {
  switch (rot) {
  case Orientation::up:
    return Orientation::right;
  case Orientation::right:
    return Orientation::down;
  case Orientation::down:
    return Orientation::left;
  case Orientation::left:
    return Orientation::up;
  case Orientation::unkown:
    assert(false && "rip");
  }
  return Orientation::unkown;
}
void RobotPathFinder::convertEnterExitPoints(CellPos& enterPoint,
                                             CellPos& leavePoint, int rows,
                                             int columns) {
  int gridRowCount{rows * 2 + 1};
  int gridColumnCount{columns * 2 + 1};
  CellPos tempEnter{}, tempExit{};

  if (enterPoint.row == 0) {
    tempEnter.row = 0;
    tempEnter.column = enterPoint.column * 2 + 1;
  } else if (enterPoint.row == rows - 1) {
    tempEnter.row = gridRowCount - 1;
    tempEnter.column = enterPoint.column * 2 + 1;
  } else if (enterPoint.column == 0) {
    tempEnter.column = 0;
    tempEnter.row = enterPoint.row * 2 + 1;
  } else if (enterPoint.column == columns - 1) {
    tempEnter.column = gridColumnCount - 1;
    tempEnter.row = enterPoint.row * 2 + 1;
  } else {
    std::cout << "ouch";
    assert(false);
  }

  if (leavePoint.row == 0) {
    tempExit.row = 0;
    tempExit.column = leavePoint.column * 2 + 1;
  } else if (leavePoint.row == rows - 1) {
    tempExit.row = gridRowCount - 1;
    tempExit.column = leavePoint.column * 2 + 1;
  } else if (leavePoint.column == 0) {
    tempExit.column = 0;
    tempExit.row = leavePoint.row * 2 + 1;
  } else if (leavePoint.column == columns - 1) {
    tempExit.column = gridColumnCount - 1;
    tempExit.row = leavePoint.row * 2 + 1;
  } else {
    std::cout << "ouch";
    assert(false);
  }
  enterPoint = tempEnter;
  leavePoint = tempExit;
}

CellInfo RobotPathFinder::getNextPoint(CellPos pos, visits_t& visits,
                                       bool_grid_t& binaryMatrix,
                                       Orientation rot) {
  int rows(binaryMatrix.size());
  int columns(binaryMatrix[0].size());

  CellInfo info{};

  if (pos.row > 0 && rot != Orientation::down &&
      binaryMatrix[pos.row - 1][pos.column] != GridCellState::FILLED &&
      !visits[pos.row - 1][pos.column]) {
    info.valid = true;
    info.pos = {pos.row - 1, pos.column};
    info.cellState = binaryMatrix[pos.row - 1][pos.column];
  } else if (pos.column < columns - 1 && rot != Orientation::left &&
             binaryMatrix[pos.row][pos.column + 1] != GridCellState::FILLED &&
             !visits[pos.row][pos.column + 1]) {
    info.valid = true;
    info.pos = {pos.row, pos.column + 1};
    info.cellState = binaryMatrix[pos.row][pos.column + 1];
  } else if (pos.row < rows - 1 && rot != Orientation::up &&
             binaryMatrix[pos.row + 1][pos.column] != GridCellState::FILLED &&
             !visits[pos.row + 1][pos.column]) {
    info.valid = true;
    info.pos = {pos.row + 1, pos.column};
    info.cellState = binaryMatrix[pos.row + 1][pos.column];
  } else if (pos.column > 0 && rot != Orientation::right &&
             binaryMatrix[pos.row][pos.column - 1] != GridCellState::FILLED &&
             !visits[pos.row][pos.column - 1]) {
    info.valid = true;
    info.pos = {pos.row, pos.column - 1};
    info.cellState = binaryMatrix[pos.row][pos.column - 1];
  }

  return info;
}

Orientation RobotPathFinder::getRotBetweenPoints(CellPos from, CellPos to) {
  if (to.row - from.row < 0)
    return Orientation::up;
  if (to.column - from.column > 0)
    return Orientation::right;
  if (to.row - from.row > 0)
    return Orientation::down;
  if (to.column - from.column < 0)
    return Orientation::left;
  return Orientation::unkown;
}
void RobotPathFinder::copyStackToPath(std::stack<CellPos> stack, Path& path) {
  // emptys
  path.empty();

  while (!stack.empty()) {
    path.addStep(stack.top());
    stack.pop();
  }
}

GridCellState RobotPathFinder::getCorrespondingState(Orientation rot) {
  switch (rot) {
  case Orientation::up:
    return GridCellState::UP_LOOKING;
  case Orientation::right:
    return GridCellState::RIGHT_LOOKING;
  case Orientation::down:
    return GridCellState::DOWN_LOOKING;
  case Orientation::left:
    return GridCellState::LEFT_LOOKING;
  default:
    assert(false && "RIP");
  }
  return GridCellState::FILLED;
}