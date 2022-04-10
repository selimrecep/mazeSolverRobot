#include "RobotPathFinder.h"
#include "TrueGrid.h"
#include <algorithm>
#include <array>
#include <functional>

PathMatrixReturn RobotPathFinder::solveMaze(bool_grid_t& binaryMatrix,
                                            CellPos entryPoint,
                                            CellPos exitPoint,
                                            bool onlyUseShortestPaths,
                                            optional_step_callback_t callback) {
  // Very dirty, but...
  int rows(binaryMatrix.size());
  int columns(binaryMatrix[0].size());
  // Very dirty, but...
  int tempOriginalRows((rows - 1) / 2);
  int tempOriginalColumns((columns - 1) / 2);
  convertEnterExitPoints(entryPoint, exitPoint, tempOriginalRows,
                         tempOriginalColumns);
  PathMatrixReturn retval{{}, {rows * columns}};
  Path& path{retval.path};

  bool_grid_t& robotMatrix{retval.robotMatrix};
  robotMatrix.resize(rows);
  for (int i{0}; i < rows; ++i) {
    robotMatrix[i].resize(columns);
    std::fill(robotMatrix[i].begin(), robotMatrix[i].end(),
              GridCellState::STAR);
  }

  CellPos currentPoint{entryPoint};
  Orientation robotRot{Orientation::up};

  visits_t visits;
  visits.resize(rows);
  for (int i{0}; i < rows; ++i) {
    visits[i].resize(columns);
  }

  std::stack<CellPos> stack;

  stack.push(currentPoint);

  // Mark enter and exit point in robot matrix, enter point will be overridden
  // tho
  robotMatrix[entryPoint.row][entryPoint.column] = GridCellState::NOTFILLED;
  robotMatrix[exitPoint.row][exitPoint.column] = GridCellState::NOTFILLED;

  bool logNotShortest{false};
  bool logFoundShorterPath{false};

  while (!stack.empty()) {

    currentPoint = stack.top();
    readSensorDataToMatrix(binaryMatrix, robotMatrix, visits, robotRot,
                           currentPoint);
    visits[currentPoint.row][currentPoint.column] = true;
    robotMatrix[currentPoint.row][currentPoint.column] =
        RobotPathFinder::getCorrespondingState(robotRot);

    if (currentPoint == exitPoint) {
      if (path.getLength() == 0 ||
          path.getLength() > static_cast<int>(stack.size())) {
        // Makes a copy of stack :p
        RobotPathFinder::copyStackToPath(stack, path);
        logFoundShorterPath = true;
      }
    }

    if (callback) {
      callback(robotMatrix, onlyUseShortestPaths, logNotShortest,
               (logFoundShorterPath ? path.getLength() : -1));
      if (logFoundShorterPath)
        logFoundShorterPath = false;
      if (logNotShortest)
        logNotShortest = false;
    }

    CellInfo nextCellInfo = getNextPointWithPreference(
        currentPoint, visits, robotMatrix, robotRot, exitPoint);

    bool validMove{nextCellInfo.valid};

    if (onlyUseShortestPaths) {
      int currentPathLength{path.getLength()};
      if (currentPathLength != 0 &&
          currentPathLength <= static_cast<int>(stack.size())) {
        // It isn't shortest path. go back
        validMove = false;
        logNotShortest = true;
      }
    }
    if (validMove) {
      robotMatrix[currentPoint.row][currentPoint.column] =
          GridCellState::BEEN_VISITED;
      robotRot = getRotBetweenPoints(currentPoint, nextCellInfo.pos);
      stack.push(nextCellInfo.pos);

    } else {
      stack.pop();
      if (!stack.empty()) {
        CellPos lastCellOnStack{stack.top()};
        robotMatrix[currentPoint.row][currentPoint.column] =
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

  return retval;
}

// Simulation of robot reading sensor data from the matrix
void RobotPathFinder::readSensorDataToMatrix(bool_grid_t& binaryMatrix,
                                             bool_grid_t& robotMatrix,
                                             visits_t& visits, Orientation rot,
                                             CellPos pos) {
  int rows(binaryMatrix.size());
  int columns(binaryMatrix[0].size());

  if (pos.row > 0 && rot != Orientation::down &&
      !visits[pos.row - 1][pos.column]) {
    robotMatrix[pos.row - 1][pos.column] =
        binaryMatrix[pos.row - 1][pos.column];
  }
  if (pos.column < columns - 1 && rot != Orientation::left &&
      !visits[pos.row][pos.column + 1]) {
    robotMatrix[pos.row][pos.column + 1] =
        binaryMatrix[pos.row][pos.column + 1];
  }
  if (pos.row < rows - 1 && rot != Orientation::up &&
      !visits[pos.row + 1][pos.column]) {
    robotMatrix[pos.row + 1][pos.column] =
        binaryMatrix[pos.row + 1][pos.column];
  }
  if (pos.column > 0 && rot != Orientation::right &&
      !visits[pos.row][pos.column - 1]) {
    robotMatrix[pos.row][pos.column - 1] =
        binaryMatrix[pos.row][pos.column - 1];
  }
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
CellInfo RobotPathFinder::getNextPointWithPreference(CellPos pos,
                                                     visits_t& visits,
                                                     bool_grid_t& binaryMatrix,
                                                     Orientation rot,
                                                     CellPos exitPoint) {
  int rows(binaryMatrix.size());
  int columns(binaryMatrix[0].size());

  CellInfo temp{};
  CellInfo chosen{};

  std::array<CellInfo, 4> allPoints;

  if (pos.row > 0 && rot != Orientation::down &&
      binaryMatrix[pos.row - 1][pos.column] != GridCellState::FILLED &&
      !visits[pos.row - 1][pos.column]) {
    temp.valid = true;
    temp.pos = {pos.row - 1, pos.column};
    temp.cellState = binaryMatrix[pos.row - 1][pos.column];

    allPoints[0] = temp;
  }

  if (pos.column < columns - 1 && rot != Orientation::left &&
      binaryMatrix[pos.row][pos.column + 1] != GridCellState::FILLED &&
      !visits[pos.row][pos.column + 1]) {
    temp.valid = true;
    temp.pos = {pos.row, pos.column + 1};
    temp.cellState = binaryMatrix[pos.row][pos.column + 1];
    allPoints[1] = temp;
  }
  if (pos.row < rows - 1 && rot != Orientation::up &&
      binaryMatrix[pos.row + 1][pos.column] != GridCellState::FILLED &&
      !visits[pos.row + 1][pos.column]) {
    temp.valid = true;
    temp.pos = {pos.row + 1, pos.column};
    temp.cellState = binaryMatrix[pos.row + 1][pos.column];
    allPoints[2] = temp;
  }
  if (pos.column > 0 && rot != Orientation::right &&
      binaryMatrix[pos.row][pos.column - 1] != GridCellState::FILLED &&
      !visits[pos.row][pos.column - 1]) {
    temp.valid = true;
    temp.pos = {pos.row, pos.column - 1};
    temp.cellState = binaryMatrix[pos.row][pos.column - 1];
    allPoints[3] = temp;
  }

  if (exitPoint.row < pos.row && allPoints[0].valid)
    chosen = allPoints[0];
  else if (exitPoint.column > pos.column && allPoints[1].valid)
    chosen = allPoints[1];
  else if (exitPoint.row > pos.row && allPoints[2].valid)
    chosen = allPoints[2];
  else if (exitPoint.column < pos.column && allPoints[3].valid)
    chosen = allPoints[3];
  else {
    // If nothing stick to normal decision
    int i{0};
    while (i < 4 && !allPoints[i].valid)
      ++i;

    if (i != 4)
      chosen = allPoints[i];
  }

  return chosen;
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