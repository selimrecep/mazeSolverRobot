#include "Path.h"
#include "RobotPathFinder.h"
#include "TrueGrid.h"
#include <Maze2.h>
#include <MazeTesting.h>
#include <conio.h>
#include <cstdio>
#include <iostream>
#include <random.h>

void testSingleMaze() {

  CellPos entryPoint{};
  CellPos exitPoint{};
  int rows{};
  int columns{};

  std::wcout << "Enter row count: ";
  std::wcin >> rows;
  std::wcout << "Enter column count: ";
  std::wcin >> columns;
  std::wcout << "Row of entryPoint(row, column), based on 1: ";
  std::wcin >> entryPoint.row;
  std::wcout << "Column of entryPoint(row, column), based on 1: ";
  std::wcin >> entryPoint.column;
  std::wcout << "Row of exitPoint(row, column), based on 1: ";
  std::wcin >> exitPoint.row;
  std::wcout << "Column of exitPoint(row, column), based on 1: ";
  std::wcin >> exitPoint.column;

  std::wcout << "Last question: do you want robot to prefer to only seek for "
                "shortest paths?(y/n): ";
  wchar_t ch{};
  bool useShortestPathOnly{false};
  std::wcin >> ch;
  if (ch == 'y')
    useShortestPathOnly = true;

  entryPoint.row--;
  entryPoint.column--;
  exitPoint.row--;
  exitPoint.column--;

  if (entryPoint.row < 0 || entryPoint.row > rows - 1 ||
      entryPoint.column < 0 || entryPoint.column > columns - 1) {
    std::wcout << "Invalid values.\n";
    testSingleMaze(); // :P
    return;
  }

  if (exitPoint.row < 0 || exitPoint.row > rows - 1 || exitPoint.column < 0 ||
      exitPoint.column > columns - 1) {
    std::wcout << "Invalid values.\n";
    testSingleMaze(); // :P
    return;
  }

  Maze2 maze2{rows, columns, entryPoint, exitPoint};
  maze2.generate();
  maze2.printMaze();

  std::cout << "\n";

  maze2.printMaze();
  bool_grid_t gridBinary;
  maze2.getTrueGrid().getAsMatrix(gridBinary);

  maze2.getTrueGrid().printTheMatrix(gridBinary);

  optional_step_callback_t callback = [](bool_grid_t& grid,
                                         bool useOnlyShortestPath,
                                         bool logNotShortPath,
                                         int logFoundShorterPath) {
    static bool passOver{false};
    if (!passOver) {
      TrueGrid::printTheMatrix(grid);
      std::wcout << "-----------\n";
      if (logFoundShorterPath != -1)
        std::wcout << "LOG: Found shorter path with length of "
                   << logFoundShorterPath << " \n";
      if (useOnlyShortestPath && logNotShortPath)
        std::wcout << "LOG: This is not a shorter path! Going back.\n";
      std::wcout << "Press any key(after a key) to iterate for next step Write "
                    "'e' if want to pass over.\n ";
      std::wcin.clear();
      std::wcin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

      // why doesnt work...
      wchar_t ch{};
      std::wcin >> ch;
      if (ch == 'e')
        passOver = true;
    }
  };

  auto euclerShortestDecider = [exitPoint](std::array<CellInfo, 4>& allPoints,
                                           CellPos pos,
                                           CellInfo& chosen) -> bool {
    if (exitPoint.row < pos.row && allPoints[0].valid)
      chosen = allPoints[0];
    else if (exitPoint.column > pos.column && allPoints[1].valid)
      chosen = allPoints[1];
    else if (exitPoint.row > pos.row && allPoints[2].valid)
      chosen = allPoints[2];
    else if (exitPoint.column < pos.column && allPoints[3].valid)
      chosen = allPoints[3];
    else
      return false;
    return true;
  };

  PathMatrixReturn pathAndMatrix{RobotPathFinder::solveMaze(
      gridBinary, entryPoint, exitPoint, euclerShortestDecider,
      useShortestPathOnly, callback)};
  Path& path{pathAndMatrix.path};
  bool_grid_t& robotMatrix{pathAndMatrix.robotMatrix};
  std::wcout << "Maze finished! Clearing matrix and printing final state: \n";
  std::wcout << "Robot matrix:\n";
  TrueGrid::printTheMatrix(robotMatrix);

  std::wcout << "cleared state: \n";

  TrueGrid& grid = maze2.getTrueGrid();
  grid.getAsMatrix(gridBinary);

  for (auto& pos : path) {
    gridBinary[pos.row][pos.column] = GridCellState::STAR;
  }

  TrueGrid::printTheMatrix(gridBinary);
}
void testMazes() {
  int testAmount{};
  std::wcout << "Please enter amount of tests: ";
  std::wcin >> testAmount;

  if (testAmount < 1) {
    std::wcout << "Invalid value.";
    testMazes();
    return;
  }

  int minRowCount{}, maxRowCount{};
  int minColumnCount{}, maxColumnCount{};

  std::wcout << "Enter minimum row count: ";
  std::cin >> minRowCount;
  std::wcout << "Enter maximum row count: ";
  std::cin >> maxRowCount;
  std::wcout << "Enter minimum column count: ";
  std::cin >> minColumnCount;
  std::wcout << "Enter maximum column count: ";
  std::cin >> maxColumnCount;

  int equalAmount{0};
  for (int i{0}; i < testAmount; ++i) {
    int rows{getRandom(minRowCount, maxRowCount)};
    int columns{getRandom(minColumnCount, maxColumnCount)};
    int rand1{getRandom(0, 3)};
    int rand2{getRandom(0, 3)};

    CellPos entryPoint{}, exitPoint{};
    if (rand1 == 0) {
      // Stick to upper side
      entryPoint.row = 0;
      entryPoint.column = getRandom(0, columns - 1);
    } else if (rand1 == 1) {
      // stick to right side
      entryPoint.row = getRandom(0, rows - 1);
      entryPoint.column = columns - 1;
    } else if (rand1 == 2) {
      // stick to bottom side
      entryPoint.row = rows - 1;
      entryPoint.column = getRandom(0, columns - 1);
    } else if (rand1 == 3) {
      // stick to left side
      entryPoint.row = getRandom(0, rows - 1);
      entryPoint.column = 0;
    }

    if (rand2 == 0) {
      // Stick to upper side
      exitPoint.row = 0;
      exitPoint.column = getRandom(0, columns - 1);
    } else if (rand1 == 1) {
      // stick to right side
      exitPoint.row = getRandom(0, rows - 1);
      exitPoint.column = columns - 1;
    } else if (rand1 == 2) {
      // stick to bottom side
      exitPoint.row = rows - 1;
      exitPoint.column = getRandom(0, columns - 1);
    } else if (rand1 == 3) {
      // stick to left side
      exitPoint.row = getRandom(0, rows - 1);
      exitPoint.column = 0;
    }

    bool falsePath{};

    Maze2 maze{rows, columns, entryPoint, exitPoint};
    maze.generate();
    bool_grid_t gridBinary;
    maze.getTrueGrid().getAsMatrix(gridBinary);

    auto euclerShortestDecider = [exitPoint](std::array<CellInfo, 4>& allPoints,
                                             CellPos pos,
                                             CellInfo& chosen) -> bool {
      if (exitPoint.row < pos.row && allPoints[0].valid)
        chosen = allPoints[0];
      else if (exitPoint.column > pos.column && allPoints[1].valid)
        chosen = allPoints[1];
      else if (exitPoint.row > pos.row && allPoints[2].valid)
        chosen = allPoints[2];
      else if (exitPoint.column < pos.column && allPoints[3].valid)
        chosen = allPoints[3];
      else
        return false;
      return true;
    };
    PathMatrixReturn pathAndMatrix = RobotPathFinder::solveMaze(
        gridBinary, entryPoint, exitPoint, euclerShortestDecider);

    Path& robotPath{pathAndMatrix.path};

    maze.getTrueGrid().getAsMatrix(gridBinary);
    Path theoricPath =
        MazeSolver::solveBinaryMaze(gridBinary, entryPoint, exitPoint);

    if (theoricPath.getLength() != robotPath.getLength()) {
      falsePath = true;
    } else {
      int length{theoricPath.getLength()};
      int i{0};
      while (i < length && robotPath.getStep(i) == theoricPath.getStep(i))
        ++i;
      if (i != length) {
        falsePath = true;
      } else {
        falsePath = false;
      }
    }

    if (!falsePath)
      equalAmount++;
  }
  printNormalisedVersion(equalAmount, testAmount);
}

void printNormalisedVersion(int value, int max) {
  int normalizedValue;
  int j;

  normalizedValue = (value * 100) / max;
  std::wcout << "Correctness [";
  for (j = 0; j < normalizedValue; j++)
    std::wcout << "*";
  for (; j < 100; j++)
    std::wcout << " ";

  std::wcout << "] " << value << "/" << max << "\n";

  std::wcout << "-----------------------------\n";
}