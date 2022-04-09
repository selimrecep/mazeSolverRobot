#include "RobotPathFinder.h"
#include "TrueGrid.h"
#include <Maze2.h>
#include <MazeTesting.h>
#include <iostream>
#include <random.h>

void getDetailsFromUser() {
  int testAmount{};
  std::wcout << "Please enter amount of tests: ";
  std::cin >> testAmount;

  if (testAmount < 1) {
    std::wcout << "Invalid value.";
    getDetailsFromUser();
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
    Path robotPath =
        RobotPathFinder::solveMaze(gridBinary, entryPoint, exitPoint);
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