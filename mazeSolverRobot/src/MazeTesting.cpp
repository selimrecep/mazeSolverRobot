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

  entryPoint.row--;
  entryPoint.column--;
  exitPoint.row--;
  exitPoint.column--;

  Maze2 maze2{rows, columns, entryPoint, exitPoint};
  maze2.generate();
  maze2.printMaze();

  std::cout << "\n";

  maze2.printMaze();
  bool_grid_t gridBinary;
  maze2.getTrueGrid().getAsMatrix(gridBinary);

  maze2.getTrueGrid().printTheMatrix(gridBinary);

  optional_step_callback_t callback = [](bool_grid_t& grid) {
    static bool passOver{false};
    if (!passOver) {
      TrueGrid::printTheMatrix(grid);
      std::wcout << "-----------\n";
      std::wcout << "Press any key(after a key) to iterate for next step Write "
                    "'e' if want to pass over.\n ";
      std::wcin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

      // why doesnt work...
      wchar_t ch{};
      std::wcin >> ch;
      if (ch == 'e')
        passOver = true;
    }
  };
  Path path{
      RobotPathFinder::solveMaze(gridBinary, entryPoint, exitPoint, callback)};
  std::wcout << "Maze finished! Clearing matrix and printing final state: \n";

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
  std::cin >> testAmount;

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