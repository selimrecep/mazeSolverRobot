
#include "TrueGrid.h"
#include <Maze2.h>
#include <MazeSolver.h>
#include <MazeTesting.h>
#include <RobotPathFinder.h>
#include <codecvt>
#include <fcntl.h>
#include <io.h>
#include <iostream>
#include <locale>
#include <string>

int main() {
  _setmode(_fileno(stdout), _O_U16TEXT);
  getDetailsFromUser();
  /*CellPos entryPoint{0, 5};
  CellPos exitPoint{4, 8};

  Maze2 maze2{5, 10, entryPoint, exitPoint};
  maze2.generate();
  maze2.printMaze();

  std::cout << "\n";

  maze2.printMaze();
  bool_grid_t gridBinary;
  maze2.getTrueGrid().getAsMatrix(gridBinary);


  maze2.getTrueGrid().printTheMatrix(gridBinary);

  optional_step_callback_t callback = [](bool_grid_t& grid) {
    TrueGrid::printTheMatrix(grid);
    std::wcout << "-----------\n";

    char ch{};
    std::cin >> ch;
  };
  Path path{
      RobotPathFinder::solveMaze(gridBinary, entryPoint, exitPoint,
  callback)}; std::wcout << "Maze finished! Clearing matrix and printing
  final state: \n";

  TrueGrid& grid = maze2.getTrueGrid();
  grid.getAsMatrix(gridBinary);

  for (auto& pos : path) {
    gridBinary[pos.row][pos.column] = GridCellState::BEEN_VISITED;
  }

  TrueGrid::printTheMatrix(gridBinary);
*/
  char ch{};
  std::cin >> ch;
}
