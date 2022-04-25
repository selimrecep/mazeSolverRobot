
#include "TrueGrid.h"
#include <Maze2.h>
#include <MazeAnalyzing.h>
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

  std::wcout << "Welcome, to run first revision enter 1, for second enter 2: ";
  wchar_t revSelect{};
  std::wcin >> revSelect;

  if (revSelect == L'1') {

    std::wcout
        << "To run iteration method choose 1, to run overall autoed tests "
           "choose 2\n";
    int mode{};
    std::wcin >> mode;
    if (mode == 1) {
      testSingleMaze();
    } else {
      testMazes();
    }
  } else {
    MazeAnalyzing::runAnalyzer();
  }

  std::wcout << "Press a key and hit enter to exit.\n";
  char ch{};
  std::cin >> ch;
  std::wcout << ch;
}
