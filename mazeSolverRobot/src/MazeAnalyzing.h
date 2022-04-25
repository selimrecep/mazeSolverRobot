#pragma once
#include <RobotPathFinder.h>
#include <iostream>
#include <string>
#include <vector>

using odd_t = double;

struct CellHistory {
  std::vector<int> counts;

  CellHistory();
  Orientation getChances(std::vector<double>& chances);
};

class MazeAnalyzing {

private:
  static std::wstring rotToStr(Orientation rot);
  static void
  testSingleMaze(std::wostream& out, std::wistream& in, bool shortPathOnly,
                 Maze2& maze,
                 std::vector<std::vector<std::vector<odd_t>>>& odds);

public:
  static void runAnalyzer(std::wostream& out = std::wcout,
                          std::wistream& in = std::wcin);
};