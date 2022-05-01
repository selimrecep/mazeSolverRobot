#include "RobotPathFinder.h"
#include "TrueGrid.h"
#include <CellPos.h>
#include <Maze2.h>
#include <MazeAnalyzing.h>
#include <codecvt>
#include <fstream>
#include <iostream>
#include <locale>
#include <numeric>
#include <random.h>
#include <string>

void MazeAnalyzing::runAnalyzer(std::wostream& out, std::wistream& in) {
  int rows{}, columns{};

  out << "Enter rows: ";
  in >> rows;
  out << "Enter columns: ";
  in >> columns;

  int mazeCount{};
  out << "Amount of mazes will be generated: ";
  in >> mazeCount;

  wchar_t answer{};
  out << "Wanna use 'only shortest path' method?(y/n): ";
  in >> answer;

  bool onlyShortestPath = (answer == L'y');

  out << "How many new mazes you'd like to test with?: ";
  int testerMazeCount{};
  in >> testerMazeCount;

  out << "Generating " << mazeCount << " mazes\n";

  // Maze2** mazes = new Maze2*[mazeCount];
  std::vector<Maze2*> mazes(mazeCount);

  for (int i{0}; i < mazeCount; ++i) {
    CellPos enterPoint{0, getRandom(0, columns - 1)};
    CellPos exitPoint{rows - 1, getRandom(0, columns - 1)};

    mazes[i] = new Maze2{rows, columns, enterPoint, exitPoint};
    mazes[i]->generate();
  }

  out << "Generated. Solving mazes using RobotSolver(No euler/dummy solver) "
         "and saving "
         "paths\n";

  // Path** dummyPaths = new Path*[mazeCount];
  // int* dummyPathsStepCounts = new int[mazeCount]();
  std::vector<Path*> dummyPaths(mazeCount);
  std::vector<int> dummyPathsStepCounts(mazeCount);
  std::vector<int> dummyFirstPathStepCounts(mazeCount);

  for (int i{0}; i < mazeCount; ++i) {
    bool_grid_t binaryMatrix;
    CellPos entryPoint{}, exitPoint{};

    mazes[i]->getTrueGrid().getAsMatrix(binaryMatrix);
    mazes[i]->getEnterAndExitPoint(entryPoint, exitPoint);
    // Path path = MazeSolver::solveBinaryMaze(binaryMatrix, entryPoint,
    // exitPoint);
    decision_callback_t dummyDecider = [](std::array<CellInfo, 4>&, CellPos,
                                          CellInfo&) -> bool { return false; };

    bool hasEverFoundPath{false};
    optional_step_callback_t stepCounter =
        [&dummyPathsStepCounts, i, &dummyFirstPathStepCounts,
         &hasEverFoundPath](bool_grid_t& grid, bool usesOnlyShortestPath,
                            bool logNotShortest, int foundShorterPath) {
          if (!hasEverFoundPath) {
            if (foundShorterPath != -1)
              hasEverFoundPath = true;
            else
              dummyFirstPathStepCounts[i]++;
          }
          dummyPathsStepCounts[i]++;
          /*
                    TrueGrid::printTheMatrix(grid);
                    wchar_t ch{};
                    std::wcin >> ch;
                    if (ch == ',') {
                      std::wcout << "hmm\n";
                    }*/
        };

    PathMatrixReturn composite =
        RobotPathFinder::solveMaze(binaryMatrix, entryPoint, exitPoint,
                                   dummyDecider, onlyShortestPath, stepCounter);
    dummyPaths[i] = new Path{composite.path};
  }

  out << "Solved mazes and saved step counts with first method. Allocating "
         "history component: \n";

  // CellHistory** cellHistory = new CellHistory*[realRows];
  std::vector<std::vector<CellHistory*>> cellHistory(rows);

  for (int i{0}; i < rows; ++i) {
    cellHistory[i].resize(columns);
    for (int j{0}; j < columns; j++)
      cellHistory[i][j] = new CellHistory;
  }

  out << "Allocated. Parsing step counts obtained before into the history "
         "component: \n";

  for (int pathI{0}; pathI < mazeCount; ++pathI) {
    Path* path = dummyPaths[pathI];
    int pathLength{path->getLength()};
    for (int stepJ{0}; stepJ < pathLength - 1; ++stepJ) {
      CellPos from{path->getStep(stepJ)};
      CellPos to{path->getStep(stepJ + 1)};
      // Skip inbetween cells(that only has one way)
      if (from.row % 2 == 1 && from.column % 2 == 1) {
        Orientation goodRot = RobotPathFinder::getRotBetweenPoints(from, to);

        int originalRow((from.row - 1) / 2);
        int originalColumn((from.column - 1) / 2);
        cellHistory[originalRow][originalColumn]
            ->counts[static_cast<int>(goodRot)]++;
      }
    }
  }

  out << "Parsed.\n";

  out << "Creating odds/probability matrix:\n";
  odds_t odds;
  odds.resize(rows);
  for (int i{0}; i < rows; ++i) {
    odds[i].resize(columns);
  }

  for (int i{0}; i < rows; ++i) {
    for (int j{0}; j < columns; ++j) {

      cellHistory[i][j]->getChances(odds[i][j]);
    }
  }
  out << "Created probability matrix.\n";

  out << "Now using a decider using the history cell data (will log step "
         "counts too)\n";

  std::vector<Path*> historyBasedPaths(mazeCount);
  std::vector<int> historyBasedPathsStepCounts(mazeCount);
  std::vector<int> historyBasedFirstPathStepCounts(mazeCount);
  for (int i{0}; i < mazeCount; ++i) {
    bool_grid_t binaryMatrix;
    CellPos entryPoint{}, exitPoint{};

    mazes[i]->getTrueGrid().getAsMatrix(binaryMatrix);
    mazes[i]->getEnterAndExitPoint(entryPoint, exitPoint);
    // Path path = MazeSolver::solveBinaryMaze(binaryMatrix, entryPoint,
    // exitPoint);
    decision_callback_t historicalDecider =
        [&odds](std::array<CellInfo, 4>& allCells, CellPos currentPos,
                CellInfo& chosen) -> bool {
      // Not possible between walls state
      if (currentPos.row % 2 == 0 || currentPos.column % 2 == 0)
        return false;

      CellPos currentPosNormalised{(currentPos.row - 1) / 2,
                                   (currentPos.column - 1) / 2};

      std::vector<odd_t>& subOdds =
          odds[currentPosNormalised.row][currentPosNormalised.column];

      Orientation bestOrientation{Orientation::unkown};
      double bestOdd = -1;
      for (int i{0}; i < 4; ++i) {
        if (allCells[i].valid) {
          if (subOdds[i] > bestOdd) {
            bestOrientation = static_cast<Orientation>(i);
            bestOdd = subOdds[i];
          }
        }
      };

      if (bestOrientation == Orientation::unkown) {
        return false;
      } else {
        chosen.valid = true;
        // chosen.cellState = // this, isn't needed, I guess? lol
        chosen.pos =
            RobotPathFinder::getNextPointWithRot(currentPos, bestOrientation);
        return true;
      }
    };

    bool hasEverFoundPath{false};
    optional_step_callback_t stepCounter =
        [&historyBasedPathsStepCounts, i, &in, &hasEverFoundPath,
         &historyBasedFirstPathStepCounts](
            bool_grid_t& grid, bool usesOnlyShortestPath, bool logNotShortest,
            int foundShorterPath) {
          if (!hasEverFoundPath) {
            if (foundShorterPath != -1)
              hasEverFoundPath = true;
            else
              historyBasedFirstPathStepCounts[i]++;
          }
          historyBasedPathsStepCounts[i]++;
          /*TrueGrid::printTheMatrix(grid);
          wchar_t ch{};
          std::wcin >> ch;
          if (ch == ',') {
            std::wcout << "hmm\n";
          }*/
        };

    PathMatrixReturn composite = RobotPathFinder::solveMaze(
        binaryMatrix, entryPoint, exitPoint, /*dummyDecider*/ historicalDecider,
        onlyShortestPath, stepCounter);
    historyBasedPaths[i] = new Path{composite.path};
  }

  out << "Solving mazes with history cells is done.\n";
  out << "Now calculating average steps, for dummy and historic: \n";
  double totalVisitDummyAverage{};
  double totalVisitHistoricAverage{};

  double firstPathDummyAverage{};
  double firstPathHistoricAverage{};

  totalVisitDummyAverage =
      std::accumulate(std::begin(dummyPathsStepCounts),
                      std::end(dummyPathsStepCounts), 0.0) /
      mazeCount;
  totalVisitHistoricAverage =
      std::accumulate(std::begin(historyBasedPathsStepCounts),
                      std::end(historyBasedPathsStepCounts), 0.0) /
      mazeCount;

  firstPathDummyAverage =
      std::accumulate(std::begin(dummyFirstPathStepCounts),
                      std::end(dummyFirstPathStepCounts), 0.0) /
      mazeCount;
  firstPathHistoricAverage =
      std::accumulate(std::begin(historyBasedFirstPathStepCounts),
                      std::end(historyBasedFirstPathStepCounts), 0.0) /
      mazeCount;

  out << "------------- ON OLD MAZE SET -------------\n";
  out << "TOTAL visiting averages: (For the oldly generated mazes), "
         "dummyAverage: "
      << totalVisitDummyAverage
      << ", historicAverage: " << totalVisitHistoricAverage
      << " In percent(historic/dummy): "
      << ((totalVisitHistoricAverage / totalVisitDummyAverage) * 100) << "% \n";

  out << "FIRST PATH ONLY visiting averages: (For the oldly generated mazes), "
         "dummyAverage: "
      << firstPathDummyAverage
      << ", historicAverage: " << firstPathHistoricAverage
      << " In percent(historic/dummy): "
      << ((firstPathHistoricAverage / firstPathDummyAverage) * 100) << "% \n";
  out << "--------------------------------------------\n";
  // ------------------------TESTING----------------------
  out << "Now generating new maze sets: \n";
  std::vector<Maze2*> testMazes(testerMazeCount);

  for (int i{0}; i < testerMazeCount; ++i) {
    CellPos enterPoint{0, getRandom(0, columns - 1)};
    CellPos exitPoint{rows - 1, getRandom(0, columns - 1)};

    testMazes[i] = new Maze2{rows, columns, enterPoint, exitPoint};
    testMazes[i]->generate();
  }

  out << "Generated. \n";
  std::vector<Path*> TESTdummyPaths(testerMazeCount);
  std::vector<int> TESTdummyPathsStepCounts(testerMazeCount);
  std::vector<int> TESTdummyFirstPathStepCounts(testerMazeCount);

  for (int i{0}; i < testerMazeCount; ++i) {
    bool_grid_t binaryMatrix;
    CellPos entryPoint{}, exitPoint{};

    testMazes[i]->getTrueGrid().getAsMatrix(binaryMatrix);
    testMazes[i]->getEnterAndExitPoint(entryPoint, exitPoint);
    // Path path = MazeSolver::solveBinaryMaze(binaryMatrix, entryPoint,
    // exitPoint);
    decision_callback_t dummyDecider = [](std::array<CellInfo, 4>&, CellPos,
                                          CellInfo&) -> bool { return false; };

    bool hasEverFoundPath{false};
    optional_step_callback_t stepCounter =
        [&TESTdummyPathsStepCounts, i, &TESTdummyFirstPathStepCounts,
         &hasEverFoundPath](bool_grid_t& grid, bool usesOnlyShortestPath,
                            bool logNotShortest, int foundShorterPath) {
          if (!hasEverFoundPath) {
            if (foundShorterPath != -1)
              hasEverFoundPath = true;
            else
              TESTdummyFirstPathStepCounts[i]++;
          }
          TESTdummyPathsStepCounts[i]++;
        };

    PathMatrixReturn composite =
        RobotPathFinder::solveMaze(binaryMatrix, entryPoint, exitPoint,
                                   dummyDecider, onlyShortestPath, stepCounter);
    TESTdummyPaths[i] = new Path{composite.path};
  }
  out << "Solved TEST mazes and saved step counts with first method:\n ";

  out << "Now using a decider using the history cell data (will log step "
         "counts too)\n";

  std::vector<Path*> TESThistoryBasedPaths(testerMazeCount);
  std::vector<int> TESThistoryBasedPathsStepCounts(testerMazeCount);
  std::vector<int> TESThistoryBasedFirstPathStepCounts(testerMazeCount);
  for (int i{0}; i < testerMazeCount; ++i) {
    bool_grid_t binaryMatrix;
    CellPos entryPoint{}, exitPoint{};

    testMazes[i]->getTrueGrid().getAsMatrix(binaryMatrix);
    testMazes[i]->getEnterAndExitPoint(entryPoint, exitPoint);
    // Path path = MazeSolver::solveBinaryMaze(binaryMatrix, entryPoint,
    // exitPoint);
    decision_callback_t historicalDecider =
        [&odds](std::array<CellInfo, 4>& allCells, CellPos currentPos,
                CellInfo& chosen) -> bool {
      // Not possible between walls state
      if (currentPos.row % 2 == 0 || currentPos.column % 2 == 0)
        return false;

      CellPos currentPosNormalised{(currentPos.row - 1) / 2,
                                   (currentPos.column - 1) / 2};

      std::vector<odd_t>& subOdds =
          odds[currentPosNormalised.row][currentPosNormalised.column];

      Orientation bestOrientation{Orientation::unkown};
      double bestOdd = -1;
      for (int i{0}; i < 4; ++i) {
        if (allCells[i].valid) {
          if (subOdds[i] > bestOdd) {
            bestOrientation = static_cast<Orientation>(i);
            bestOdd = subOdds[i];
          }
        }
      };

      if (bestOrientation == Orientation::unkown) {
        return false;
      } else {
        chosen.valid = true;
        // chosen.cellState = // this, isn't needed, I guess? lol
        chosen.pos =
            RobotPathFinder::getNextPointWithRot(currentPos, bestOrientation);
        return true;
      }
    };

    bool hasEverFoundPath{false};
    optional_step_callback_t stepCounter =
        [&TESThistoryBasedPathsStepCounts, i, &in, &hasEverFoundPath,
         &TESThistoryBasedFirstPathStepCounts](
            bool_grid_t& grid, bool usesOnlyShortestPath, bool logNotShortest,
            int foundShorterPath) {
          if (!hasEverFoundPath) {
            if (foundShorterPath != -1)
              hasEverFoundPath = true;
            else
              TESThistoryBasedFirstPathStepCounts[i]++;
          }
          TESThistoryBasedPathsStepCounts[i]++;
        };

    PathMatrixReturn composite = RobotPathFinder::solveMaze(
        binaryMatrix, entryPoint, exitPoint, /*dummyDecider*/ historicalDecider,
        onlyShortestPath, stepCounter);
    TESThistoryBasedPaths[i] = new Path{composite.path};
  }

  out << "Done finding paths using history cells. \n";
  double TESTtotalVisitDummyAverage{};
  double TESTtotalVisitHistoricAverage{};

  double TESTfirstPathDummyAverage{};
  double TESTfirstPathHistoricAverage{};

  TESTtotalVisitDummyAverage =
      std::accumulate(std::begin(TESTdummyPathsStepCounts),
                      std::end(TESTdummyPathsStepCounts), 0.0) /
      testerMazeCount;
  TESTtotalVisitHistoricAverage =
      std::accumulate(std::begin(TESThistoryBasedPathsStepCounts),
                      std::end(TESThistoryBasedPathsStepCounts), 0.0) /
      testerMazeCount;

  TESTfirstPathDummyAverage =
      std::accumulate(std::begin(TESTdummyFirstPathStepCounts),
                      std::end(TESTdummyFirstPathStepCounts), 0.0) /
      testerMazeCount;
  TESTfirstPathHistoricAverage =
      std::accumulate(std::begin(TESThistoryBasedFirstPathStepCounts),
                      std::end(TESThistoryBasedFirstPathStepCounts), 0.0) /
      testerMazeCount;

  out << "------------- ON NEW MAZE SET -------------\n";
  out << "TOTAL visiting averages: (For the oldly generated mazes), "
         "dummyAverage: "
      << TESTtotalVisitDummyAverage
      << ", historicAverage: " << totalVisitHistoricAverage
      << " In percent(historic/dummy): "
      << ((TESTtotalVisitHistoricAverage / TESTtotalVisitDummyAverage) * 100)
      << "% \n";

  out << "FIRST PATH ONLY visiting averages: (For the oldly generated mazes), "
         "dummyAverage: "
      << firstPathDummyAverage
      << ", historicAverage: " << firstPathHistoricAverage
      << " In percent(historic/dummy): "
      << ((TESTfirstPathHistoricAverage / TESTfirstPathDummyAverage) * 100)
      << "% \n";
  out << "--------------------------------------------\n";

  MazeAnalyzing::exportData(cellHistory, odds);

  out << "Enter 'e' if you want to exit, enter something else(a char plz) if "
         "you want to continue testing.\n";

  wchar_t ch{};
  in >> ch;
  if (ch == L'e') {
    std::exit(0);
  }

  out << "Use only shortest paths?(y/n)\n";
  bool shortPathOnly{};
  in >> ch;

  shortPathOnly = (ch == L'y');

  // Continue testing on a random maze
  out << "Choosing a random maze(from test set). (ENTER 'e' WITHOUT QUOTES, to "
         "finish fast) \n";
  int mazeId{getRandom(0, testerMazeCount - 1)};

  // Shorten ugly dereference operator for future uses
  Maze2& maze = (*testMazes[mazeId]);

  MazeAnalyzing::testSingleMaze(out, in, shortPathOnly, maze, odds);
  int aaaa;
  in >> aaaa;
}

void MazeAnalyzing::exportData(
    std::vector<std::vector<CellHistory*>>& cellHistory, odds_t& odds) {
  std::wcout << "Base path for outputting files: ";
  std::wstring wbaseFileName{};
  std::wcin >> wbaseFileName;

  // oh god...

  using convert_type = std::codecvt_utf8<wchar_t>;
  std::wstring_convert<convert_type, wchar_t> converter;

  std::string baseFileName(converter.to_bytes(wbaseFileName));

  std::ofstream file;

  std::string firstDataFileName{baseFileName + "_base.csv"};
  file.open(firstDataFileName);

  file << ",";
  int size(cellHistory[0].size());
  for (int i{1}; i <= size; ++i) {
    file << "C" << i;
    if (i != size)
      file << ",";
  }
  file << "\n";

  int tmpSize(cellHistory.size());
  for (int i{0}; i < tmpSize; ++i) {
    file << "R" << (i + 1) << ",";

    std::vector<double> chances;

    for (int j{0}; j < size; ++j) {
      file << static_cast<int>(cellHistory[i][j]->getChances(chances));
      if (j != size - 1) {
        file << ",";
      }
    }

    file << "\n";
  }

  file.close();

  std::string rawDataFileName{baseFileName + "_raw.csv"};
  file.open(rawDataFileName);

  // bleeding eyes
  size_t rowCount{odds.size()};
  size_t sizeOdds{odds[0].size()};

  for (size_t i{0}; i < rowCount; ++i) {
    for (size_t j{0}; j < sizeOdds; j++) {
      file << i << "," << j << ",";
      for (int k{0}; k < 4; ++k) {
        file << std::to_string(odds[i][j][k]);
        if (k != 3)
          file << ",";
      }
      file << "\n";
    }
  }

  file.close();

  std::string rotOnlyDataFileName{baseFileName + "_rotOnly.csv"};
  file.open(rotOnlyDataFileName);

  for (size_t i{0}; i < rowCount; ++i) {
    for (size_t j{0}; j < sizeOdds; j++) {
      file << i << "," << j << ",";
      std::vector<double> chances;
      file << static_cast<int>(cellHistory[i][j]->getChances(chances));
      if (j != sizeOdds - 1)
        file << ",";

      file << "\n";
    }
  }

  file.close();
}
void MazeAnalyzing::testSingleMaze(std::wostream& out, std::wistream& in,
                                   bool shortPathOnly, Maze2& maze,
                                   odds_t& odds) {
  bool_grid_t binaryMatrix;
  CellPos entryPoint{}, exitPoint{};

  maze.getTrueGrid().getAsMatrix(binaryMatrix);
  maze.getEnterAndExitPoint(entryPoint, exitPoint);
  // Path path = MazeSolver::solveBinaryMaze(binaryMatrix, entryPoint,
  // exitPoint);
  bool letItGo{false};
  decision_callback_t historicalDecider =
      [&odds, &out, &in, &letItGo](std::array<CellInfo, 4>& allCells,
                                   CellPos currentPos,
                                   CellInfo& chosen) -> bool {
    if (!letItGo) {
      out << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n";
    }
    // Not possible between walls state
    if (currentPos.row % 2 == 0 || currentPos.column % 2 == 0) {
      if (!letItGo) {
        out << "Not deducable cell(index isn't odd num). Normal selection.\n";
        wchar_t ch{};
        in >> ch;
        if (ch == L'e')
          letItGo = true;
      }
      return false;
    }

    CellPos currentPosNormalised{(currentPos.row - 1) / 2,
                                 (currentPos.column - 1) / 2};

    std::vector<odd_t>& subOdds =
        odds[currentPosNormalised.row][currentPosNormalised.column];

    Orientation bestOrientation{Orientation::unkown};
    double bestOdd = -1;
    int validCount{0};

    for (int i{0}; i < 4; ++i) {
      if (allCells[i].valid) {
        if (subOdds[i] > bestOdd) {
          bestOrientation = static_cast<Orientation>(i);
          bestOdd = subOdds[i];
        }
        ++validCount;
      }
    };

    if (bestOrientation == Orientation::unkown) {
      if (!letItGo) {
        out << "Couldn't deduce anything, unkown orientation. Normal deduce.\n";
        wchar_t ch{};
        in >> ch;
        if (ch == L'e')
          letItGo = true;
      }
      return false;
    } else {
      if (validCount == 1) {
        if (!letItGo) {
          out << "Only one valid possible cell. Don't deduce, let the main "
                 "decider decide\n";
          wchar_t ch{};
          in >> ch;
          if (ch == L'e')
            letItGo = true;
        }
        return false;
      } else {
        if (!letItGo) {
          out << "Deduced orientation: "
              << MazeAnalyzing::rotToStr(bestOrientation) << "\n";
          out << "From chance list: \n";

          for (int i{0}; i < 4; ++i) {
            double odd{subOdds[i]};
            out << MazeAnalyzing::rotToStr(static_cast<Orientation>(i)) << ": "
                << odd << " (" << (odd * 100) << "%)\n";
          }
          out << "\n";
          wchar_t ch{};
          in >> ch;
          if (ch == L'e')
            letItGo = true;
        }
        chosen.valid = true;
        // chosen.cellState = // this, isn't needed, I guess? lol
        chosen.pos =
            RobotPathFinder::getNextPointWithRot(currentPos, bestOrientation);
        return true;
      }
    }
  };

  bool hasEverFoundPath{false};
  optional_step_callback_t stepCounter =
      [&in, &out, &hasEverFoundPath,
       &letItGo](bool_grid_t& grid, bool usesOnlyShortestPath,
                 bool logNotShortest, int foundShorterPath) {
        if (!hasEverFoundPath) {
          if (foundShorterPath != -1) {
            out << "Found first short path!\n";
            hasEverFoundPath = true;
          }
        }
        if (!letItGo) {
          out << "--------------------------\n\n";

          TrueGrid::printTheMatrix(grid);
        }
      };

  PathMatrixReturn composite = RobotPathFinder::solveMaze(
      binaryMatrix, entryPoint, exitPoint, /*dummyDecider*/ historicalDecider,
      shortPathOnly, stepCounter);

  std::wcout << "Maze finished! Clearing matrix and printing final state: \n";
  std::wcout << "Robot matrix:\n";
  TrueGrid::printTheMatrix(composite.robotMatrix);

  std::wcout << "cleared state: \n";

  TrueGrid& grid = maze.getTrueGrid();
  grid.getAsMatrix(binaryMatrix);

  for (auto& pos : composite.path) {
    binaryMatrix[pos.row][pos.column] = GridCellState::STAR;
  }

  TrueGrid::printTheMatrix(binaryMatrix);
}

CellHistory::CellHistory() { counts.resize(4); }
Orientation CellHistory::getChances(std::vector<double>& chances) {
  int64_t totalCount{0};
  int64_t max = {0};
  Orientation rot = Orientation::unkown;

  chances.resize(4);

  totalCount = std::accumulate(std::begin(counts), std::end(counts), 0);

  for (int i{0}; i < 4; ++i) {
    int count = counts[i];
    if (max < count) {
      max = count;
      rot = static_cast<Orientation>(i);
    }
    chances[i] = ((double)count) / totalCount;
  }

  return rot;
}

std::wstring MazeAnalyzing::rotToStr(Orientation rot) {
  switch (rot) {
  case Orientation::up:
    return L"UP";
  case Orientation::right:
    return L"RIGHT";
  case Orientation::down:
    return L"DOWN";
  case Orientation::left:
    return L"LEFT";
  default:
    return L"UNKOWN";
  }
}