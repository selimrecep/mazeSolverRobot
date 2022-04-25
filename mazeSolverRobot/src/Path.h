#pragma once
#include <CellPos.h>
#include <vector>

using steps_t = std::vector<CellPos>;

class Path {
private:
  steps_t pathSteps{};
  int maxStepCount{0};
  int stepCount{0};

public:
  Path(int maxStepCount);
  void addStep(CellPos pos);
  void setStep(int index, CellPos pos);
  CellPos getStep(int index);
  void empty();

  int getLength();

  /* Iterator */
  steps_t::iterator begin() { return pathSteps.begin(); }
  steps_t::iterator end() { return pathSteps.begin() + stepCount; }

  steps_t::const_iterator begin() const { return pathSteps.begin(); }
  steps_t::const_iterator end() const { return pathSteps.begin() + stepCount; }
};