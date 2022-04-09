#include <CellPos.h>
#include <Path.h>

Path::Path(int maxStepCount) { pathSteps.resize(maxStepCount); }

void Path::addStep(CellPos pos) {
  if (stepCount == maxStepCount) {
    maxStepCount = (stepCount + 1) * 2;
    pathSteps.resize(maxStepCount);
  }

  pathSteps[stepCount++] = pos;
}

int Path::getLength() { return stepCount; }

void Path::empty() { stepCount = 0; }

CellPos Path::getStep(int index) {
  if (index > stepCount - 1)
    return {};

  return pathSteps[index];
}