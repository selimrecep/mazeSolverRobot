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

void Path::setStep(int index, CellPos pos) {
  if (index >= maxStepCount) {
    maxStepCount = (index + 2) * 2;
    pathSteps.resize(maxStepCount);
  }
  if (index + 1 > stepCount)
    stepCount = index + 1;
  pathSteps.at(index) = pos;
}

int Path::getLength() { return stepCount; }

void Path::empty() { stepCount = 0; }

CellPos Path::getStep(int index) {
  if (index > stepCount - 1)
    return {};

  return pathSteps[index];
}