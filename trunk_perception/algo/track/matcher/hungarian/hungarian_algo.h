#pragma once

#include <time.h>
#include <iostream>
#include <limits>
#include <vector>

#include "trunk_perception/common/macros.h"

TRUNK_PERCEPTION_LIB_NAMESPACE_BEGIN

using namespace std;

class AssignmentProblemSolver {
 private:
  // --------------------------------------------------------------------------
  // Computes the optimal assignment (minimum overall costs) using Munkres
  // algorithm.
  // --------------------------------------------------------------------------
  void assignmentoptimal(int* assignment, double* cost, double* distMatrix, int nOfRows, int nOfColumns);
  void buildassignmentvector(int* assignment, bool* starMatrix, int nOfRows, int nOfColumns);
  void computeassignmentcost(int* assignment, double* cost, double* distMatrix, int nOfRows);
  void step2a(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix,
              bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
  void step2b(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix,
              bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
  void step3(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix,
             bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);
  void step4(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix,
             bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
  void step5(int* assignment, double* distMatrix, bool* starMatrix, bool* newStarMatrix, bool* primeMatrix,
             bool* coveredColumns, bool* coveredRows, int nOfRows, int nOfColumns, int minDim);

 public:
  enum TMethod { optimal, many_forbidden_assignments, without_forbidden_assignments };
  AssignmentProblemSolver() = default;
  ~AssignmentProblemSolver() = default;
  double Solve(const vector<vector<double> >& DistMatrix, vector<int>& Assignment, TMethod Method = optimal);
};

TRUNK_PERCEPTION_LIB_NAMESPACE_END