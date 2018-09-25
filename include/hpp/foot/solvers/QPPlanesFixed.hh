#pragma once

#include <iostream>
#include <hpp/foot/solvers/QP.hh>
#include <hpp/foot/utils/defs.hh>
#include <hpp/foot/TrajectoryProblem.hh>
#include <hpp/foot/functions/PlanBetweenBoxAndObstacle.hh>

namespace hpp{
namespace foot
{
class QPPlanesFixed : public QP
{
 public:
  QPPlanesFixed(const TrajectoryProblem& pb);
  virtual ~QPPlanesFixed();
  //void addRelaxationTerm(const double& alpha);
  void addRelaxationTerm(size_t cstrIndexBegin, size_t cstrSize, bool isVirtual);
  void formQP(ConstRefVec planVariables);
  void updatePlanD(RefVec planVariables);

 private:
  const TrajectoryProblem& pb_;
  /// @brief relaxation term for non virtual collision avoidance
  double alpha_ = 1000;
  /// @brief relaxation term for virtual collision avoidance
  double alphaVirtual_ = 100;
};

} /* feettrajectory */
}

