#pragma once
#include <iostream>

#include <hpp/foot/solvers/QP.hh>
#include <hpp/foot/utils/defs.hh>
#include <hpp/foot/TrajectoryProblem.hh>
#include <hpp/foot/functions/PlanBetweenBoxAndObstacle.hh>

namespace hpp{
namespace foot
{
class QPBoxesFixed : public QP
{
 public:
  QPBoxesFixed(const TrajectoryProblem& pb);
  virtual ~QPBoxesFixed();
  void addRelaxationTerm(const double& alpha);
  void formQP(ConstRefVec boxesVariables, ConstRefVec xPreviousPlanes);

 private:
  const TrajectoryProblem& pb_;
  Eigen::Vector3d getBoxPos(Index iBox, ConstRefVec xBoxes);
};

} /* feettrajectory */
}