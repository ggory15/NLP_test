#pragma once
#include <iostream>

#include <hpp/foot/solvers/QP.hh>
#include <hpp/foot/utils/defs.hh>
#include <hpp/foot/TrajectoryProblem.hh>
#include <hpp/foot/functions/PlanBetweenBoxAndObstacle.hh>

namespace hpp{
namespace foot
{
class QPBoxesFixedIndividual : public QP
{
 public:
  QPBoxesFixedIndividual(const TrajectoryProblem& pb);
  virtual ~QPBoxesFixedIndividual();
  void addRelaxationTerm(const double& alpha);
  void formQP(const size_t& iPlan, ConstRefVec boxesVariables, ConstRefVec xPreviousPlanes);

 private:
  const TrajectoryProblem& pb_;
  Eigen::Vector3d getBoxPos(Index iBox, ConstRefVec xBoxes);
};

} /* feettrajectory */
}

