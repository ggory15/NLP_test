#pragma once
#include <iostream>
#include <Eigen/Core>
#include <hpp/foot/utils/Box.hh>
#include <hpp/foot/utils/defs.hh>

namespace hpp{
namespace foot
{
class PlanBetweenBoxAndObstacle
{
 public:
  PlanBetweenBoxAndObstacle();
  virtual ~PlanBetweenBoxAndObstacle();
  // This constraint has the effect of setting the distance of the plan to the
  // minimum possible value, which is computed in updatePlanD.
  // It is the charge of the user to update the values of the distances
  static void fillLinCstr(const Box& box, const Box& obstacle,
                          const double& planD, const Eigen::Vector3d& planN,
                          RefVec lb, RefMat C, double securityDistance = 0);
  static void updatePlanD(const Box& obstacle, double& planD,
                          const Eigen::Vector3d& planN,
                          const double& securityDistance);

 private:
  /* data */
};
} /* feettrajectory */
}