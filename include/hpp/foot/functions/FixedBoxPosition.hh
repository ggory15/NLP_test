#pragma once
#include <iostream>
#include <Eigen/Core>
#include <hpp/foot/utils/Box.hh>
#include <hpp/foot/utils/defs.hh>

namespace hpp {
namespace foot
{
class FixedBoxPosition
{
 public:
  FixedBoxPosition(const Eigen::Vector3d& targetPos);
  virtual ~FixedBoxPosition();

  void compute(Eigen::Ref<Eigen::Matrix<double, 3, 1>> res,
               const Eigen::Ref<const Eigen::Vector3d> pos) const;
  void diff(Eigen::Ref<Eigen::Matrix<double, 3, 3>> res) const;
  void LB(Eigen::Ref<Eigen::Matrix<double, 3, 1>> res) const;
  void UB(Eigen::Ref<Eigen::Matrix<double, 3, 1>> res) const;


  // This constraint fixes the position of a given box to a target position
  static void fillLinCstr(const Eigen::Vector3d& targetPos, RefVec lb, RefMat C, RefVec ub);

 private:
  Eigen::Vector3d targetPos_;
};
} /* feettrajectory */
}