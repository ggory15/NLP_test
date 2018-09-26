#pragma once

#include <iostream>
#include <Eigen/Core>

#include <hpp/foot/utils/Box.hh>
#include <hpp/foot/utils/FixedPlan.hh>
#include <hpp/foot/utils/PlanForHull.hh>
#include <hpp/foot/utils/ProblemConfig.hh>
#include <hpp/foot/functions/BoxAbovePlan.hh>
#include <hpp/foot/functions/BoxAboveFixedPlan.hh>
#include <hpp/foot/functions/FixedBoxPosition.hh>

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
//#include <ifopt_core/problem.h>
//#include <ifopt_ipopt/ipopt_solver.h>

namespace hpp
{
namespace foot
{
class BoxesHullTrajProblem
{
 public:
  BoxesHullTrajProblem(const std::string& configPath, const Index nBoxes, const Index nObstacles);
  virtual ~BoxesHullTrajProblem();

  void getManifoldsize(const Index& nBoxes,const Index& nObstacles);
  Eigen::VectorXd findInitPoint();

  void getTangentLB(RefVec out) const;
  void getTangentUB(RefVec out) const;

  //void evalObj(double& out, RefVec in) const;
  double evalObj(RefVec in);
  void evalObjDiff(RefMat out, RefVec in) const;

  void evalLinCstr(RefVec out, size_t i) const;
  void evalLinCstrDiff(RefMat out, size_t i) const;
  void getLinCstrLB(RefVec out, size_t i) const;
  void getLinCstrUB(RefVec out, size_t i) const;

  void evalNonLinCstr(RefVec out, RefVec in, size_t i) const;
  void evalNonLinCstrDiff(Eigen::SparseMatrix<double>& out, RefVec in, size_t i) const;
  void getNonLinCstrLB(RefVec out, size_t i) const;
  void getNonLinCstrUB(RefVec out, size_t i) const;

  size_t numberOfCstr() const;
  Index linCstrDim(size_t i) const;
  Index nonLinCstrDim(size_t i) const;

  void solveNLP(RefVec init, RefVec out) const;
  // void fileForMatlab(std::string fileName, const mnf::Point& x) const;

  std::string getCstrName(const size_t i) const;

  // getters
  const size_t& nBoxes() const { return nBoxes_; }
  const size_t& nPlans() const { return nPlans_; }
  const size_t& nObstacles() const { return nObstacles_; }
  const size_t& nFixedPlanes() const { return nFixedPlanes_; }
  const std::vector<Box>& boxes() const { return boxes_; }
  const std::vector<Box>& obstacles() const { return obstacles_; }
  const std::vector<FixedPlan>& fixedPlanes() const { return fixedPlanes_; }
  const std::vector<PlanForHull>& plans() const { return plans_; }
  const Eigen::Vector3d& initPos() const { return initPos_; }
  const Eigen::Vector3d& finalPos() const { return finalPos_; }
  const Eigen::Vector3d& boxSize() const { return boxSize_; }
  const std::vector<std::string> cstrNames() const { return cstrNames_; };
  const double& securityDistance() const { return securityDistance_; }
  Index dimVar() const { return manifold_size; }
  const ProblemConfig& config() const { return config_; }
  const double& getCost() const {return cost_;}
  const Eigen::MatrixXd& getCostDiff() const {return costDiff_;}

  Eigen::Vector3d getBoxPositionFromX(size_t i, const Eigen::VectorXd& x) const;
  void logAllX(const std::string& fileName, RefVec res) const;

 private:
  ProblemConfig config_;

  Eigen::Vector3d initPos_;
  Eigen::Vector3d finalPos_;

  std::vector<Box> obstacles_;
  std::vector<FixedPlan> fixedPlanes_;

  Index manifold_size;

  Eigen::Vector3d boxSize_;

  size_t nBoxes_;
  size_t nPlans_;
  size_t nObstacles_;
  size_t nFixedPlanes_;
  size_t nMobilePlanCstr_;
  size_t nFixedPlanCstr_;

  double securityDistance_;
  double maxStepHeight_;

  //double threshold_; //half of the min dimension of the box

  Box initBox_;
  BoxAbovePlan initBoxAbovePlanFct_;
  FixedBoxPosition fixedFinalBox_;

  std::vector<Box> boxes_;
  std::vector<PlanForHull> plans_;
  std::vector<BoxAboveFixedPlan> boxAboveFixedPlanFcts_;
  std::vector<BoxAbovePlan> boxAbovePlanFcts_;
  std::vector<BoxAbovePlan> obstacleAbovePlanFcts_;
  std::vector<std::string> cstrNames_;

  double cost_;
  Eigen::MatrixXd costDiff_;

  // buffers
  mutable Eigen::MatrixXd outRepObjDiff_;
  // mutable Eigen::MatrixXd outRepLinCstrDiff_;
  mutable Eigen::MatrixXd outRep_;
};
} /* feettrajectory */
}
