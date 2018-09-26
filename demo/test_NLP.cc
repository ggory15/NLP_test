#include <iostream>
#include <cstdlib>
#include <ctime>

#include <hpp/foot/utils/Printer.hh>
#include <hpp/foot/BoxesHullTrajProblem.hh>
#include <hpp/foot/utils/ProblemConfig.hh>

#include <hpp/foot/solvers/NLP/NLP_formulation.hh>
#include <hpp/foot/solvers/NLP/NLP_variable.hh>
#include <hpp/foot/solvers/NLP/NLP_cost.hh>
#include <hpp/foot/solvers/NLP/NLP_constraint.hh>

#include <ifopt/ipopt_solver.h>

#include "ifopt/test_vars_constr_cost.h"

using namespace ifopt;
using namespace std;
using namespace hpp::foot;
using VariablePtrVec = ifopt::VariableSet::Ptr;
using ContraintPtrVec = ifopt::ConstraintSet::Ptr;
using CostPtrVec = ifopt::CostTerm::Ptr;

int main(int argc, char* argv[])
{
  std::string ymlPath;
  if (argc > 1)
  {
    ymlPath = std::string(CONFIGS_DATA_DIR) + "/" + argv[1] + ".yml";
  }
  else
  {
    std::cout << "Loading default file \"singleObstacle.yml\"" << std::endl;
    ymlPath = std::string(CONFIGS_DATA_DIR) + "/singleObstacle.yml";
  }
  ProblemConfig config(ymlPath);
  
  int nBoxes = config["nBoxes"]; // 12
  int nObstacles = 0;
  if (config.has("obstacles")) // 1
    nObstacles = static_cast<int>(config["obstacles"].asVecBox().size());

  BoxesHullTrajProblem myProb(ymlPath, nBoxes, nObstacles);
    // Initialization
  Eigen::VectorXd initVec(myProb.dimVar());
  if (myProb.config().has("x0"))
  {
    initVec << myProb.config()["x0"].asVectorXd();
    std::cout << "myProb.config()[x0].asVectorXd(): "
              << myProb.config()["x0"].asVectorXd().transpose() << std::endl;
  }
  else
    initVec = myProb.findInitPoint();

  if (myProb.config().has("initialGuessRandomFactor"))
  {
    initVec = initVec +
              myProb.config()["initialGuessRandomFactor"] *
                  Eigen::VectorXd::Random(myProb.dimVar());
  }
//   Eigen::VectorXd out(8);
//   out.setZero();
//   for (int i=0; i<37; i++){
//     if (i == 12)
//       out.resize(3);
//     else if (i<25 && i>12)
//       out.resize(24);
//     else if (i>=25 && i<37)
//       out.resize(1);

//     myProb.evalNonLinCstr(out, initVec, i);
//     myProb.getNonLinCstrUB(out, i);
//     myProb.getNonLinCstrLB(out, i);
//   }
//   double cost;
//  // myProb.evalObj(cost, initVec);
//   Eigen::MatrixXd cost_diff(1, 108);
//   cost_diff.setZero();
//   myProb.evalObjDiff(cost_diff, initVec);




  std::shared_ptr<NLPVariables> vars(new NLPVariables("var_set1", myProb.dimVar()));
  vars->SetInitVariable(initVec);
  VecBound vars_bound(myProb.dimVar());
  for (int i=0; i< myProb.dimVar(); i++)
    vars_bound.at(i) = NoBound;//(-1.0, 1.0);;  
  vars->SetBounds(vars_bound);
  
  std::shared_ptr<NLPCosts> costs(new NLPCosts(&myProb));
  costs->SetStepSize(nBoxes);
  
  using ContraintPtrVec  = std::vector<ifopt::ConstraintSet::Ptr>;
  ContraintPtrVec ConstraintsGroup;
  for (int i=1; i<37; i++)
    ConstraintsGroup.push_back(make_shared<NLPConstraints>(&myProb, i, "constset1"));
  
  
  ifopt::Problem nlp;

  nlp.AddVariableSet(vars);
  nlp.AddCostSet(costs);
  //nlp.AddConstraintSet(std::make_shared<ExConstraint>());
  for (auto c: ConstraintsGroup)
     nlp.AddConstraintSet(c);
  //nlp.AddConstraintSet(constraints);
  //nlp.AddConstraintSet(constraints2);

  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("tol", 0.0000001);
  //solver->SetOption("jacobian_approximation", "finite-difference-values");
  solver->SetOption("linear_solver", "ma27");
  solver->SetOption("jacobian_approximation", "exact");
  solver->SetOption("max_iter", 30000);
  solver->Solve(nlp);
  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
  std::cout << "sol" << x.head(36).transpose() << std::endl;  
  myProb.logAllX("/home/skim/foot_data/", x);
  //std::cout << "ini" << initVec.head(36).transpose() << std::endl;
//  std::cout << "init_step" << myProb.initPos().transpose() << std:: endl;

//   ifopt::Problem nlp;
  // vars->SetInitVariable(initVec);
  // VecBound vars_bound(myProb.dimVar());
  // for (int i=0; i< myProb.dimVar(); i++)
  //   vars_bound.at(i) = NoBound;//(-1.0, 1.0);;  

  // vars->SetBounds(vars_bound);
  // costs->SetInitState(myProb.initPos());  
  // costs->SetStepSize(nBoxes);
  // ifopt::Problem nlp;

  // nlp.AddVariableSet(vars);
  // nlp.AddCostSet(costs);
  // nlp.AddConstraintSet(std::make_shared<ExConstraint>());

  // auto solver = std::make_shared<ifopt::IpoptSolver>();
  // // solver->SetOption("linear_solver", "ma27");
  // // solver->SetOption("tol", 0.001);
  // // solver->SetOption("jacobian_approximation", "exact");
  // // solver->SetOption("mu_strategy", "adaptive");
  // // solver->SetOption("nlp_scaling_method", "gradient-based");
  // //solver->SetOption("max_iter", 3000);
  // //solver->SetOption("hessian_approximation", "exact");
  // //solver->SetOption("print_level", 8);

  // solver->Solve(nlp);
  // Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
  // std::cout << "sol" << x.head(36).transpose() << std::endl;  
  // std::cout << "ini" << initVec.head(36).transpose() << std::endl;
  // std::cout << "init_step" << myProb.initPos().transpose() << std:: endl;
//   ifopt::Problem nlp;

//   //nlp.AddVariableSet(formulation.GetVariableSets());
//   nlp.AddVariableSet(std::make_shared<ExVariables>());
//   nlp.AddConstraintSet(std::make_shared<ExConstraint>());
//   nlp.AddCostSet      (std::make_shared<ExCost>());
//   nlp.PrintCurrent();

//   auto solver = std::make_shared<ifopt::IpoptSolver>();
//   solver->SetOption("linear_solver", "ma27");
//   solver->SetOption("jacobian_approximation", "finite-difference-values"); // "finite difference-values"
//   solver->SetOption("jacobian_approximation", "exact");
// //  solver->SetOption("limited_memory_initialization", "constant");
//   solver->SetOption("max_cpu_time", 20.0);
//   solver->SetOption("mu_strategy", "adaptive");
//   //solver->SetOption("print_level", 8);

//   solver->Solve(nlp);
  

//   Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
//   std::cout << x.head(10).transpose() << std::endl;  




  // Eigen::VectorXd initVec(myProb.dimVar());
  // if (myProb.config().has("x0"))
  // {
  //   initVec << myProb.config()["x0"].asVectorXd();
  //   std::cout << "myProb.config()[x0].asVectorXd(): "
  //             << myProb.config()["x0"].asVectorXd().transpose() << std::endl;
  // }
  // else
  //   initVec = myProb.findInitPoint();

  // if (myProb.config().has("initialGuessRandomFactor"))
  // {
  //   initVec = initVec +
  //             myProb.config()["initialGuessRandomFactor"] *
  //                 Eigen::VectorXd::Random(myProb.dimVar());
  // }
  // Eigen::VectorXd x_result;
  

  

  //myProb.solveNLP(initVec, x_result);
  
  //myProb.M().forceOnM(initVec, initVec);
  // //////////////////////////////////////////////////////////////////////////////////////////////////
  // // 1. define the problem
  // Problem nlp;
  // std::shared_ptr<ExVariables> spw(new ExVariables(2));
  // Problem::VecBound bounds(2);
  // bounds.at(0) = Bounds(-1.0, 1.0);
  // bounds.at(1) = Bounds(-1.0, 0.0);
  // spw->SetBounds(bounds);
  // spw->SetVariables(Vector2d(0.00000000001, 0));

  // nlp.AddVariableSet  (spw);
  // //nlp.AddVariableSet (std::make_shared<ExVariables>());
  // nlp.AddConstraintSet(std::make_shared<ExConstraint>());
  // nlp.AddCostSet      (std::make_shared<ExCost>());
  // nlp.PrintCurrent();

  // // 2. choose solver and options
  // IpoptSolver ipopt;
  // ipopt.SetOption("linear_solver", "ma27");
  // ipopt.SetOption("max_iter", 10000000);
  // ipopt.SetOption("print_level", 9);
  // // ipopt.SetOption("mumps_pivtolmax", 0.9);
  // // ipopt.SetOption("mumps_mem_percent", 1000000);

  // //ipopt.SetOption("jacobian_approximation", "exact");
  // ipopt.SetOption("jacobian_approximation", "finite-difference-values");
  
  // // 3 . solve
  // ipopt.Solve(nlp);
  // Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
  // std::cout << x.transpose() << std::endl;

  // // 4. test if solution correct
  // double eps = 1e-5; //double precision
  // assert(1.0-eps < x(0) && x(0) < 1.0+eps);
  // assert(0.0-eps < x(1) && x(1) < 0.0+eps); 

  return 0;
}