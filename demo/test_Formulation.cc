#include <iostream>
// #include <cstdlib>
// #include <ctime>

#include <hpp/foot/solvers/NLP/NLP_formulation.hh>
#include <hpp/foot/solvers/NLP/NLP_variable.hh>
#include <ifopt/ipopt_solver.h>

//#include <hpp/foot/solvers/QP.hh>
#include "ifopt/test_vars_constr_cost.h"

using namespace ifopt;
using namespace std;
using namespace hpp::foot;


int main(int argc, char* argv[]){
    
    Eigen::VectorXd a; 
    ifopt::Problem nlp;

    //nlp.AddVariableSet(formulation.GetVariableSets());
    nlp.AddVariableSet(std::make_shared<ExVariables>());
    nlp.AddConstraintSet(std::make_shared<ExConstraint>());
    nlp.AddCostSet      (std::make_shared<ExCost>());
    nlp.PrintCurrent();

    auto solver = std::make_shared<ifopt::IpoptSolver>();
    solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
    solver->SetOption("limited_memory_initialization", "constant");
    solver->SetOption("max_cpu_time", 20.0);
    solver->SetOption("mu_strategy", "adaptive");
    //solver->SetOption("print_level", 8);
  
    solver->Solve(nlp);
    

    Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
    std::cout << x.transpose() << std::endl;  

    return 0;
}