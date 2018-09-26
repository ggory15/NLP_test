#include <hpp/foot/solvers/NLP/NLP_constraint.hh>	
#include <Eigen/SparseCore>

using namespace std;

namespace hpp{
    namespace foot{
        VectorXd NLPConstraints::GetValues() const 
        {
           VectorXd g(GetRows());
           VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
           VectorXd out;
           out.resize(Problem_->nonLinCstrDim(index_));
           Problem_->evalNonLinCstr(out, x, index_);
           g = out;
           return g;
        }
        VecBound NLPConstraints::GetBounds() const 
        {
          VecBound b(GetRows());
          VectorXd lb, ub;
          lb.resize(Problem_->nonLinCstrDim(index_));
          ub.resize(Problem_->nonLinCstrDim(index_));

          Problem_->getNonLinCstrLB(lb, index_);
          Problem_->getNonLinCstrUB(ub, index_);
          for (int i=0; i<Problem_->nonLinCstrDim(index_); i++)
            b.at(i) = Bounds(lb(i), ub(i));

          return b;
        }
        void NLPConstraints::FillJacobianBlock (std::string var_set, Jacobian& jac_block) const
        {
            if (var_set == "var_set1") {
                VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
                Eigen::SparseMatrix<double> out;
                Problem_->evalNonLinCstrDiff(out, x, index_);
                for (int i=0; i< out.rows(); i++)
                    jac_block.row(i) = out.row(i); 
            }
        }
    }
}