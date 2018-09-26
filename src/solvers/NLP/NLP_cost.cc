#include <hpp/foot/solvers/NLP/NLP_cost.hh>
using namespace std;
namespace hpp{
    namespace foot{
        double NLPCosts::GetCost() const 
        {
            double obj = 0.0;
            VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
            obj = Problem_->evalObj(x);
            return obj;
        }
        void NLPCosts::FillJacobianBlock (std::string var_set, Jacobian& jac) const 
        {
            double scaling =0.1;
            VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
            Eigen::MatrixXd out(1, x.size());
            Problem_->evalObjDiff(out, x);
            for (int i=0; i <stepsize_*3; i++)
                jac.coeffRef(0, i) = out(0, i);
        }
        void NLPCosts::SetInitState(const Vector3d& x_init){
            x_init_ = x_init;
        }
        void NLPCosts::SetStepSize(const int& stepsize){
            stepsize_ = stepsize;
        }

    }
}