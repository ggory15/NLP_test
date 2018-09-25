#include <hpp/foot/solvers/NLP/NLP_cost.hh>

namespace hpp{
    namespace foot{
        double NLPCosts::GetCost() const 
        {
            double obj = 0.0;
            VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
            Vector3d pos_next = x.head(3);
            Vector3d pos = x_init_;
            
            obj += pow( (pos_next-pos).norm(), 2);
            for (int i=0; i< stepsize_; i++){
                pos_next = x.segment(3*i+3, 3);
                pos = x.segment(3*i, 3);
                obj += pow( (pos_next-pos).norm(), 2);
            }

            return obj;
        }
        void NLPCosts::FillJacobianBlock (std::string var_set, Jacobian& jac) const 
        {
            double scaling =0.1;
            VectorXd x = GetVariables()->GetComponent("var_set1")->GetValues();
            for (int i=0; i< 3; i++){
                jac.coeffRef(0, i) = scaling * (40*x(i) - 20* x_init_(i) - 20 * x(3 + i));
                //jac.coeffRef(0, 9 + i) = scaling*(20*x(9+i) - 20* x(i+6));//- 20 * x(12 + i);
            }
             for (int i=0; i< 3; i++)
                 for (int j=1; j<stepsize_-2; j++)
                     jac.coeffRef(0, 3*j + i) = scaling* (40*x(3*j+i) - 20* x(3*(j-1)+i) - 20 * x(3*(j+1) + i));
            for (int i=0; i<3; i++)
                 jac.coeffRef(0, (stepsize_-1)*3 +i) = scaling * (20 * x((stepsize_-1)*3 +i) - 20 * x((stepsize_-2)*3 +i));
        }
        void NLPCosts::SetInitState(const Vector3d& x_init){
            x_init_ = x_init;
        }
        void NLPCosts::SetStepSize(const int& stepsize){
            stepsize_ = stepsize;
        }

    }
}