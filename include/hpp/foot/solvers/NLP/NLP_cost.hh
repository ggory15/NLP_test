#pragma once

#include <ifopt/cost_term.h>
#include <hpp/foot/utils/defs.hh>

using VectorXd = Eigen::VectorXd;
using Vector3d = Eigen::Vector3d;
using namespace ifopt;
namespace hpp{
    namespace foot{
        class NLPCosts : public CostTerm {
        public:
            NLPCosts() : NLPCosts("cost_term1"){};
            NLPCosts(const std::string& name) : CostTerm(name){};
            double GetCost() const override;
            void FillJacobianBlock (std::string var_set, Jacobian& jac) const override;
            void SetInitState(const Vector3d& x_init);
            void SetStepSize(const int& stepsize);
        private:
            Vector3d x_init_;
            int stepsize_;
        };
    } // foot
}

