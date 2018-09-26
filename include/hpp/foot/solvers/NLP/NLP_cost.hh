#pragma once

#include <ifopt/cost_term.h>
#include <hpp/foot/utils/defs.hh>
#include <hpp/foot/BoxesHullTrajProblem.hh>

using VectorXd = Eigen::VectorXd;
using Vector3d = Eigen::Vector3d;
using namespace ifopt;
namespace hpp{
    namespace foot{
        class NLPCosts : public CostTerm {
        public:
            NLPCosts(BoxesHullTrajProblem* Problem) : NLPCosts("cost_term1"){
                Problem_ = Problem;
            };
            NLPCosts(const std::string& name) : CostTerm(name){};
            double GetCost() const override;
            void FillJacobianBlock (std::string var_set, Jacobian& jac) const override;
            void SetInitState(const Vector3d& x_init);
            void SetStepSize(const int& stepsize);
        private:
            Vector3d x_init_;
            int stepsize_;
            BoxesHullTrajProblem* Problem_;

        };
    } // foot
}

