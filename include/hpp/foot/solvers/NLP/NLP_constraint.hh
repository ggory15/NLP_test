#pragma once

#include <ifopt/constraint_set.h>
#include <hpp/foot/utils/defs.hh>
#include <hpp/foot/BoxesHullTrajProblem.hh>

using VectorXd = Eigen::VectorXd;
using Vector3d = Eigen::Vector3d;
using VecBound = ifopt::VariableSet::VecBound;
using namespace ifopt;
using namespace std;
namespace hpp{
    namespace foot{
        class NLPConstraints : public ConstraintSet {
        public:
            NLPConstraints(BoxesHullTrajProblem* Problem, const Index& i, const std::string& name) : NLPConstraints(name, Problem->nonLinCstrDim(i)){
                Problem_ = Problem;
                index_  = i;
            };
            NLPConstraints(const std::string& name, const Index& size) : ConstraintSet(size, name){};
            VectorXd GetValues() const override;
            VecBound GetBounds() const override;
            void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override;
        private:
            BoxesHullTrajProblem* Problem_;
            Index index_;
        };
    } // foot
}