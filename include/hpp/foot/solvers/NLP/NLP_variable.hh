#pragma once

#include <ifopt/variable_set.h>
#include <hpp/foot/utils/defs.hh>

using VectorXd = Eigen::VectorXd;
using VecBound = ifopt::VariableSet::VecBound;
using namespace ifopt;
namespace hpp{
    namespace foot{
        class NLPVariables : public VariableSet {
        public:
            NLPVariables(const std::string& name);
            NLPVariables(const std::string& name, const int& size);
            virtual ~NLPVariables() = default;

            void SetVariables(const VectorXd& x) override;
            VectorXd GetValues() const override;
            VecBound GetBounds() const override;
            void SetBounds(const VecBound& bound);
            void SetInitVariable(const VectorXd& x_init);
        private:             
            VectorXd x_;
            VecBound bounds_;
        };
    } // foot
}