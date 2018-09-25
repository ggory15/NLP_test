#pragma once

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

namespace hpp {
    namespace foot{
        class NLPFormulation {
            public:
            using VectorXd = Eigen::VectorXd;
            using VariablePtrVec = ifopt::VariableSet::Ptr;
            using ContraintPtrVec = ifopt::ConstraintSet::Ptr;
            using CostPtrVec = ifopt::CostTerm::Ptr;

            NLPFormulation();
            virtual ~NLPFormulation() = default;
            VariablePtrVec GetVariableSets();
            ContraintPtrVec GetConstraints() const;
            CostPtrVec GetCosts() const;            
        };
    }
}