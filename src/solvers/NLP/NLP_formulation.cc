#include <hpp/foot/solvers/NLP/NLP_formulation.hh>
#include <hpp/foot/solvers/NLP/NLP_variable.hh>
#include <hpp/foot/solvers/NLP/NLP_cost.hh>
#include <iostream>

namespace hpp
{
    namespace foot{
        NLPFormulation::NLPFormulation ()
        {
        using namespace std;
            cout << "START NLP FORMULATION" << endl;
        }

        NLPFormulation::VariablePtrVec
        NLPFormulation::GetVariableSets ()
        {
            VariablePtrVec vars = std::make_shared<NLPVariables>("var_set1");
            return vars;
        }
        NLPFormulation::ContraintPtrVec
        NLPFormulation::GetConstraints() const
        {
            ContraintPtrVec constraints;

            return constraints;
        }
        NLPFormulation::CostPtrVec
        NLPFormulation::GetCosts() const
        {
            CostPtrVec costs = std::make_shared<NLPCosts>("cost_term1");
            return costs;
        }

    } //foot
} //hpp