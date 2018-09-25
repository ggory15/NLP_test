#include <hpp/foot/solvers/NLP/NLP_variable.hh>

namespace hpp{
    namespace foot{
        NLPVariables::NLPVariables(const std::string& name) : VariableSet(108, name)
        {
        }
        NLPVariables::NLPVariables(const std::string& name, const int& size) : VariableSet(size, name)
        {
            x_.resize(size);
            x_.setZero();
            bounds_.resize(size);            
        }
        void NLPVariables::SetVariables(const Eigen::VectorXd& x) {
            assert(x.size() == x_.size() && "wrong input variable size");
            x_ = x;
        }
        Eigen::VectorXd NLPVariables::GetValues() const
        {
            return x_;
        }
        VecBound NLPVariables::GetBounds() const
        {
            return bounds_;
        }
        void NLPVariables::SetBounds(const VecBound& bound) 
        {
          //  assert (bound.size() == bounds_.size() && " wrong input variable bound size");
            bounds_ = bound;
        }       
        void NLPVariables::SetInitVariable(const VectorXd& x_init)
        {
             assert(x_init.size() == x_.size() && "wrong init variable size");    
             x_ = x_init;
        }        
    } //foot
} // hpp
