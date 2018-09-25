#pragma once

#include <iostream>
#include <fstream>

// #include <eigenqp/LSSOL.h>
// #include <eigenqp/LSSOL_QP.h>
// #include <eigenqp/LSSOL_LP.h>

#include <qpOASES.hpp>

#include <hpp/foot/TrajectoryProblem.hh> // done
#include <hpp/foot/utils/defs.hh> // done
#include <hpp/foot/solvers/QP.hh>
#include <hpp/foot/solvers/QPPlanesFixed.hh> //done
#include <hpp/foot/solvers/QPBoxesFixed.hh> // done
#include <hpp/foot/solvers/QPBoxesFixedIndividual.hh> //done  

namespace hpp{
    namespace foot
    {
    class LPQPSolver
    {
    public:
    LPQPSolver(const TrajectoryProblem& pb, const size_t& maxIter, const double& precision = 1e-8);
    virtual ~LPQPSolver();
    void init(const Eigen::VectorXd& xInit);
    void solve();
    void logAllX(const std::string& fileName) const;
    const QPPlanesFixed& qpPlanesFixed() const { return qpPlanesFixed_; }
    const QPBoxesFixed& qpBoxesFixed() const { return qpBoxesFixed_; }
    void formAndSolveQPPlanesFixed(RefVec x);
    void formAndSolveLPBoxesFixed(RefVec x);
    void formAndSolveIndividualLPBoxesFixed(RefVec x);
    Eigen::VectorXd res() const { return res_; }
    const size_t& totalIter() const { return totalIter_; }

    private:
    const TrajectoryProblem& pb_;
    QPPlanesFixed qpPlanesFixed_;
    QPBoxesFixed qpBoxesFixed_;
    QPBoxesFixedIndividual qpBoxesFixedIndividual_;
    Eigen::VectorXd res_;
    size_t maxIter_, totalIter_;
    std::vector<Eigen::VectorXd> resHistory_;
    double precision_;

    /// @brief QP solver
    qpOASES::SQProblem* QPSolver_;
    qpOASES::SQProblem* LPSolver_;
    qpOASES::SQProblem* LPSolverIndiv_;

    bool m_init_QP, m_init_LP, m_init_LPInd;
    qpOASES::returnValue m_status_QP, m_status_LP, m_status_LPInd;
    qpOASES::Options m_options;
    Eigen::VectorXd x_sol_;

    };
    } /* feettrajectory */
}