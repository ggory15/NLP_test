#include <hpp/foot/solvers/LPQPsolver.hh>
#include <hpp/foot/utils/defs.hh>

using namespace qpOASES;
using namespace std;

namespace hpp{
namespace foot
{
LPQPSolver::LPQPSolver(const TrajectoryProblem& pb,
                                     const size_t& maxIter,
                                     const double& prec)
    : pb_(pb),
      qpPlanesFixed_(pb_),
      qpBoxesFixed_(pb_),
      qpBoxesFixedIndividual_(pb_),
      res_(pb_.dimVar()),
      maxIter_(maxIter),
      precision_(prec)
{
  resHistory_.resize(maxIter_ + 1);
  for (size_t i = 0; i < resHistory_.size(); i++)
  {
    resHistory_[i].resize(pb_.dimVar());
    resHistory_[i].setZero();
  }
  m_init_QP= false;
  m_init_LP = false;
  m_init_LPInd =false;
}

LPQPSolver::~LPQPSolver() {}

void LPQPSolver::init(const Eigen::VectorXd& xInit)
{
  res_ << xInit;
  resHistory_[0] << res_;
  qpPlanesFixed_.formQP(xInit.tail(pb_.dimPlans()));
  qpBoxesFixed_.formQP(xInit.head(pb_.dimBoxes()), xInit.tail(pb_.dimPlans()));
  QPSolver_ = new SQProblem(qpPlanesFixed_.dimVar(), qpPlanesFixed_.dimCstr()); //skim check, constraint, variable)
  LPSolver_ = new SQProblem(qpBoxesFixed_.dimVar(), qpBoxesFixed_.dimCstr());
  LPSolverIndiv_ = new SQProblem(5, 25, HST_ZERO);
  //LPSolverIndiv_ = QProblem(25, 5, HST_ZERO);
  m_options.setToDefault();
  m_options.printLevel = PL_LOW;
  m_options.enableEqualities = BT_TRUE;

  QPSolver_->setOptions(m_options);
  LPSolver_->setOptions(m_options);
  LPSolverIndiv_->setOptions(m_options);

//   QPSolver_.resize(qpPlanesFixed_.dimVar(), qpPlanesFixed_.dimCstr(),
//                    Eigen::lssol::eType::QP2);
//   LPSolver_.resize(qpBoxesFixed_.dimVar(), qpBoxesFixed_.dimCstr());
//   LPSolverIndiv_.resize(5, 25);
}

void LPQPSolver::formAndSolveQPPlanesFixed(RefVec x)
{
    qpPlanesFixed_.formQP(x.tail(pb_.dimPlans()));
    qpPlanesFixed_.updatePlanD(x.tail(pb_.dimPlans()));
    Eigen::MatrixXd Acopy(qpPlanesFixed_.A());
    MatrixRowMajorXd A_row = Acopy;
    MatrixRowMajorXd C_row = qpPlanesFixed_.C();

    int_t iter = 100;
    m_status_QP = QPSolver_->init(A_row.data(), qpPlanesFixed_.c().data(), C_row.data(), qpPlanesFixed_.lVar().data(), qpPlanesFixed_.uVar().data(), qpPlanesFixed_.l().data(), qpPlanesFixed_.u().data(), iter);
    x_sol_.resize(A_row.rows());
    x_sol_.setZero();
    
    QPSolver_->getPrimalSolution(x_sol_.data());
    x.head(pb_.dimBoxes()) << x_sol_.head(pb_.dimBoxes());
    QPSolver_->reset();

    // qpPlanesFixed_.printinTerminal();
    
//   QPSolver_.solve(qpPlanesFixed_.lVar(), qpPlanesFixed_.uVar(), Acopy,
//                   qpPlanesFixed_.c(), qpPlanesFixed_.C(), qpPlanesFixed_.l(),
//                   qpPlanesFixed_.u());
//   if (!(QPSolver_.inform() == 0 || QPSolver_.inform() == 1))
//   {
//     QPSolver_.print_inform();
//     std::cerr << "QP solver FAILED!!! Damnit" << std::endl;
//   }
//   // std::cout << "QPSolver_.result(): \n" << QPSolver_.result().transpose()
//   //<< std::endl;
//   x.head(pb_.dimBoxes()) << QPSolver_.result().head(pb_.dimBoxes());
}

void LPQPSolver::formAndSolveLPBoxesFixed(RefVec x)
{
//   qpBoxesFixed_.formQP(x.head(pb_.dimBoxes()), x.tail(pb_.dimPlans()));
//   LPSolver_.solve(qpBoxesFixed_.lVar(), qpBoxesFixed_.uVar(), qpBoxesFixed_.c(),
//                   qpBoxesFixed_.C(), qpBoxesFixed_.l(), qpBoxesFixed_.u());
//   if (!(LPSolver_.inform() == 0 || LPSolver_.inform() == 1))
//   {
//     LPSolver_.print_inform();
//     std::cerr << "LP solver FAILED!!! Damnit" << std::endl;
//   }
  //std::cout << "LPSolver_.result(): \n"
            //<< LPSolver_.result().transpose().format(fmt::custom) << std::endl;
  //x.tail(pb_.dimPlans()) << LPSolver_.result().head(pb_.dimPlans());
}

void LPQPSolver::formAndSolveIndividualLPBoxesFixed(RefVec x)
{
  for (size_t iPlan = 0; iPlan < pb_.nMobilePlanCstr(); ++iPlan)
  {
    int_t iter = 100;
    qpBoxesFixedIndividual_.formQP(iPlan, x.head(pb_.dimBoxes()),
                                   x.tail(pb_.dimPlans()));
    MatrixRowMajorXd C_row = qpBoxesFixedIndividual_.C();
    LPSolverIndiv_->init(0, qpBoxesFixedIndividual_.c().data(), C_row.data(), qpBoxesFixedIndividual_.lVar().data(), qpBoxesFixedIndividual_.uVar().data(), 
                    qpBoxesFixedIndividual_.l().data(), qpBoxesFixedIndividual_.u().data(), iter);
    
    x_sol_.resize(C_row.cols());
    x_sol_.setZero();
    
    LPSolverIndiv_->getPrimalSolution(x_sol_.data());
    x.segment(pb_.dimBoxes() + 4 * iPlan, 4) << x_sol_.head(4);
    LPSolverIndiv_->reset();
  }
}

void LPQPSolver::solve()
{
  Eigen::VectorXd prevRes(res_.rows());
  prevRes.setZero();
  bool converged = false;

  int nIter = 1;
  while (nIter < maxIter_ && !converged)
  {
    formAndSolveIndividualLPBoxesFixed(res_);
    pb_.normalizeNormals(res_);
    resHistory_[nIter] << res_;
    nIter++;

    formAndSolveQPPlanesFixed(res_);
    resHistory_[nIter] << res_;

    if((prevRes - res_).head(pb_.dimBoxes()).norm() < precision_)
    {
      converged = true;
      std::cout << "LPQP CONVERGED on iteration " << nIter << std::endl;
    }
    prevRes = res_;

    nIter++;
  }
  totalIter_ = nIter;
}

void LPQPSolver::logAllX(const std::string& folderName) const
{
  std::ofstream xLogFile;
  xLogFile.open(folderName + "xLog.m");
 
  for (size_t i = 0; i < totalIter_; i++)
  {
    // xLogFile << "%============== iteration " << i
    //          << "==================" << std::endl;
    // xLogFile << "x_" << i << " = ";
    //xLogFile << resHistory_[i].format(fmt::matlabVector) << std::endl;
    xLogFile << resHistory_[i].transpose().format(fmt::CommaInitFmt) << std::endl;
  }
  xLogFile.close();
}

} /* feettrajectory */
}
