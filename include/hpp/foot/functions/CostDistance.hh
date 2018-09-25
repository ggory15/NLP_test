#pragma once
#include <iostream>
#include <Eigen/Core>
#include <hpp/foot/utils/Box.hh>
#include <hpp/foot/utils/defs.hh>

namespace hpp{
    namespace foot
    {
    class CostDistance
    {
    public:
        CostDistance (const long& nMobileBoxes, ConstRefVec3d initPos, ConstRefVec3d finalPos);
        virtual ~CostDistance ();
        double compute(ConstRefVec x);

        const Eigen::MatrixXd& Q() const { return Q_; }
        const Eigen::VectorXd& c() const { return c_; }

        void fillQuadCost(RefMat Q, RefVec c) const;

    private:
        long n_;
        Eigen::MatrixXd Q_;
        Eigen::VectorXd c_;
    };
    } /* feettrajectory */ 
}