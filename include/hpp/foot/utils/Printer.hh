#pragma once
#include <iostream>
#include <fstream>
#include <string>

#include <hpp/foot/utils/defs.hh>
#include <hpp/foot/TrajectoryProblem.hh>

namespace hpp{
namespace foot
{
//void print(const std::string& fileName, const BoxTrajProblem& pb,
//           const mnf::Point& xStar);
//void print(const std::string& fileName, const BoxesHullTrajProblem& pb,
//           const mnf::Point& xStar);
void print(const std::string& fileName, const Eigen::Vector3d& bSize,
           const Eigen::Vector3d& oSize, const Eigen::Vector3d& oPos,
           const Eigen::Vector3d& t, const double& d, const Eigen::Vector3d& n);
//void printAllIterations(const std::string& fileName,
//                        const BoxesHullTrajProblem& pb, const mnf::Point& xStar,
//                        const std::string& folder);
void printAllIterations(const std::string& fileName,
                        const TrajectoryProblem& pb,
                        const Eigen::VectorXd& xStar,
                        const std::string& folder);
std::vector<Eigen::VectorXd> parseX(const std::string& file,
                                    const Index& x);
} /* feettrajectory */
}
