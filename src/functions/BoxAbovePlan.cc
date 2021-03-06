#include <hpp/foot/functions/BoxAbovePlan.hh>
//#include <feet-trajectory/utils/quat2mat.hh>
#include <Eigen/Geometry> 

namespace hpp{
namespace foot
{
BoxAbovePlan::BoxAbovePlan(const Box& c) : box_(c)
{
  std::stringstream ss;
  ss << "Box " << std::to_string(box_.index()) << " above plan";
  name_ = ss.str();
}

BoxAbovePlan::~BoxAbovePlan() {}

void BoxAbovePlan::compute(Eigen::Ref<Eigen::Matrix<double, 8, 1>> res,
                           const Eigen::Ref<const Eigen::Vector3d> t,
                           const Eigen::Ref<const Eigen::Vector4d> q,
                           const double& d,
                           const Eigen::Ref<const Eigen::Vector3d> n) const
{
  Eigen::Quaterniond q2(q(3), q(0), q(1), q(2)); //skim:: check the order of quarternion.

  for (size_t i = 0; i < 8; i++)
    res[static_cast<long>(i)] = (t + q2.toRotationMatrix() * box_.vertex(i)).dot(n) - d;
}

void BoxAbovePlan::diffTrans(Eigen::Ref<Eigen::Matrix<double, 8, 3>> res,
                             const Eigen::Ref<const Eigen::Vector3d>,
                             const Eigen::Ref<const Eigen::Vector4d>,
                             const double&,
                             const Eigen::Ref<const Eigen::Vector3d> n) const
{
  for (long i = 0; i < 8; i++)
  {
    res(i, 0) = n.x();
    res(i, 1) = n.y();
    res(i, 2) = n.z();
  }
}
void BoxAbovePlan::diffTrans(Eigen::Ref<Eigen::Matrix<double, 1, 3>> res,
                             const Eigen::Ref<const Eigen::Vector3d>,
                             const Eigen::Ref<const Eigen::Vector4d>,
                             const double&,
                             const Eigen::Ref<const Eigen::Vector3d> n,
                             const long&) const
{
  res(0, 0) = n.x();
  res(0, 1) = n.y();
  res(0, 2) = n.z();
}

void BoxAbovePlan::diffQuat(Eigen::Ref<Eigen::Matrix<double, 8, 4>> res,
                            const Eigen::Ref<const Eigen::Vector3d>,
                            const Eigen::Ref<const Eigen::Vector4d> q,
                            const double&,
                            const Eigen::Ref<const Eigen::Vector3d> n) const
{
  for (long i = 0; i < 8; i++)
  {
    Eigen::Vector3d v = box_.vertex(static_cast<size_t>(i));
    res(i, 0) =
        n.z() * (2 * q.w() * v.y() - 4 * q.x() * v.z() + 2 * q.z() * v.x()) -
        n.y() * (2 * q.w() * v.z() + 4 * q.x() * v.y() - 2 * q.y() * v.x()) +
        n.x() * (2 * q.y() * v.y() + 2 * q.z() * v.z());
    res(i, 1) =
        n.x() * (2 * q.w() * v.z() + 2 * q.x() * v.y() - 4 * q.y() * v.x()) -
        n.z() * (2 * q.w() * v.x() + 4 * q.y() * v.z() - 2 * q.z() * v.y()) +
        n.y() * (2 * q.x() * v.x() + 2 * q.z() * v.z());
    res(i, 2) =
        n.y() * (2 * q.w() * v.x() + 2 * q.y() * v.z() - 4 * q.z() * v.y()) -
        n.x() * (2 * q.w() * v.y() - 2 * q.x() * v.z() + 4 * q.z() * v.x()) +
        n.z() * (2 * q.x() * v.x() + 2 * q.y() * v.y());
    res(i, 3) = n.z() * (2 * q.x() * v.y() - 2 * q.y() * v.x()) -
                n.y() * (2 * q.x() * v.z() - 2 * q.z() * v.x()) +
                n.x() * (2 * q.y() * v.z() - 2 * q.z() * v.y());
  }
}
void BoxAbovePlan::diffQuat(Eigen::Ref<Eigen::Matrix<double, 1, 4>> res,
                            const Eigen::Ref<const Eigen::Vector3d>,
                            const Eigen::Ref<const Eigen::Vector4d> q,
                            const double&,
                            const Eigen::Ref<const Eigen::Vector3d> n,
                            const long& i) const
{
  Eigen::Vector3d v = box_.vertex(static_cast<size_t>(i));
  res(0, 0) =
      n.z() * (2 * q.w() * v.y() - 4 * q.x() * v.z() + 2 * q.z() * v.x()) -
      n.y() * (2 * q.w() * v.z() + 4 * q.x() * v.y() - 2 * q.y() * v.x()) +
      n.x() * (2 * q.y() * v.y() + 2 * q.z() * v.z());
  res(0, 1) =
      n.x() * (2 * q.w() * v.z() + 2 * q.x() * v.y() - 4 * q.y() * v.x()) -
      n.z() * (2 * q.w() * v.x() + 4 * q.y() * v.z() - 2 * q.z() * v.y()) +
      n.y() * (2 * q.x() * v.x() + 2 * q.z() * v.z());
  res(0, 2) =
      n.y() * (2 * q.w() * v.x() + 2 * q.y() * v.z() - 4 * q.z() * v.y()) -
      n.x() * (2 * q.w() * v.y() - 2 * q.x() * v.z() + 4 * q.z() * v.x()) +
      n.z() * (2 * q.x() * v.x() + 2 * q.y() * v.y());
  res(0, 3) = n.z() * (2 * q.x() * v.y() - 2 * q.y() * v.x()) -
              n.y() * (2 * q.x() * v.z() - 2 * q.z() * v.x()) +
              n.x() * (2 * q.y() * v.z() - 2 * q.z() * v.y());
}

void BoxAbovePlan::diffD(Eigen::Ref<Eigen::Matrix<double, 8, 1>> res,
                         const Eigen::Ref<const Eigen::Vector3d>,
                         const Eigen::Ref<const Eigen::Vector4d>, const double&,
                         const Eigen::Ref<const Eigen::Vector3d>) const
{
  for (long i = 0; i < 8; i++)
  {
    res(i, 0) = -1;
  }
}
void BoxAbovePlan::diffD(Eigen::Ref<Eigen::Matrix<double, 1, 1>> res,
                         const Eigen::Ref<const Eigen::Vector3d>,
                         const Eigen::Ref<const Eigen::Vector4d>, const double&,
                         const Eigen::Ref<const Eigen::Vector3d>,
                         const long&) const
{
  res(0, 0) = -1;
}

void BoxAbovePlan::diffNormal(Eigen::Ref<Eigen::Matrix<double, 8, 3>> res,
                              const Eigen::Ref<const Eigen::Vector3d> t,
                              const Eigen::Ref<const Eigen::Vector4d> q,
                              const double&,
                              const Eigen::Ref<const Eigen::Vector3d>) const
{
  for (long i = 0; i < 8; i++)
  {
    Eigen::Vector3d v = box_.vertex(static_cast<size_t>(i));
    res(i, 0) = t.x() - v.x() * (2 * q.y() * q.y() + 2 * q.z() * q.z() - 1) -
                v.y() * (2 * q.w() * q.z() - 2 * q.x() * q.y()) +
                v.z() * (2 * q.w() * q.y() + 2 * q.x() * q.z());
    res(i, 1) = t.y() - v.y() * (2 * q.x() * q.x() + 2 * q.z() * q.z() - 1) +
                v.x() * (2 * q.w() * q.z() + 2 * q.x() * q.y()) -
                v.z() * (2 * q.w() * q.x() - 2 * q.y() * q.z());
    res(i, 2) = t.z() - v.z() * (2 * q.x() * q.x() + 2 * q.y() * q.y() - 1) -
                v.x() * (2 * q.w() * q.y() - 2 * q.x() * q.z()) +
                v.y() * (2 * q.w() * q.x() + 2 * q.y() * q.z());
  }
}

void BoxAbovePlan::diffNormal(Eigen::Ref<Eigen::Matrix<double, 1, 3>> res,
                              const Eigen::Ref<const Eigen::Vector3d> t,
                              const Eigen::Ref<const Eigen::Vector4d> q,
                              const double&,
                              const Eigen::Ref<const Eigen::Vector3d>,
                              const long& i) const
{
  Eigen::Vector3d v = box_.vertex(static_cast<size_t>(i));
  res(0, 0) = t.x() - v.x() * (2 * q.y() * q.y() + 2 * q.z() * q.z() - 1) -
              v.y() * (2 * q.w() * q.z() - 2 * q.x() * q.y()) +
              v.z() * (2 * q.w() * q.y() + 2 * q.x() * q.z());
  res(0, 1) = t.y() - v.y() * (2 * q.x() * q.x() + 2 * q.z() * q.z() - 1) +
              v.x() * (2 * q.w() * q.z() + 2 * q.x() * q.y()) -
              v.z() * (2 * q.w() * q.x() - 2 * q.y() * q.z());
  res(0, 2) = t.z() - v.z() * (2 * q.x() * q.x() + 2 * q.y() * q.y() - 1) -
              v.x() * (2 * q.w() * q.y() - 2 * q.x() * q.z()) +
              v.y() * (2 * q.w() * q.x() + 2 * q.y() * q.z());
}

void BoxAbovePlan::LB(Eigen::Ref<Eigen::Matrix<double, 8, 1>> res,
                      const double& d) const
{
  res << d, d, d, d, d, d, d, d;
}

void BoxAbovePlan::UB(Eigen::Ref<Eigen::Matrix<double, 8, 1>> res) const
{
  double inf = std::numeric_limits<double>::infinity();
  res << inf, inf, inf, inf, inf, inf, inf, inf;
}

void BoxAbovePlan::fillLinCstrNFixed(double& lb,
                                     Eigen::Ref<Eigen::MatrixXd> CboxPart,
                                     double& CdPart, Eigen::Vector3d normal,
                                     bool isBoxBelow) const
{
  // Constraint is:
  // for all i in [1:8]
  // -vi.n <= n.b - d
  // Simplified into
  // max_i -vi.n <= n.b - d
  if (isBoxBelow) normal << -normal;

  lb = -std::numeric_limits<double>::infinity();
  double val;
  for (size_t i = 0; i < box().vertex().size(); i++)
  {
    if (!box().fixed())
      val = -box().vertex(i).dot(normal);
    else
      val = -(box().vertex(i) + box().center()).dot(normal);

    if (val > lb) lb = val;
  }
  if (isBoxBelow)
    CdPart = 1;
  else
    CdPart = -1;
  if (!box().fixed()) CboxPart << normal.transpose();
}

} /* feettrajectory */
}