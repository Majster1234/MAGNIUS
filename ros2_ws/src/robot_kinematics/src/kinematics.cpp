#include "robot_kinematics/kinematics.hpp"
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <Eigen/Cholesky>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <openssl/sha.h>
#include <urdf/model.h>

namespace robot_kinematics {
namespace {
KDL::JntArray joints(const Joints &q) {
  KDL::JntArray result(6);
  for (unsigned i = 0; i < 6; ++i) {
    if (!std::isfinite(q[i])) throw std::invalid_argument("Nieznana lub nieskończona pozycja osi");
    result(i) = q[i];
  }
  return result;
}
Eigen::Matrix<double, 6, 1> error(const Pose &current, const Pose &target) {
  Eigen::Matrix<double, 6, 1> e;
  e.head<3>() = target.translation() - current.translation();
  const Eigen::AngleAxisd rotation(target.rotation() * current.rotation().transpose());
  e.tail<3>() = rotation.axis() * rotation.angle();
  return e;
}
}
std::string model_id(const std::string &xml) {
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(xml.data()), xml.size(), hash);
  std::ostringstream stream;
  for (auto byte : hash) stream << std::hex << std::setw(2) << std::setfill('0') << unsigned(byte);
  return stream.str();
}
Pose interpolate(const Pose &a, const Pose &b, double t) {
  Pose result = Pose::Identity();
  result.translation() = (1-t)*a.translation() + t*b.translation();
  result.linear() = Eigen::Quaterniond(a.rotation()).slerp(t, Eigen::Quaterniond(b.rotation())).toRotationMatrix();
  return result;
}
Model::Model(const std::string &xml, const Joints &lower, const Joints &upper) : lower_(lower), upper_(upper) {
  if (xml.empty() || xml.size() > 262144) throw std::invalid_argument("Brak modelu lub zbyt duży URDF");
  urdf::Model parsed;
  if (!parsed.initString(xml)) throw std::invalid_argument("Nieprawidłowy URDF");
  const auto tool = parsed.getJoint("tool_fixed");
  if (!tool || tool->type != urdf::Joint::FIXED) throw std::invalid_argument("Model wymaga stałego joint tool_fixed opisującego TCP");
  base_ = parsed.getRoot()->name; tip_ = tool->child_link_name;
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(parsed, tree) || !tree.getChain(base_, tip_, chain_) || chain_.getNrOfJoints() != 6)
    throw std::invalid_argument("Wymagany łańcuch sześciu osi od bazy do TCP");
  unsigned index = 0;
  for (const auto &segment : chain_.segments) {
    if (segment.getJoint().getType() == KDL::Joint::None) continue;
    const auto name = "joint_" + std::to_string(++index);
    const auto joint = parsed.getJoint(name);
    if (segment.getJoint().getName() != name || !joint || joint->mimic ||
        (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::CONTINUOUS))
      throw std::invalid_argument("Kolejność modelu musi odpowiadać joint_1–joint_6, bez mimic");
  }
  for (unsigned i=0; i<6; ++i) if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i] >= upper[i])
    throw std::invalid_argument("Nieprawidłowe limity przegubu");
}
Pose Model::fk(const Joints &q) const {
  KDL::ChainFkSolverPos_recursive solver(chain_); KDL::Frame frame;
  if (solver.JntToCart(joints(q), frame) < 0) throw std::runtime_error("Błąd FK");
  Pose result = Pose::Identity();
  for (unsigned i=0; i<3; ++i) { result.translation()[i]=frame.p[i]; for(unsigned j=0;j<3;++j) result.linear()(i,j)=frame.M(i,j); }
  return result;
}
Eigen::Matrix<double,6,6> Model::jacobian(const Joints &q) const {
  KDL::ChainJntToJacSolver solver(chain_); KDL::Jacobian jac(6);
  if (solver.JntToJac(joints(q), jac) < 0) throw std::runtime_error("Błąd Jacobianu");
  return jac.data;
}
Solution Model::solve(const Pose &target, const Joints &seed, const Mask &locked, double p_tol, double r_tol,
                      const std::function<bool()> &cancelled) const {
  if (!target.matrix().allFinite() || !target.linear().isUnitary(1e-6) || target.linear().determinant()<0 ||
      !std::isfinite(p_tol) || p_tol<=0 || p_tol>.01 || !std::isfinite(r_tol) || r_tol<=0 || r_tol>.1)
    throw std::invalid_argument("Nieprawidłowy cel lub tolerancje IK");
  joints(seed);
  Solution result; result.positions=seed;
  for(unsigned i=0;i<6;++i) if(seed[i]<lower_[i] || seed[i]>upper_[i]) throw std::invalid_argument("Pozycja początkowa poza limitami");
  for(unsigned iteration=0;iteration<300;++iteration) {
    if(cancelled && cancelled()) { result.message="Obliczenia anulowane lub model zmieniony"; return result; }
    const auto e=error(fk(result.positions),target);
    result.position_error=e.head<3>().norm(); result.orientation_error=e.tail<3>().norm();
    if(result.position_error<=p_tol && result.orientation_error<=r_tol) { result.success=true; result.message="IK rozwiązane"; return result; }
    auto jac=jacobian(result.positions);
    for(unsigned i=0;i<6;++i) if(locked[i]) jac.col(i).setZero();
    // Metry i radiany w jednym błędzie: ramię skalowania 0,2 m.
    auto weighted=e; weighted.tail<3>()*=.2; jac.bottomRows<3>()*=.2;
    const auto normal=jac*jac.transpose()+1e-6*Eigen::Matrix<double,6,6>::Identity();
    Eigen::Matrix<double,6,1> delta=jac.transpose()*normal.ldlt().solve(weighted);
    if(!delta.allFinite()) break;
    if(delta.norm()>.15) delta*=.15/delta.norm();
    bool advanced=false;
    for(double scale : {1.,.5,.25,.1,.02}) {
      auto candidate=result.positions;
      for(unsigned i=0;i<6;++i) if(!locked[i]) candidate[i]=std::clamp(candidate[i]+scale*delta[i],lower_[i],upper_[i]);
      auto candidate_error=error(fk(candidate),target); candidate_error.tail<3>()*=.2;
      if(candidate_error.squaredNorm()<weighted.squaredNorm()) { result.positions=candidate; advanced=true; break; }
    }
    if(!advanced) break;
  }
  const auto e=error(fk(result.positions),target); result.position_error=e.head<3>().norm(); result.orientation_error=e.tail<3>().norm();
  result.message="Brak rozwiązania pełnej pozy TCP: granice, osobliwość lub zamrożone osie"; return result;
}
}
