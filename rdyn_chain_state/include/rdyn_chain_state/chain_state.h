#ifndef RDYN_UTILITIES__CHAIN_STATE__H
#define RDYN_UTILITIES__CHAIN_STATE__H

#include <memory>
#include <state_space_filters/filtered_values.h>
#include <rdyn_core/primitives.h>
#include <rdyn_chain_state/chain_state_n.h>

namespace rdyn
{


/**
 * @class ChainState
 *
 * The class does not own the pointer to ChainInterface, but only a raw pointer is used.
 * This allow the usage also in the stack as stati object.
 */
class ChainState
{
private:
  ChainStateN<-1,rdyn::max_num_axes> impl_;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<ChainState> Ptr;
  typedef std::shared_ptr<ChainState const> ConstPtr;

  using Value = typename eigen_control_toolbox::FilteredValue<-1,rdyn::max_num_axes>::Value;
  using JacobianMatrix = Eigen::Matrix<double,6,-1, Eigen::ColMajor,6, rdyn::max_num_axes>;

  // GETTER
  const Value& q() const;
  const Value& qd() const;
  const Value& qdd() const;
  const Value& effort() const;
  const Value& external_effort() const;

  double q(const int& iAx) const;
  double qd(const int& iAx) const;
  double qdd(const int& iAx) const;
  double effort(const int& iAx) const;
  double external_effort(const int& iAx) const;

  double q(const std::string& name) const;
  double qd(const std::string& name) const;
  double qdd(const std::string& name) const;
  double effort(const std::string& name) const;
  double external_effort(const std::string& name) const;

  const rdyn::VectorOfAffine3d& linkPose( ) const;
  const rdyn::VectorOfVector6d& linkTwist( ) const;
  const rdyn::VectorOfVector6d& linkTwistd( ) const;
  
  const Eigen::Affine3d& toolPose( ) const;
  const Eigen::Vector6d& toolTwist( ) const;
  const Eigen::Vector6d& toolTwistd( ) const;
  const Eigen::Vector6d& toolWrench( ) const;
  const JacobianMatrix&  toolJacobian( ) const;

  // SETTER
  Value& q();
  Value& qd();
  Value& qdd();
  Value& effort();
  Value& external_effort();
  Eigen::Vector6d& wrench( );

  double& q(const int& iAx);
  double& qd(const int& iAx);
  double& qdd(const int& iAx);
  double& effort(const int& iAx);
  double& external_effort(const int& iAx);

  double& q(const std::string& name);
  double& qd(const std::string& name);
  double& qdd(const std::string& name);
  double& effort(const std::string& name);
  double& external_effort(const std::string& name);

  eigen_control_toolbox::FilteredValue<-1,rdyn::max_num_axes>& qFilteredValue();
  eigen_control_toolbox::FilteredValue<-1,rdyn::max_num_axes>& qdFilteredValue();
  eigen_control_toolbox::FilteredValue<-1,rdyn::max_num_axes>& qddFilteredValue();
  eigen_control_toolbox::FilteredValue<-1,rdyn::max_num_axes>& effortFilteredValue();
  eigen_control_toolbox::FilteredValue<-1,rdyn::max_num_axes>& externalEffortFilteredValue();
  eigen_control_toolbox::FilteredValue<6>&      wrenchFilteredValue();

  // METHODS
  enum KinematicsType
  {
    ZERO_ORDER   = 0b00000001, // ONLY POSITION FFWD
    FIRST_ORDER  = 0b00000011, // POS AND VEL FFWD
    SECOND_ORDER = 0b00000111, // POS, VEL, ACC FFWD
    FFWD_STATIC  = 0b00001000, // from EFFORT to wrench
    INV_STATIC   = 0b00010000  // from wrench to effort
  };

  void updateTransformations(ChainPtr kin, int ffwd_kin_type);
  void updateTransformations(Chain& kin, int ffwd_kin_type);

  double* handle_to_q(const int& iAx=0);
  double* handle_to_qd(const int& iAx=0);
  double* handle_to_qdd(const int& iAx=0);
  double* handle_to_effort(const int& iAx=0);
  double* handle_to_external_effort(const int& iAx=0);

  double* handle_to_q(const std::string& name);
  double* handle_to_qd(const std::string& name);
  double* handle_to_qdd(const std::string& name);
  double* handle_to_effort(const std::string& name);
  double* handle_to_external_effort(const std::string& name);

  ChainState() = default;
  ChainState(ChainPtr chain);
  ChainState(Chain&   chain);
  ChainState(const ChainState& cpy) = delete;
  ChainState(ChainState&& cpy) = delete;
  virtual ChainState& operator=(const ChainState& rhs) = delete;
  virtual ChainState& operator=(ChainState&& rhs) = delete;
  virtual ~ChainState() = default;

  virtual bool init(ChainPtr kin);
  virtual bool init(Chain& kin);
  void setZero(ChainPtr kin);
  void setZero(Chain& kin);

  enum CopyType { ONLY_JOINT, ONLY_CART, FULL_STATE };
  void copy(const ChainState& cpy, CopyType what);

  const std::vector<std::string> getJointNames() const;
  int nAx() const;
};

}

namespace std
{
std::string to_string(const rdyn::ChainState& chain);
}


#endif  // RDYN_UTILITIES__CHAIN_STATE__H
