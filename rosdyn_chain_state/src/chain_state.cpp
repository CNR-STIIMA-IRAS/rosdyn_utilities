#include <memory>
#include <state_space_filters/filtered_values.h>
#include <rosdyn_chain_state/chain_state.h>

namespace rosdyn
{

const ChainState::Value& ChainState::q() const { return impl_.q();  }
const ChainState::Value& ChainState::qd() const { return impl_.qd(); }
const ChainState::Value& ChainState::qdd() const { return impl_.qdd();}
const ChainState::Value& ChainState::effort() const { return impl_.effort(); }
const ChainState::Value& ChainState::external_effort() const { return impl_.external_effort(); }

double ChainState::q(const int& iAx) const {return impl_.q(iAx); }
double ChainState::qd(const int& iAx) const {return impl_.qd(iAx); }
double ChainState::qdd(const int& iAx) const {return impl_.qdd(iAx); }
double ChainState::effort(const int& iAx) const {return impl_.effort(iAx); }
double ChainState::external_effort(const int& iAx) const {return impl_.external_effort(iAx); }

double ChainState::q(const std::string& name) const {return impl_.q(name); }
double ChainState::qd(const std::string& name) const {return impl_.qd(name); }
double ChainState::qdd(const std::string& name) const {return impl_.qdd(name); }
double ChainState::effort(const std::string& name) const {return impl_.effort(name); }
double ChainState::external_effort(const std::string& name) const {return impl_.external_effort(name); }

const rosdyn::VectorOfAffine3d& ChainState::linkPose( ) const { return impl_.linkPose(); }
const rosdyn::VectorOfVector6d& ChainState::linkTwist( ) const { return impl_.linkTwist(); }
const rosdyn::VectorOfVector6d& ChainState::linkTwistd( ) const { return impl_.linkTwistd(); }

const Eigen::Affine3d& ChainState::toolPose( ) const { return impl_.toolPose();   }
const Eigen::Vector6d& ChainState::toolTwist( ) const { return impl_.toolTwist(); }
const Eigen::Vector6d& ChainState::toolTwistd( ) const { return impl_.toolTwistd();}
const Eigen::Vector6d& ChainState::toolWrench( ) const { return impl_.toolWrench();}
const ChainState::JacobianMatrix&  ChainState::toolJacobian( ) const { return impl_.toolJacobian(); }

// SETTER
ChainState::Value& ChainState::q() { return impl_.q();  }
ChainState::Value& ChainState::qd() { return impl_.qd(); }
ChainState::Value& ChainState::qdd() { return impl_.qdd();}
ChainState::Value& ChainState::effort() { return impl_.effort(); }
ChainState::Value& ChainState::external_effort() { return impl_.external_effort(); }
Eigen::Vector6d&   ChainState::wrench( ) { return impl_.wrench();}

double& ChainState::q(const int& iAx) { return impl_.q(iAx); }
double& ChainState::qd(const int& iAx) { return impl_.qd(iAx); }
double& ChainState::qdd(const int& iAx) { return impl_.qdd(iAx); }
double& ChainState::effort(const int& iAx) { return impl_.effort(iAx); }
double& ChainState::external_effort(const int& iAx) { return impl_.external_effort(iAx); }

double& ChainState::q(const std::string& name) {return impl_.q(name); }
double& ChainState::qd(const std::string& name) {return impl_.qd(name); }
double& ChainState::qdd(const std::string& name) {return impl_.qdd(name); }
double& ChainState::effort(const std::string& name) {return impl_.effort(name); }
double& ChainState::external_effort(const std::string& name) {return impl_.external_effort(name); }

eigen_control_toolbox::FilteredValue<-1,rdyn::max_num_axes>& ChainState::qFilteredValue() {return impl_.qFilteredValue();}
eigen_control_toolbox::FilteredValue<-1,rdyn::max_num_axes>& ChainState::qdFilteredValue() {return impl_.qdFilteredValue();}
eigen_control_toolbox::FilteredValue<-1,rdyn::max_num_axes>& ChainState::qddFilteredValue() {return impl_.qddFilteredValue();}
eigen_control_toolbox::FilteredValue<-1,rdyn::max_num_axes>& ChainState::effortFilteredValue() {return impl_.effortFilteredValue();}
eigen_control_toolbox::FilteredValue<-1,rdyn::max_num_axes>& ChainState::externalEffortFilteredValue() {return impl_.externalEffortFilteredValue();}
eigen_control_toolbox::FilteredValue<6>&      ChainState::wrenchFilteredValue() {return impl_.wrenchFilteredValue();}

void ChainState::updateTransformations(ChainPtr kin, int ffwd_kin_type) {impl_.updateTransformations(kin, ffwd_kin_type);}
void ChainState::updateTransformations(Chain& kin, int ffwd_kin_type) {impl_.updateTransformations(kin, ffwd_kin_type);}

double* ChainState::handle_to_q(const int& iAx) {return impl_.handle_to_q(iAx); }
double* ChainState::handle_to_qd(const int& iAx) {return impl_.handle_to_qd(iAx); }
double* ChainState::handle_to_qdd(const int& iAx) {return impl_.handle_to_qdd(iAx); }
double* ChainState::handle_to_effort(const int& iAx) {return impl_.handle_to_effort(iAx); }
double* ChainState::handle_to_external_effort(const int& iAx) {return impl_.handle_to_external_effort(iAx); }

double* ChainState::handle_to_q(const std::string& name) {return impl_.handle_to_q(name); }
double* ChainState::handle_to_qd(const std::string& name) {return impl_.handle_to_qd(name); }
double* ChainState::handle_to_qdd(const std::string& name) {return impl_.handle_to_qdd(name); }
double* ChainState::handle_to_effort(const std::string& name) {return impl_.handle_to_effort(name); }
double* ChainState::handle_to_external_effort(const std::string& name) {return impl_.handle_to_external_effort(name); }

bool ChainState::init(ChainPtr kin) {return impl_.init(kin);}
bool ChainState::init(Chain& kin) {return impl_.init(kin);}
void ChainState::setZero(ChainPtr kin) {return impl_.setZero(kin);}
void ChainState::setZero(Chain& kin) {return impl_.setZero(kin);}

void ChainState::copy(const ChainState& cpy, ChainState::CopyType what)
{
  ChainStateN<-1,rdyn::max_num_axes>::CopyType st = 
    what == ChainState::ONLY_JOINT ? ChainStateN<-1,rdyn::max_num_axes>::ONLY_JOINT :
    what == ChainState::ONLY_CART  ? ChainStateN<-1,rdyn::max_num_axes>::ONLY_CART  :
    ChainStateN<-1,rdyn::max_num_axes>::FULL_STATE;
  return impl_.copy(cpy.impl_, st);
}

const std::vector<std::string> ChainState::getJointNames() const { return impl_.getJointNames(); }
int ChainState::nAx() const { return impl_.nAx(); }

}

namespace std
{

std::string to_string(const rosdyn::ChainState& chain)
{
  std::string ret;
  ret += "q:     " + eigen_utils::to_string(chain.q())                        + "\n";
  ret += "qd:    " + eigen_utils::to_string(chain.qd())                       + "\n";
  ret += "qdd:   " + eigen_utils::to_string(chain.qdd())                      + "\n";
  ret += "eff:   " + eigen_utils::to_string(chain.effort())                   + "\n";
  ret += "tool:  " + eigen_utils::to_string(chain.toolPose().matrix(),false)  + "\n";
  ret += "twist: " + eigen_utils::to_string(chain.toolTwist())                + "\n";
  ret += "twistd:" + eigen_utils::to_string(chain.toolTwistd())               + "\n";
  return ret;
}
}
