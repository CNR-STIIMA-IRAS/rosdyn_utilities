/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <math.h>
#include <iostream>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <rosdyn_chain_state/chain_state.h>
#include <rosdyn_chain_state/chain_state_publisher.h>
#include <gtest/gtest.h>

//======================================================================================================================
// to catch and print the exceptions
//======================================================================================================================
namespace detail
{
  struct unwrapper
  {
    unwrapper(std::exception_ptr pe) : pe_(pe) {}

    operator bool() const
    {
      return bool(pe_);
    }

    friend auto operator<<(std::ostream& os, unwrapper const& u) -> std::ostream&
    {
      try
      {
          std::rethrow_exception(u.pe_);
          return os << "no exception";
      }
      catch(std::runtime_error const& e)
      {
          return os << "runtime_error: " << e.what();
      }
      catch(std::logic_error const& e)
      {
          return os << "logic_error: " << e.what();
      }
      catch(std::exception const& e)
      {
          return os << "exception: " << e.what();
      }
      catch(...)
      {
          return os << "non-standard exception";
      }
    }
    std::exception_ptr pe_;
  };
}

auto unwrap(std::exception_ptr pe)
{
  return detail::unwrapper(pe);
}

template<class F>
::testing::AssertionResult no_throw(F&& f)
{
  try
  {
     f();
     return ::testing::AssertionSuccess();
  }
  catch(...)
  {
     return ::testing::AssertionFailure() << unwrap(std::current_exception());
  }
};
//======================================================================================================================

rosdyn::ChainPtr kin1  ;
rosdyn::ChainPtr kin3  ;
rosdyn::ChainPtr kin6  ;

using ChainStateX  = rosdyn::ChainStateX;
using ChainStateX20= rosdyn::ChainStateN<-1, 20>;
using ChainStateXX = rosdyn::ChainStateN<-1>;
using ChainState1  = rosdyn::ChainStateN<1>;
using ChainState3  = rosdyn::ChainStateN<3>;
using ChainState6  = rosdyn::ChainStateN<6>;


using ChainStatePublisherX20= rosdyn::ChainStatePublisherN<-1, 20>;
using ChainStatePublisherXX = rosdyn::ChainStatePublisherN<-1>;
using ChainStatePublisher1  = rosdyn::ChainStatePublisherN<1>;
using ChainStatePublisher3  = rosdyn::ChainStatePublisherN<3>;
using ChainStatePublisher6  = rosdyn::ChainStatePublisherN<6>;

std::shared_ptr<rosdyn::ChainState>  chain;
std::shared_ptr<ChainStateXX>        chainX;
std::shared_ptr<ChainStateX20>       chainX20;
std::shared_ptr<ChainStateXX>        chainXX;
std::shared_ptr<ChainState1>         chain1;
std::shared_ptr<ChainState3>         chain3;
std::shared_ptr<ChainState6>         chain6;

std::shared_ptr<ChainStatePublisherX20> chain_pub_X20;
std::shared_ptr<ChainStatePublisherXX>  chain_pub_XX;
std::shared_ptr<ChainStatePublisher1>   chain_pub_1;
std::shared_ptr<ChainStatePublisher3>   chain_pub_3;
std::shared_ptr<ChainStatePublisher6>   chain_pub_6;

TEST(TestSuite, chainInit)
{
  ASSERT_NO_THROW(kin1.reset(new rosdyn::Chain()););
  ASSERT_NO_THROW(kin3.reset(new rosdyn::Chain()););
  ASSERT_NO_THROW(kin6.reset(new rosdyn::Chain()););

  urdf::Model model;
  model.initParam("robot_description");

  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  std::string error;
  rosdyn::LinkPtr root_link;
  NEW_HEAP(root_link, rosdyn::Link());
  root_link->fromUrdf(GET(model.root_link_));

  std::stringstream report;
  int ret = -1;
  ASSERT_NO_THROW(kin1->init(error, root_link, "base_link","shoulder_link"););
  ASSERT_NO_THROW(kin3->init(error, root_link, "base_link","forearm_link") ;);
  ASSERT_NO_THROW(kin6->init(error, root_link, "base_link","tool0")        ;);

  if(ret==-1)
    std::cerr << "ERROR: " << report.str() << std::endl;

  if(ret==0)
    std::cerr << "WARNING: " << report.str() << std::endl;
}

TEST(TestSuite, emptyConstructor)
{ 
  ASSERT_TRUE(no_throw([]{chain   .reset(new rosdyn::ChainState());}));
  ASSERT_TRUE(no_throw([]{chainX20.reset(new ChainStateX20());}));
  ASSERT_TRUE(no_throw([]{chainXX .reset(new ChainStateXX ());}));
  ASSERT_TRUE(no_throw([]{chainX  .reset(new ChainStateX  ());}));
  ASSERT_TRUE(no_throw([]{chain1  .reset(new ChainState1  ());}));
  ASSERT_TRUE(no_throw([]{chain3  .reset(new ChainState3  ());}));
  ASSERT_TRUE(no_throw([]{chain6  .reset(new ChainState6  ());}));
}

TEST(TestSuite, init)
{
  EXPECT_TRUE(chain->init(kin1) );
  EXPECT_TRUE(chain->init(kin6) );

  EXPECT_TRUE(chainX20->init(kin1) );
  EXPECT_TRUE(chainX20->init(kin6) );

  EXPECT_TRUE(chainXX->init(kin3) );
  EXPECT_TRUE(chainXX->init(kin6) );

  EXPECT_TRUE(chainX->init(kin3) );
  EXPECT_TRUE(chainX->init(kin6) );

  EXPECT_TRUE(chain1->init(kin1) );
  EXPECT_FALSE(chain1->init(kin6) );

  EXPECT_TRUE(chain3->init(kin3) );
  EXPECT_FALSE(chain3->init(kin1) );

  EXPECT_TRUE(chain6->init(kin6) );
  EXPECT_FALSE(chain6->init(kin1) );
}

TEST(TestSuite, chainSetter)
{ //X20 is 6dof
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chain->q()              );}));
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chain->qd()             );}));
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chain->qdd()            );}));
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chain->effort()         );}));
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chain->external_effort());}));
  for(int i=0; i< chain->nAx();i++)
  {
    ASSERT_TRUE(no_throw([i]{chain->q(i)               = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([i]{chain->qd(i)              = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([i]{chain->qdd(i)             = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([i]{chain->effort(i)          = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([i]{chain->external_effort(i) = std::rand() / RAND_MAX;}));
  }
  for(const auto & n : chain->getJointNames())
  {
    ASSERT_TRUE(no_throw([n]{chain->q(n)              = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([n]{chain->qd(n)             = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([n]{chain->qdd(n)            = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([n]{chain->effort(n)         = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([n]{chain->external_effort(n)= std::rand() / RAND_MAX;}));
  }
}


TEST(TestSuite, chainSetterX20)
{ //X20 is 6dof
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chainX20->q()              );}));
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chainX20->qd()             );}));
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chainX20->qdd()            );}));
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chainX20->effort()         );}));
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chainX20->external_effort());}));
  for(int i=0; i< chainX20->nAx();i++)
  {
    ASSERT_TRUE(no_throw([i]{chainX20->q(i)               = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([i]{chainX20->qd(i)              = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([i]{chainX20->qdd(i)             = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([i]{chainX20->effort(i)          = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([i]{chainX20->external_effort(i) = std::rand() / RAND_MAX;}));
  }
  for(const auto & n : chainX20->getJointNames())
  {
    ASSERT_TRUE(no_throw([n]{chainX20->q(n)              = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([n]{chainX20->qd(n)             = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([n]{chainX20->qdd(n)            = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([n]{chainX20->effort(n)         = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([n]{chainX20->external_effort(n)= std::rand() / RAND_MAX;}));
  }
}


TEST(TestSuite, chainSetter6)
{ //6 is 6dof
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chain6->q()              );}));
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chain6->qd()             );}));
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chain6->qdd()            );}));
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chain6->effort()         );}));
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chain6->external_effort());}));
  for(int i=0; i< chain6->nAx();i++)
  {
    ASSERT_TRUE(no_throw([i]{chain6->q(i)               = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([i]{chain6->qd(i)              = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([i]{chain6->qdd(i)             = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([i]{chain6->effort(i)          = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([i]{chain6->external_effort(i) = std::rand() / RAND_MAX;}));
  }
  for(const auto & n : chain6->getJointNames())
  {
    ASSERT_TRUE(no_throw([n]{chain6->q(n)              = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([n]{chain6->qd(n)             = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([n]{chain6->qdd(n)            = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([n]{chain6->effort(n)         = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([n]{chain6->external_effort(n)= std::rand() / RAND_MAX;}));
  }
}



TEST(TestSuite, chainSetter1)
{ //1 is 1dof
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chain1->q()               );}));
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chain1->qd()              );}));
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chain1->qdd()             );}));
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chain1->effort()          );}));
  ASSERT_TRUE(no_throw([]{eigen_utils::setRandom(chain1->external_effort() );}));
  for(int i=0; i< chain1->nAx();i++)
  {
    ASSERT_TRUE(no_throw([i]{chain1->q(i)               = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([i]{chain1->qd(i)              = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([i]{chain1->qdd(i)             = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([i]{chain1->effort(i)          = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([i]{chain1->external_effort(i) = std::rand() / RAND_MAX;}));
  }
  for(const auto & n : chain1->getJointNames())
  {
    ASSERT_TRUE(no_throw([n]{chain1->q(n)              = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([n]{chain1->qd(n)             = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([n]{chain1->qdd(n)            = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([n]{chain1->effort(n)         = std::rand() / RAND_MAX;}));
    ASSERT_TRUE(no_throw([n]{chain1->external_effort(n)= std::rand() / RAND_MAX;}));
  }
}


TEST(TestSuite, chainGetter)
{
  ASSERT_TRUE(no_throw([]{chain->q()              ;}));
  ASSERT_TRUE(no_throw([]{chain->qd()             ;}));
  ASSERT_TRUE(no_throw([]{chain->qdd()            ;}));
  ASSERT_TRUE(no_throw([]{chain->effort()         ;}));
  ASSERT_TRUE(no_throw([]{chain->external_effort();}));

  for(int i=0; i< chain->nAx();i++)
  {
    ASSERT_TRUE(no_throw([i]{chain->q(i)              ;}));
    ASSERT_TRUE(no_throw([i]{chain->qd(i)             ;}));
    ASSERT_TRUE(no_throw([i]{chain->qdd(i)            ;}));
    ASSERT_TRUE(no_throw([i]{chain->effort(i)         ;}));
    ASSERT_TRUE(no_throw([i]{chain->external_effort(i);}));
  }
  ASSERT_FALSE(no_throw([]{chain->q(-1);}));
  ASSERT_FALSE(no_throw([]{chain->q(1234);}));

  for(const auto & n : chain->getJointNames())
  {
    ASSERT_TRUE(no_throw([n]{chain->q(n)              ;}));
    ASSERT_TRUE(no_throw([n]{chain->qd(n)             ;}));
    ASSERT_TRUE(no_throw([n]{chain->qdd(n)            ;}));
    ASSERT_TRUE(no_throw([n]{chain->effort(n)         ;}));
    ASSERT_TRUE(no_throw([n]{chain->external_effort(n);}));
  }
  ASSERT_FALSE(no_throw([]{chain->q("foo_the_name_does_not_exist");}));

  ASSERT_TRUE(no_throw([]{chain->toolPose( ).matrix() ;}));
  ASSERT_TRUE(no_throw([]{chain->toolTwist( ).transpose() ;}));
  ASSERT_TRUE(no_throw([]{chain->toolTwistd( ).transpose();}));
  ASSERT_TRUE(no_throw([]{chain->toolWrench( ).transpose();}));
  ASSERT_TRUE(no_throw([]{chain->toolJacobian( )          ;}));
}


TEST(TestSuite, chainGetterX20)
{
  ASSERT_TRUE(no_throw([]{chainX20->q()              ;}));
  ASSERT_TRUE(no_throw([]{chainX20->qd()             ;}));
  ASSERT_TRUE(no_throw([]{chainX20->qdd()            ;}));
  ASSERT_TRUE(no_throw([]{chainX20->effort()         ;}));
  ASSERT_TRUE(no_throw([]{chainX20->external_effort();}));

  for(int i=0; i< chainX20->nAx();i++)
  {
    ASSERT_TRUE(no_throw([i]{chainX20->q(i)              ;}));
    ASSERT_TRUE(no_throw([i]{chainX20->qd(i)             ;}));
    ASSERT_TRUE(no_throw([i]{chainX20->qdd(i)            ;}));
    ASSERT_TRUE(no_throw([i]{chainX20->effort(i)         ;}));
    ASSERT_TRUE(no_throw([i]{chainX20->external_effort(i);}));
  }
  ASSERT_FALSE(no_throw([]{chainX20->q(-1);}));
  ASSERT_FALSE(no_throw([]{chainX20->q(1234);}));

  for(const auto & n : chainX20->getJointNames())
  {
    ASSERT_TRUE(no_throw([n]{chainX20->q(n)              ;}));
    ASSERT_TRUE(no_throw([n]{chainX20->qd(n)             ;}));
    ASSERT_TRUE(no_throw([n]{chainX20->qdd(n)            ;}));
    ASSERT_TRUE(no_throw([n]{chainX20->effort(n)         ;}));
    ASSERT_TRUE(no_throw([n]{chainX20->external_effort(n);}));
  }
  ASSERT_FALSE(no_throw([]{chainX20->q("foo_the_name_does_not_exist");}));

  ASSERT_TRUE(no_throw([]{chainX20->toolPose( ).matrix() ;}));
  ASSERT_TRUE(no_throw([]{chainX20->toolTwist( ).transpose() ;}));
  ASSERT_TRUE(no_throw([]{chainX20->toolTwistd( ).transpose();}));
  ASSERT_TRUE(no_throw([]{chainX20->toolWrench( ).transpose();}));
  ASSERT_TRUE(no_throw([]{chainX20->toolJacobian( )          ;}));
}



TEST(TestSuite, chainGetter6)
{
  ASSERT_TRUE(no_throw([]{chain6->q()              ;}));
  ASSERT_TRUE(no_throw([]{chain6->qd()             ;}));
  ASSERT_TRUE(no_throw([]{chain6->qdd()            ;}));
  ASSERT_TRUE(no_throw([]{chain6->effort()         ;}));
  ASSERT_TRUE(no_throw([]{chain6->external_effort();}));

  for(int i=0; i< chain6->nAx();i++)
  {
    ASSERT_TRUE(no_throw([i]{chain6->q(i)              ;}));
    ASSERT_TRUE(no_throw([i]{chain6->qd(i)             ;}));
    ASSERT_TRUE(no_throw([i]{chain6->qdd(i)            ;}));
    ASSERT_TRUE(no_throw([i]{chain6->effort(i)         ;}));
    ASSERT_TRUE(no_throw([i]{chain6->external_effort(i);}));
  }
  ASSERT_FALSE(no_throw([]{chain6->q(-1);}));
  ASSERT_FALSE(no_throw([]{chain6->q(7);}));

  for(const auto & n : chain6->getJointNames())
  {
    ASSERT_TRUE(no_throw([n]{chain6->q(n)              ;}));
    ASSERT_TRUE(no_throw([n]{chain6->qd(n)             ;}));
    ASSERT_TRUE(no_throw([n]{chain6->qdd(n)            ;}));
    ASSERT_TRUE(no_throw([n]{chain6->effort(n)         ;}));
    ASSERT_TRUE(no_throw([n]{chain6->external_effort(n);}));
  }

  ASSERT_FALSE(no_throw([]{chain6->q("foo_the_name_does_not_exist");}));
  ASSERT_TRUE(no_throw([]{chain6->toolPose( ).matrix() ;}));
  ASSERT_TRUE(no_throw([]{chain6->toolTwist( ).transpose() ;}));
  ASSERT_TRUE(no_throw([]{chain6->toolTwistd( ).transpose();}));
  ASSERT_TRUE(no_throw([]{chain6->toolWrench( ).transpose();}));
  ASSERT_TRUE(no_throw([]{chain6->toolJacobian( )          ;}));
}

TEST(TestSuite, handles)
{
  chain->q() = Eigen::Matrix<double,6,1,Eigen::ColMajor,6,1>().setRandom();
  std::cout << "q: " << eigen_utils::to_string( chain->q() ) << std::endl;

  double* d = nullptr;
  ASSERT_TRUE(no_throw([&d]{ d = chain->handle_to_q(); } ));

  d[3] = 100;
  std::cout << "q: " << eigen_utils::to_string( chain->q() ) << std::endl;
}

TEST(TestSuite, handlesX20)
{
  chainX20->q() = Eigen::Matrix<double,6,1,Eigen::ColMajor,6,1>().setRandom();
  std::cout << "q: " << eigen_utils::to_string( chainX20->q() ) << std::endl;

  double* d = nullptr;
  ASSERT_TRUE(no_throw([&d]{ d = chainX20->handle_to_q(); } ));

  d[3] = 100;
  std::cout << "q: " << eigen_utils::to_string( chainX20->q() ) << std::endl;
}

TEST(TestSuite, fkinTest)
{

  Eigen::Matrix<double,6,1> tmp;
  tmp << -1.0761402289019983, -2.00346547762026, -1.6660807768451136, -0.1624382177935999, 0.8131169676780701, 0.1402850896120071;
  auto tool_rosdyn = kin6->getTransformation(tmp);

  chain->q() = tmp;
  std::cout << "q: " << eigen_utils::to_string( chain->q() ) << std::endl;

  chain->updateTransformations(kin6, rosdyn::ChainState::SECOND_ORDER|rosdyn::ChainState::FFWD_STATIC);

  std::cout << "rosdyn fkin:\n" << eigen_utils::to_string( tool_rosdyn.matrix(), false ) << std::endl;
  std::cout << "chain state fkin:\n" << eigen_utils::to_string( chain->toolPose().matrix(), false ) << std::endl;
  std::cout << "diff:\n" << eigen_utils::to_string( (tool_rosdyn * chain->toolPose().inverse()).matrix(), false ) << std::endl;
  ASSERT_TRUE( ((tool_rosdyn * chain->toolPose().inverse()).matrix() - Eigen::MatrixXd::Identity(4,4)).cwiseAbs().maxCoeff()<1e-4);
}


TEST(TestSuite, jacTest)
{

  Eigen::Matrix<double,6,1> tmp;
  tmp << -1.0761402289019983, -2.00346547762026, -1.6660807768451136, -0.1624382177935999, 0.8131169676780701, 0.1402850896120071;
  auto jac_rosdyn = kin6->getJacobian(tmp);


  chain->q() = tmp;
  std::cout << "q: " << eigen_utils::to_string( chain->q() ) << std::endl;

  chain->updateTransformations(kin6, rosdyn::ChainState::SECOND_ORDER|rosdyn::ChainState::FFWD_STATIC);

  std::cout << "rosdyn jacobian:\n" << eigen_utils::to_string( jac_rosdyn, false ) << std::endl;
  std::cout << "chain state jacobian:\n" << eigen_utils::to_string( chain->toolJacobian(), false ) << std::endl;

  ASSERT_TRUE( (jac_rosdyn-chain->toolJacobian()).cwiseAbs().maxCoeff()<1e-4);
}



TEST(TestSuite, publisherConstructors)
{
  ros::NodeHandle nh("~");
  ASSERT_TRUE(no_throw([&nh]{chain_pub_XX .reset( new ChainStatePublisherXX (nh, "chain_pub_XX", *kin6, chainXX .get()));}));
  ASSERT_TRUE(no_throw([&nh]{chain_pub_1  .reset( new ChainStatePublisher1  (nh, "chain_pub_1" , *kin1, chain1  .get()));}));
  ASSERT_TRUE(no_throw([&nh]{chain_pub_3  .reset( new ChainStatePublisher3  (nh, "chain_pub_3" , *kin3, chain3  .get()));}));
  ASSERT_TRUE(no_throw([&nh]{chain_pub_6  .reset( new ChainStatePublisher6  (nh, "chain_pub_6" , *kin6, chain6  .get()));}));
  ASSERT_TRUE(no_throw([&nh]{chain_pub_X20.reset( new ChainStatePublisherX20(nh, "chain_pub_X" , *kin6, chainX20.get()));}));
}

TEST(TestSuite, publish)
{
  ASSERT_TRUE(no_throw([]{chain_pub_X20->publish();}));
  ASSERT_TRUE(no_throw([]{chain_pub_XX->publish() ;}));
  ASSERT_TRUE(no_throw([]{chain_pub_1->publish()  ;}));
  ASSERT_TRUE(no_throw([]{chain_pub_3->publish()  ;}));
  ASSERT_TRUE(no_throw([]{chain_pub_6->publish()  ;}));
}

TEST(TestSuite, publisherDestructors)
{
  ASSERT_TRUE(no_throw([]{chain_pub_X20.reset();}));
  ASSERT_TRUE(no_throw([]{chain_pub_XX .reset();}));
  ASSERT_TRUE(no_throw([]{chain_pub_1  .reset();}));
  ASSERT_TRUE(no_throw([]{chain_pub_3  .reset();}));
  ASSERT_TRUE(no_throw([]{chain_pub_6  .reset();}));
}

TEST(TestSuite, desctructor)
{
  ASSERT_TRUE(no_throw([]{chainX20.reset();}));
  ASSERT_TRUE(no_throw([]{chainXX .reset();}));
  ASSERT_TRUE(no_throw([]{chain1  .reset();}));
  ASSERT_TRUE(no_throw([]{chain3  .reset();}));
  ASSERT_TRUE(no_throw([]{chain6  .reset();}));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "cnr_logger_tester");

  return RUN_ALL_TESTS();
}
