#include <gtest/gtest.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/BilateralConstraint.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/ConvexHull.h>
#include <string>
#include <xbot2_interface/xbotinterface2.h>
#include "../common.h"


namespace {

// The fixture for testing class Foo.
class testAggregated : public TestBase {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  testAggregated() : TestBase("coman_floating_base") {
    // You can do set-up work for each test here.
  }

  virtual ~testAggregated() {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for Foo.
};


// Tests that the Foo::Bar() method does Abc.
TEST_F(testAggregated, AggregatedWorks) {
    using namespace OpenSoT::constraints;
    std::list<Aggregated::ConstraintPtr> constraints;
    const double dT = 0.1;
    const double qDotMax = 0.5;

    Eigen::VectorXd q = _model_ptr->getNeutralQ();
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    constraints.push_back(  Aggregated::ConstraintPtr(
            new velocity::VelocityLimits(*_model_ptr, qDotMax,dT)
                                                    )
                          );


    Eigen::VectorXd q_next = _model_ptr->generateRandomQ();

    Eigen::MatrixXd A(_model_ptr->getNv(),_model_ptr->getNv()); A.setIdentity();
    Eigen::VectorXd bUpperBound(_model_ptr->getNv()); bUpperBound.setConstant(_model_ptr->getNv(), M_PI);
    Eigen::VectorXd bLowerBound(_model_ptr->getNv()); bLowerBound.setZero();
    constraints.push_back(Aggregated::ConstraintPtr(
        new BilateralConstraint(A,bUpperBound,bLowerBound)));

    constraints.push_back(Aggregated::ConstraintPtr(
        new velocity::JointLimits(*_model_ptr,bUpperBound,bLowerBound)));
    Aggregated::ConstraintPtr aggregated(new Aggregated(constraints, _model_ptr->getNv()));

    /* we should mash joint limits and velocity limits in one */
    EXPECT_TRUE(aggregated->getLowerBound().size() == _model_ptr->getNv());
    EXPECT_TRUE(aggregated->getUpperBound().size() == _model_ptr->getNv());
    /* we have a BilateralConstraint... */
    EXPECT_TRUE(aggregated->getAineq().rows() == _model_ptr->getNv());
    EXPECT_TRUE(aggregated->getbLowerBound().size() == _model_ptr->getNv());
    EXPECT_TRUE(aggregated->getbUpperBound().size() == _model_ptr->getNv());
    /* and no equality constraint */
    EXPECT_TRUE(aggregated->getAeq().rows() == 0);
    EXPECT_TRUE(aggregated->getbeq().size() == 0);

    Eigen::VectorXd oldLowerBound = aggregated->getLowerBound();
    Eigen::VectorXd oldUpperBound = aggregated->getUpperBound();

    _model_ptr->setJointPosition(q_next);
    _model_ptr->update();
    aggregated->update();

    Eigen::VectorXd newLowerBound = aggregated->getLowerBound();
    Eigen::VectorXd newUpperBound = aggregated->getUpperBound();
    EXPECT_FALSE(oldLowerBound == newLowerBound);
    EXPECT_FALSE(oldUpperBound == newUpperBound);
}

TEST_F(testAggregated, UnilateralToBilateralWorks) {


     Eigen::VectorXd q = _model_ptr->getNeutralQ();

     _model_ptr->setJointPosition(q);
     _model_ptr->update();

     std::list<std::string> _links_in_contact;
     _links_in_contact.push_back("l_foot_lower_left_link");
     _links_in_contact.push_back("l_foot_lower_right_link");
     _links_in_contact.push_back("l_foot_upper_left_link");
     _links_in_contact.push_back("l_foot_upper_right_link");
     _links_in_contact.push_back("r_foot_lower_left_link");
     _links_in_contact.push_back("r_foot_lower_right_link");
     _links_in_contact.push_back("r_foot_upper_left_link");
     _links_in_contact.push_back("r_foot_upper_right_link");

     OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr convexHull(
                 new OpenSoT::constraints::velocity::ConvexHull(*_model_ptr.get(), _links_in_contact));

     std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> constraints;

     constraints.push_back(convexHull);

     OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr aggregated(
                 new OpenSoT::constraints::Aggregated(constraints,
                                                      _model_ptr->getNv()));

     EXPECT_TRUE(aggregated->getbLowerBound().rows() == aggregated->getbUpperBound().rows()) <<
                 "bLowerBound:" << aggregated->getbLowerBound() << std::endl <<
                 "bUpperBound " << aggregated->getbUpperBound();
     EXPECT_TRUE(aggregated->getAineq().rows() == aggregated->getbLowerBound().rows());

}

/**
 * @todo implement
 */
TEST_F(testAggregated, EqualityToInequalityWorks) {
}

TEST_F(testAggregated, MultipleAggregationdWork) {
    using namespace OpenSoT::constraints;
    std::list<Aggregated::ConstraintPtr> constraints;
    const double dT = 1.;
    const double qDotMax = 50.;

    Eigen::VectorXd q = _model_ptr->getNeutralQ();
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    constraints.push_back(  Aggregated::ConstraintPtr(
            new velocity::VelocityLimits(*_model_ptr, qDotMax,dT)
                                                    )
                          );

    Eigen::MatrixXd A;
    A.setZero(_model_ptr->getNv(),_model_ptr->getNv());
    Eigen::VectorXd bUpperBound(_model_ptr->getNv());
    bUpperBound<<bUpperBound.setOnes(_model_ptr->getNv())*M_PI;
    Eigen::VectorXd bLowerBound;
    bLowerBound.setZero(_model_ptr->getNv());
    constraints.push_back(Aggregated::ConstraintPtr(
        new BilateralConstraint(A, bUpperBound, bLowerBound)));

    Aggregated::ConstraintPtr aggregated(new Aggregated(constraints, _model_ptr->getNv()));

    constraints.push_back(  Aggregated::ConstraintPtr(
            new velocity::VelocityLimits(*_model_ptr, qDotMax/2,dT)));

    for(typename std::list<Aggregated::ConstraintPtr>::iterator i = constraints.begin();
        i != constraints.end(); ++i) {
        Aggregated::ConstraintPtr b(*i);
        if(b->getLowerBound().size() > 0)
            std::cout << b->getLowerBound() << std::endl;
    }
    Aggregated::ConstraintPtr aggregated2(new Aggregated(constraints, _model_ptr->getNv()));

    constraints.push_back(aggregated);
    constraints.push_back(aggregated2);

    Aggregated::ConstraintPtr aggregated3(new Aggregated(constraints, _model_ptr->getNv()));

    Aggregated::ConstraintPtr aggregated4(new Aggregated(aggregated2,
                                                        aggregated3, _model_ptr->getNv()));

    Eigen::VectorXd qq(_model_ptr->getNv());
    qq<<qq.setOnes()*-0.1;
    Aggregated::ConstraintPtr aggregated5(new Aggregated(aggregated4,
                                                        Aggregated::ConstraintPtr(
                                        new velocity::JointLimits(*_model_ptr,qq,qq)), _model_ptr->getNv()));


    /* testing constraints stay put */
    ASSERT_TRUE(aggregated->getbLowerBound() == aggregated2->getbLowerBound());
    ASSERT_TRUE(aggregated->getbUpperBound() == aggregated2->getbUpperBound());

    /* testing aggregation rules for multiple bounds */
    for(unsigned int i = 0; i < aggregated->getLowerBound().size(); ++i) {
        ASSERT_NEAR(aggregated->getLowerBound()[i], 2*aggregated2->getLowerBound()[i], 1E-16);
        ASSERT_NEAR(aggregated->getUpperBound()[i], 2*aggregated2->getUpperBound()[i], 1E-16);
    }

    /* testing bounds don't go into constraints */
    ASSERT_TRUE(aggregated2->getLowerBound() == aggregated3->getLowerBound());
    ASSERT_TRUE(aggregated2->getUpperBound() == aggregated3->getUpperBound());
    // aggregated5->getLowerBound() > aggregated4.getLowerBound()
    Eigen::VectorXd a = aggregated5->getLowerBound();
    Eigen::VectorXd b = aggregated4->getLowerBound();
    double min = (a-b).minCoeff() ;
    ASSERT_GT(min, 0);
    // aggregated5->getUpperBound() < aggregated4.getUpperBound()
    a = aggregated5->getUpperBound();
    b = aggregated4->getUpperBound();
    double max = (a-b).maxCoeff() ;
    ASSERT_LT(max,0);
}


}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
