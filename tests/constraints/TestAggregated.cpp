#include <gtest/gtest.h>
#include <drc_shared/tests_utils.h>
#include <drc_shared/idynutils.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/BilateralConstraint.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/ConvexHull.h>
#include <yarp/math/Math.h>
#include <string>

using namespace yarp::math;

namespace {

// The fixture for testing class Foo.
class testAggregated : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  testAggregated() {
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
    const unsigned int nJ = 6;
    const double dT = 0.1;
    const double qDotMax = 0.5;
    constraints.push_back(  Aggregated::ConstraintPtr(
            new velocity::VelocityLimits(qDotMax,dT,nJ)
                                                    )
                          );

    yarp::sig::Vector q(nJ, 0.0);
    yarp::sig::Vector q_next(nJ, M_PI - 0.01);

    yarp::sig::Matrix A(nJ,nJ); A.eye();
    yarp::sig::Vector bUpperBound(nJ,M_PI);
    yarp::sig::Vector bLowerBound(nJ,0.0);
    constraints.push_back(Aggregated::ConstraintPtr(
        new BilateralConstraint(A, bUpperBound, bLowerBound)
                                                  )
                          );

    constraints.push_back(Aggregated::ConstraintPtr(
        new velocity::JointLimits(q, bUpperBound, bLowerBound)
                                                  )
                          );
    Aggregated::ConstraintPtr aggregated(new Aggregated(constraints, q));

    /* we should mash joint limits and velocity limits in one */
    EXPECT_TRUE(aggregated->getLowerBound().size() == nJ);
    EXPECT_TRUE(aggregated->getUpperBound().size() == nJ);
    /* we have a BilateralConstraint... */
    EXPECT_TRUE(aggregated->getAineq().rows() == nJ);
    EXPECT_TRUE(aggregated->getbLowerBound().size() == nJ);
    EXPECT_TRUE(aggregated->getbUpperBound().size() == nJ);
    /* and no equality constraint */
    EXPECT_TRUE(aggregated->getAeq().rows() == 0);
    EXPECT_TRUE(aggregated->getbeq().size() == 0);

    yarp::sig::Vector oldLowerBound = aggregated->getLowerBound();
    yarp::sig::Vector oldUpperBound = aggregated->getUpperBound();
    aggregated->update(q_next);
    yarp::sig::Vector newLowerBound = aggregated->getLowerBound();
    yarp::sig::Vector newUpperBound = aggregated->getUpperBound();
    EXPECT_FALSE(oldLowerBound == newLowerBound);
    EXPECT_FALSE(oldUpperBound == newUpperBound);
}

TEST_F(testAggregated, UnilateralToBilateralWorks) {
    using namespace yarp::sig;
    iDynUtils robot;
    Vector q(robot.coman_iDyn3.getNrOfDOFs(),0.0);

    OpenSoT::Constraint<Matrix, Vector>::ConstraintPtr convexHull(
                new OpenSoT::constraints::velocity::ConvexHull(q,robot));
    std::list<OpenSoT::Constraint<Matrix, Vector>::ConstraintPtr> constraints;
    constraints.push_back(convexHull);
    OpenSoT::Constraint<Matrix, Vector>::ConstraintPtr aggregated(
                new OpenSoT::constraints::Aggregated(constraints,
                                                     robot.coman_iDyn3.getNrOfDOFs()));

    EXPECT_TRUE(aggregated->getbLowerBound().size() == aggregated->getbUpperBound().size()) <<
                "bLowerBound:" << aggregated->getbLowerBound().toString() << std::endl <<
                "bUpperBound " << aggregated->getbUpperBound().toString();
    EXPECT_TRUE(aggregated->getAineq().rows() == aggregated->getbLowerBound().size());

}

/// TODO implement
TEST_F(testAggregated, EqualityToInequalityWorks) {
}

TEST_F(testAggregated, MultipleAggregationdWork) {
    using namespace OpenSoT::constraints;
    std::list<Aggregated::ConstraintPtr> constraints;
    const unsigned int nJ = 6;
    const double dT = 1;
    const double qDotMax = 50;
    constraints.push_back(  Aggregated::ConstraintPtr(
            new velocity::VelocityLimits(qDotMax,dT,nJ)
                                                    )
                          );

    yarp::sig::Vector q(nJ);
    q = tests_utils::getRandomAngles(yarp::sig::Vector(nJ,-M_PI),
                                     yarp::sig::Vector(nJ,M_PI),
                                     nJ);

    yarp::sig::Matrix A(nJ,nJ); A.eye();
    yarp::sig::Vector bUpperBound(nJ,M_PI);
    yarp::sig::Vector bLowerBound(nJ,0.0);
    constraints.push_back(Aggregated::ConstraintPtr(
        new BilateralConstraint(A, bUpperBound, bLowerBound)
                                                  )
                          );

    Aggregated::ConstraintPtr aggregated(new Aggregated(constraints, q));

    constraints.push_back(  Aggregated::ConstraintPtr(
            new velocity::VelocityLimits(qDotMax/2,dT,nJ)                        )
                          );

    for(typename std::list<Aggregated::ConstraintPtr>::iterator i = constraints.begin();
        i != constraints.end(); ++i) {
        Aggregated::ConstraintPtr b(*i);
        if(b->getLowerBound().size() > 0)
            std::cout << b->getLowerBound().toString() << std::endl;
    }
    Aggregated::ConstraintPtr aggregated2(new Aggregated(constraints, q));

    constraints.push_back(aggregated);
    constraints.push_back(aggregated2);

    Aggregated::ConstraintPtr aggregated3(new Aggregated(constraints, q));

    Aggregated::ConstraintPtr aggregated4(new Aggregated(aggregated2,
                                                        aggregated3, q.size()));

    Aggregated::ConstraintPtr aggregated5(new Aggregated(aggregated4,
                                                        Aggregated::ConstraintPtr(
                                        new velocity::JointLimits(q,
                                                                  yarp::sig::Vector(q.size(),-0.1),
                                                                  yarp::sig::Vector(q.size(),-0.1))
                                                        ), q.size()));


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
    ASSERT_GT(findMin(aggregated5->getLowerBound() - aggregated4->getLowerBound()), 0);
    // aggregated5->getUpperBound() < aggregated4.getUpperBound()
    ASSERT_LT(findMax(aggregated5->getUpperBound() - aggregated4->getUpperBound()),0);
}


}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
