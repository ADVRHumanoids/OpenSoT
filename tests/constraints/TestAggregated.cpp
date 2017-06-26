#include <gtest/gtest.h>
#include <advr_humanoids_common_utils/test_utils.h>
#include <advr_humanoids_common_utils/idynutils.h>
#include <advr_humanoids_common_utils/conversion_utils_YARP.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/BilateralConstraint.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/ConvexHull.h>
#include <yarp/math/Math.h>
#include <string>
#include <ModelInterfaceIDYNUTILS/ModelInterfaceIDYNUTILS.h>

using namespace yarp::math;

typedef idynutils2 iDynUtils;
static void null_deleter(iDynUtils *) {}
std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman.yaml";
std::string _path_to_cfg = robotology_root + relative_path;

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
        new BilateralConstraint(conversion_utils_YARP::toEigen(A),
                                conversion_utils_YARP::toEigen(bUpperBound),
                                conversion_utils_YARP::toEigen(bLowerBound))
                                                  )
                          );

    constraints.push_back(Aggregated::ConstraintPtr(
        new velocity::JointLimits(conversion_utils_YARP::toEigen(q),
                                  conversion_utils_YARP::toEigen(bUpperBound),
                                  conversion_utils_YARP::toEigen(bLowerBound))
                          ));
    Aggregated::ConstraintPtr aggregated(new Aggregated(constraints, conversion_utils_YARP::toEigen(q)));

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

    Eigen::VectorXd oldLowerBound = aggregated->getLowerBound();
    Eigen::VectorXd oldUpperBound = aggregated->getUpperBound();
    aggregated->update(conversion_utils_YARP::toEigen(q_next));
    Eigen::VectorXd newLowerBound = aggregated->getLowerBound();
    Eigen::VectorXd newUpperBound = aggregated->getUpperBound();
    EXPECT_FALSE(oldLowerBound == newLowerBound);
    EXPECT_FALSE(oldUpperBound == newUpperBound);
}

TEST_F(testAggregated, UnilateralToBilateralWorks) {

    iDynUtils robot("coman",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                    std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");


    XBot::ModelInterfaceIDYNUTILS::Ptr _model_ptr;
    _model_ptr = std::dynamic_pointer_cast<XBot::ModelInterfaceIDYNUTILS>
            (XBot::ModelInterface::getModel(_path_to_cfg));
    _model_ptr->loadModel(boost::shared_ptr<iDynUtils>(&robot, &null_deleter));

    if(_model_ptr)
        std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
    else
        std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;


    Eigen::VectorXd q;
    q.setZero(robot.iDynTree_model.getNrOfDOFs());

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
                new OpenSoT::constraints::velocity::ConvexHull(q,*(_model_ptr.get()), _links_in_contact));
    std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> constraints;
    constraints.push_back(convexHull);
    OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr aggregated(
                new OpenSoT::constraints::Aggregated(constraints,
                                                     robot.iDynTree_model.getNrOfDOFs()));

    EXPECT_TRUE(aggregated->getbLowerBound().rows() == aggregated->getbUpperBound().rows()) <<
                "bLowerBound:" << aggregated->getbLowerBound() << std::endl <<
                "bUpperBound " << aggregated->getbUpperBound();
    EXPECT_TRUE(aggregated->getAineq().rows() == aggregated->getbLowerBound().rows());

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
    q = conversion_utils_YARP::toYARP(tests_utils::getRandomAngles(
                conversion_utils_YARP::toEigen(yarp::sig::Vector(nJ,-M_PI)),
                conversion_utils_YARP::toEigen(yarp::sig::Vector(nJ,M_PI)),
                                     nJ));

    Eigen::MatrixXd A;
    A.setZero(nJ,nJ);
    Eigen::VectorXd bUpperBound(nJ);
    bUpperBound<<bUpperBound.setOnes(nJ)*M_PI;
    Eigen::VectorXd bLowerBound;
    bLowerBound.setZero(nJ);
    constraints.push_back(Aggregated::ConstraintPtr(
        new BilateralConstraint(A, bUpperBound, bLowerBound)
                                                  )
                          );

    Aggregated::ConstraintPtr aggregated(new Aggregated(constraints, conversion_utils_YARP::toEigen(q)));

    constraints.push_back(  Aggregated::ConstraintPtr(
            new velocity::VelocityLimits(qDotMax/2,dT,nJ)                        )
                          );

    for(typename std::list<Aggregated::ConstraintPtr>::iterator i = constraints.begin();
        i != constraints.end(); ++i) {
        Aggregated::ConstraintPtr b(*i);
        if(b->getLowerBound().size() > 0)
            std::cout << b->getLowerBound() << std::endl;
    }
    Aggregated::ConstraintPtr aggregated2(new Aggregated(constraints, conversion_utils_YARP::toEigen(q)));

    constraints.push_back(aggregated);
    constraints.push_back(aggregated2);

    Aggregated::ConstraintPtr aggregated3(new Aggregated(constraints, conversion_utils_YARP::toEigen(q)));

    Aggregated::ConstraintPtr aggregated4(new Aggregated(aggregated2,
                                                        aggregated3, q.size()));

    Eigen::VectorXd qq(q.size());
    qq<<qq.setOnes()*-0.1;
    Aggregated::ConstraintPtr aggregated5(new Aggregated(aggregated4,
                                                        Aggregated::ConstraintPtr(
                                        new velocity::JointLimits(conversion_utils_YARP::toEigen(q),
                                                                  qq,
                                                                  qq)
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
    yarp::sig::Vector a = conversion_utils_YARP::toYARP(aggregated5->getLowerBound());
    yarp::sig::Vector b = conversion_utils_YARP::toYARP(aggregated4->getLowerBound());
    double min = findMin(a-b);
    ASSERT_GT(min, 0);
    // aggregated5->getUpperBound() < aggregated4.getUpperBound()
    a = conversion_utils_YARP::toYARP(aggregated5->getUpperBound());
    b = conversion_utils_YARP::toYARP(aggregated4->getUpperBound());
    double max = findMax(a-b);
    ASSERT_LT(max,0);
}


}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
