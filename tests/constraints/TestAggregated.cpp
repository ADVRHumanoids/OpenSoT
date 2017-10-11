#include <gtest/gtest.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/constraints/BilateralConstraint.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/ConvexHull.h>
#include <string>
#include <XBotInterface/ModelInterface.h>


std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT-lite/tests/configs/coman/configs/config_coman.yaml";
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

void initializeIfNeeded()
{
    static bool is_initialized = false;

    if(!is_initialized) {
        time_t seed = time(NULL);
        seed48((unsigned short*)(&seed));
        srand((unsigned int)(seed));

        is_initialized = true;
    }

}

double getRandomAngle()
{
    initializeIfNeeded();
    return drand48()*2.0*M_PI-M_PI;
}

double getRandomAngle(const double min, const double max)
{
    initializeIfNeeded();
    assert(min <= max);
    if(min < -M_PI || max > M_PI)
        return getRandomAngle();

    return (double)rand()/RAND_MAX * (max-min) + min;
}

Eigen::VectorXd getRandomAngles(const Eigen::VectorXd &min,
                                               const Eigen::VectorXd &max,
                                               const int size)
{
    initializeIfNeeded();
    Eigen::VectorXd q(size);
    assert(min.size() >= size);
    assert(max.size() >= size);
    for(unsigned int i = 0; i < size; ++i)
        q(i) = getRandomAngle(min[i],max[i]);
    return q;
}

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

    Eigen::VectorXd q(nJ); q.setZero(nJ);
    Eigen::VectorXd q_next(nJ); q_next.setConstant(nJ, M_PI - 0.01);

    Eigen::MatrixXd A(nJ,nJ); A.setIdentity(nJ,nJ);
    Eigen::VectorXd bUpperBound(nJ); bUpperBound.setConstant(nJ, M_PI);
    Eigen::VectorXd bLowerBound(nJ); bLowerBound.setZero(nJ);
    constraints.push_back(Aggregated::ConstraintPtr(
        new BilateralConstraint(A,bUpperBound,bLowerBound)));

    constraints.push_back(Aggregated::ConstraintPtr(
        new velocity::JointLimits(q,bUpperBound,bLowerBound)));
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

    Eigen::VectorXd oldLowerBound = aggregated->getLowerBound();
    Eigen::VectorXd oldUpperBound = aggregated->getUpperBound();
    aggregated->update(q_next);
    Eigen::VectorXd newLowerBound = aggregated->getLowerBound();
    Eigen::VectorXd newUpperBound = aggregated->getUpperBound();
    EXPECT_FALSE(oldLowerBound == newLowerBound);
    EXPECT_FALSE(oldUpperBound == newUpperBound);
}

TEST_F(testAggregated, UnilateralToBilateralWorks) {

//     XBot::ModelInterface::Ptr _model_ptr;
//     _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);
// 
//     if(_model_ptr)
//         std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
//     else
//         std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;
// 
// 
//     Eigen::VectorXd q;
//     q.setZero(_model_ptr->getJointNum());
// 
//     std::list<std::string> _links_in_contact;
//     _links_in_contact.push_back("l_foot_lower_left_link");
//     _links_in_contact.push_back("l_foot_lower_right_link");
//     _links_in_contact.push_back("l_foot_upper_left_link");
//     _links_in_contact.push_back("l_foot_upper_right_link");
//     _links_in_contact.push_back("r_foot_lower_left_link");
//     _links_in_contact.push_back("r_foot_lower_right_link");
//     _links_in_contact.push_back("r_foot_upper_left_link");
//     _links_in_contact.push_back("r_foot_upper_right_link");
// 
//     OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr convexHull(
//                 new OpenSoT::constraints::velocity::ConvexHull(q,*(_model_ptr.get()), _links_in_contact));
//     
//     std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> constraints;
//     
//     constraints.push_back(convexHull);
//     
//     OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr aggregated(
//                 new OpenSoT::constraints::Aggregated(constraints,
//                                                      _model_ptr->getJointNum()));
// 
//     EXPECT_TRUE(aggregated->getbLowerBound().rows() == aggregated->getbUpperBound().rows()) <<
//                 "bLowerBound:" << aggregated->getbLowerBound() << std::endl <<
//                 "bUpperBound " << aggregated->getbUpperBound();
//     EXPECT_TRUE(aggregated->getAineq().rows() == aggregated->getbLowerBound().rows());

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
            new velocity::VelocityLimits(qDotMax,dT,nJ)));

    Eigen::VectorXd q(nJ);
    q = getRandomAngles(Eigen::VectorXd::Constant(nJ, -M_PI),
                                     Eigen::VectorXd::Constant(nJ, M_PI), nJ);

    Eigen::MatrixXd A;
    A.setZero(nJ,nJ);
    Eigen::VectorXd bUpperBound(nJ);
    bUpperBound<<bUpperBound.setOnes(nJ)*M_PI;
    Eigen::VectorXd bLowerBound;
    bLowerBound.setZero(nJ);
    constraints.push_back(Aggregated::ConstraintPtr(
        new BilateralConstraint(A, bUpperBound, bLowerBound)));

    Aggregated::ConstraintPtr aggregated(new Aggregated(constraints, q));

    constraints.push_back(  Aggregated::ConstraintPtr(
            new velocity::VelocityLimits(qDotMax/2,dT,nJ)));

    for(typename std::list<Aggregated::ConstraintPtr>::iterator i = constraints.begin();
        i != constraints.end(); ++i) {
        Aggregated::ConstraintPtr b(*i);
        if(b->getLowerBound().size() > 0)
            std::cout << b->getLowerBound() << std::endl;
    }
    Aggregated::ConstraintPtr aggregated2(new Aggregated(constraints, q));

    constraints.push_back(aggregated);
    constraints.push_back(aggregated2);

    Aggregated::ConstraintPtr aggregated3(new Aggregated(constraints, q));

    Aggregated::ConstraintPtr aggregated4(new Aggregated(aggregated2,
                                                        aggregated3, q.size()));

    Eigen::VectorXd qq(q.size());
    qq<<qq.setOnes()*-0.1;
    Aggregated::ConstraintPtr aggregated5(new Aggregated(aggregated4,
                                                        Aggregated::ConstraintPtr(
                                        new velocity::JointLimits(q,qq,qq)), q.size()));


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
