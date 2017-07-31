#include <advr_humanoids_common_utils/test_utils.h>
#include <gtest/gtest.h>
#include <OpenSoT/tasks/velocity/MinimumVelocity.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <yarp/math/Math.h>
#include <advr_humanoids_common_utils/conversion_utils_YARP.h>

using namespace yarp::math;

namespace {

class testMinimumVelocityTask: public ::testing::Test
{
protected:

    testMinimumVelocityTask()
    {

    }

    virtual ~testMinimumVelocityTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

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

TEST_F(testMinimumVelocityTask, testMinimumVelocityTask_)
{
    Eigen::VectorXd q(6);
    for(unsigned int i = 0; i < q.size(); ++i)
        q[i] = getRandomAngle();

    Eigen::VectorXd dq_zeros(q.size());
    dq_zeros.setZero(q.size());

    OpenSoT::tasks::velocity::MinimumVelocity minimumVelocity(q.size());

    Eigen::MatrixXd I(q.size(), q.size()); I.setIdentity(q.size(), q.size());
    EXPECT_TRUE(minimumVelocity.getA() == I);
    EXPECT_TRUE(minimumVelocity.getWeight() == I);

    EXPECT_TRUE(minimumVelocity.getConstraints().size() == 0);

    double K = 0.1;
    minimumVelocity.setLambda(K);
    EXPECT_DOUBLE_EQ(minimumVelocity.getLambda(), K);

    minimumVelocity.update(q);
    EXPECT_TRUE(minimumVelocity.getb() == dq_zeros);

    for(unsigned int i = 0; i < 100; ++i)
    {
        minimumVelocity.update(q);
        q += minimumVelocity.getLambda()*minimumVelocity.getb();
    }

    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR(q[i], q[i], 1E-3);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
