#include <drc_shared/tests_utils.h>
#include <gtest/gtest.h>
#include <wb_sot/tasks/velocity/Postural.h>
#include <yarp/math/Math.h>

using namespace yarp::math;

namespace {

class testPosturalTask: public ::testing::Test
{
protected:

    testPosturalTask()
    {

    }

    virtual ~testPosturalTask() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testPosturalTask, testPosturalTask_)
{
    yarp::sig::Vector q(6, 0.0);
    for(unsigned int i = 0; i < q.size(); ++i)
        q[i] = tests_utils::getRandomAngle();

    yarp::sig::Vector q_ref(q.size(), 0.0);

    wb_sot::tasks::velocity::Postural postural(q);

    EXPECT_TRUE(postural.getA() == yarp::sig::Matrix(q.size(), q.size()).eye());
    EXPECT_TRUE(postural.getWeight() == yarp::sig::Matrix(q.size(), q.size()).eye());

    EXPECT_TRUE(postural.getConstraints().size() == 0);

    double K = 0.1;
    postural.setAlpha(K);
    EXPECT_DOUBLE_EQ(postural.getAlpha(), K);

    postural.setReference(q_ref);
    postural.update(q);
    EXPECT_TRUE(postural.getb() == (q_ref-q));

    for(unsigned int i = 0; i < 100; ++i)
    {
        postural.update(q);
        q += postural.getAlpha()*postural.getb();
    }

    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR(q[i], q_ref[i], 1E-3);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
