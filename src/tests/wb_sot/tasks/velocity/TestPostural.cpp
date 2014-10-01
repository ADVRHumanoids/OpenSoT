#include <drc_shared/tests_utils.h>
#include <gtest/gtest.h>
#include <wb_sot/tasks/velocity/Postural.h>
#include <wb_sot/constraints/velocity/JointLimits.h>
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

    OpenSoT::tasks::velocity::Postural postural(q);

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

TEST_F(testPosturalTask, testPosturalTaskWithJointLimits_)
{
    iDynUtils idynutils;
    using namespace OpenSoT::tasks::velocity;
    using namespace OpenSoT::constraints::velocity;

    yarp::sig::Vector q(idynutils.coman_iDyn3.getNrOfDOFs(), 0.0);
    yarp::sig::Vector q_next(q);

    for(unsigned int i = 0; i < q.size(); ++i) {
        q[i] = tests_utils::getRandomAngle();
        q_next[i] = tests_utils::getRandomAngle();
        assert((q[i]!=q_next[i]));
    }
    idynutils.updateiDyn3Model(q);

    boost::shared_ptr< Postural::TaskType > postural( new Postural(q) );
    boost::shared_ptr< Postural::ConstraintType > bound(
        new JointLimits(q,
                        idynutils.coman_iDyn3.getJointBoundMax(),
                        idynutils.coman_iDyn3.getJointBoundMin())
    );

    postural->getConstraints().push_back( bound );

    yarp::sig::Vector old_b = postural->getb();
    yarp::sig::Vector old_LowerBound = bound->getLowerBound();
    yarp::sig::Vector old_UpperBound = bound->getUpperBound();
    idynutils.updateiDyn3Model(q_next);
    postural->update(q_next);
    yarp::sig::Vector new_b = postural->getb();
    yarp::sig::Vector new_LowerBound = bound->getLowerBound();
    yarp::sig::Vector new_UpperBound = bound->getUpperBound();

    EXPECT_FALSE(old_b == new_b);
    EXPECT_FALSE(old_LowerBound == new_LowerBound);
    EXPECT_FALSE(old_UpperBound == new_UpperBound);

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
