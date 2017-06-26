#include <advr_humanoids_common_utils/test_utils.h>
#include <advr_humanoids_common_utils/idynutils.h>
#include <advr_humanoids_common_utils/conversion_utils_YARP.h>
#include <gtest/gtest.h>
#include <yarp/math/Math.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>

using namespace yarp::math;

typedef idynutils2 iDynUtils;

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

    OpenSoT::tasks::velocity::Postural postural(conversion_utils_YARP::toEigen(q));
    std::cout<<"Postural Task Inited"<<std::endl;
    EXPECT_TRUE(postural.getA() == conversion_utils_YARP::toEigen(yarp::sig::Matrix(q.size(), q.size()).eye()));
    EXPECT_TRUE(postural.getWeight() == conversion_utils_YARP::toEigen(yarp::sig::Matrix(q.size(), q.size()).eye()));
    EXPECT_TRUE(postural.getConstraints().size() == 0);

    double K = 0.1;
    postural.setLambda(K);
    EXPECT_DOUBLE_EQ(postural.getLambda(), K);

    postural.setReference(conversion_utils_YARP::toEigen(q_ref));
    postural.update(conversion_utils_YARP::toEigen(q));
    EXPECT_TRUE(postural.getb() == conversion_utils_YARP::toEigen(postural.getLambda()*(q_ref-q)));

    for(unsigned int i = 0; i < 100; ++i)
    {
        postural.update(conversion_utils_YARP::toEigen(q));
        yarp::sig::Vector qq = conversion_utils_YARP::toYARP(postural.getb());
        q += qq;
    }

    for(unsigned int i = 0; i < q.size(); ++i)
        EXPECT_NEAR(q[i], q_ref[i], 1E-3);
}

TEST_F(testPosturalTask, testPosturalTaskWithJointLimits_)
{
    iDynUtils idynutils("coman",
                        std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                        std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");

    yarp::sig::Vector q(idynutils.iDynTree_model.getNrOfDOFs(), 0.0);
    yarp::sig::Vector q_next(q);

    for(unsigned int i = 0; i < q.size(); ++i) {
        q[i] = tests_utils::getRandomAngle();
        q_next[i] = tests_utils::getRandomAngle();
        assert((q[i]!=q_next[i]));
    }
    idynutils.updateiDynTreeModel(conversion_utils_YARP::toEigen(q));

    OpenSoT::tasks::velocity::Postural::TaskPtr postural( new OpenSoT::tasks::velocity::Postural(conversion_utils_YARP::toEigen(q)) );
    OpenSoT::tasks::velocity::Postural::ConstraintPtr bound(
        new OpenSoT::constraints::velocity::JointLimits(conversion_utils_YARP::toEigen(q),
                        idynutils.getJointBoundMax(),
                        idynutils.getJointBoundMin())
    );

    postural->getConstraints().push_back( bound );

    yarp::sig::Vector old_b = conversion_utils_YARP::toYARP(postural->getb());
    yarp::sig::Vector old_LowerBound = conversion_utils_YARP::toYARP(bound->getLowerBound());
    yarp::sig::Vector old_UpperBound = conversion_utils_YARP::toYARP(bound->getUpperBound());
    idynutils.updateiDynTreeModel(conversion_utils_YARP::toEigen(q_next));
    postural->update(conversion_utils_YARP::toEigen(q_next));
    yarp::sig::Vector new_b = conversion_utils_YARP::toYARP(postural->getb());
    yarp::sig::Vector new_LowerBound = conversion_utils_YARP::toYARP(bound->getLowerBound());
    yarp::sig::Vector new_UpperBound = conversion_utils_YARP::toYARP(bound->getUpperBound());

    EXPECT_FALSE(old_b == new_b);
    EXPECT_FALSE(old_LowerBound == new_LowerBound);
    EXPECT_FALSE(old_UpperBound == new_UpperBound);

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
