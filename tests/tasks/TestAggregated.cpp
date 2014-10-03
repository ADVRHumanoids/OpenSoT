#include <drc_shared/tests_utils.h>
#include <gtest/gtest.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <yarp/math/Math.h>

using namespace yarp::math;
using namespace OpenSoT::tasks;

namespace {

class testAggregatedTask: public ::testing::Test
{
protected:

    std::list< boost::shared_ptr<Aggregated::TaskType> > _tasks;
    yarp::sig::Vector q;

    testAggregatedTask()
    {
        q.resize(6,0.0);

        for(unsigned int i = 0; i < q.size(); ++i)
            q[i] = tests_utils::getRandomAngle();

        _tasks.push_back(boost::shared_ptr<Aggregated::TaskType>(new velocity::Postural(q)));
        _tasks.push_back(boost::shared_ptr<Aggregated::TaskType>(new velocity::Postural(2*q)));
    }

    virtual ~testAggregatedTask() {
        /*
        for(std::list< boost::shared_ptr<Aggregated::TaskType> >::iterator i = _tasks.begin();
            i!=_tasks.end();
            ++i) {
            boost::shared_ptr<Aggregated::TaskType> t = *i;
            delete t;
        } */
        _tasks.clear();
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testAggregatedTask, testAggregatedTask_)
{

    Aggregated aggregated(_tasks, q.size());
    yarp::sig::Matrix posturalAOne, posturalATwo;
    yarp::sig::Vector posturalbOne, posturalbTwo;

    posturalAOne = (*(_tasks.begin()))->getA();
    posturalATwo = (*(++_tasks.begin()))->getA();
    posturalbOne = (*(_tasks.begin()))->getb();
    posturalbTwo = (*(++_tasks.begin()))->getb();

    EXPECT_TRUE(aggregated.getA() == yarp::math::pile(posturalAOne,posturalATwo));
    EXPECT_TRUE(aggregated.getb() == yarp::math::cat(posturalbOne,posturalbTwo));

    EXPECT_TRUE(aggregated.getConstraints().size() == 0);

    double K = 0.1;
    aggregated.setLambda(K);
    EXPECT_DOUBLE_EQ(aggregated.getLambda(), K);

    EXPECT_TRUE(aggregated.getWeight() == yarp::sig::Matrix(q.size()*2, q.size()*2).eye());
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
