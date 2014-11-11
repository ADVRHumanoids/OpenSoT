#include <idynutils/tests_utils.h>
#include <gtest/gtest.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

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

TEST_F(testAggregatedTask, testConcatenateTaskIds)
{
    boost::shared_ptr<OpenSoT::tasks::velocity::Postural> postural_in_aggregated(
            new OpenSoT::tasks::velocity::Postural(q));
    std::list<OpenSoT::Task<yarp::sig::Matrix,yarp::sig::Vector>::TaskPtr> task_list;
    task_list.push_back(postural_in_aggregated);
    task_list.push_back(postural_in_aggregated);

    EXPECT_TRUE(Aggregated::concatenateTaskIds(task_list) == postural_in_aggregated->getTaskID()+"+"+postural_in_aggregated->getTaskID());
}

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

    iDynUtils idynutils;
    yarp::sig::Vector q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
    idynutils.updateiDyn3Model(q, true);
    yarp::sig::Vector q_ref(q.size(), M_PI);

    boost::shared_ptr<OpenSoT::tasks::velocity::Postural> postural_in_aggregated(
            new OpenSoT::tasks::velocity::Postural(q));
    postural_in_aggregated->setReference(q_ref);
    std::list<OpenSoT::Task<yarp::sig::Matrix,yarp::sig::Vector>::TaskPtr> task_list;
    task_list.push_back(postural_in_aggregated);
    boost::shared_ptr<OpenSoT::tasks::Aggregated> aggregated_task(
                new OpenSoT::tasks::Aggregated(task_list, q.size()));
    aggregated_task->setLambda(0.1);
    boost::shared_ptr<OpenSoT::tasks::velocity::Postural> postural_task(
            new OpenSoT::tasks::velocity::Postural(q));
    postural_task->setReference(q_ref);
    postural_task->setLambda(0.1);

//1. Here we use postural_task

    yarp::sig::Vector dq(q.size(), 0.0);
    for(unsigned int i = 0; i < 1000; ++i)
    {
        postural_task->update(q);
        aggregated_task->update(q);
        EXPECT_TRUE(aggregated_task->getA() == postural_task->getA());
        EXPECT_TRUE(aggregated_task->getb() == postural_task->getb());
        dq = pinv(postural_task->getA(),1E-7)*postural_task->getLambda()*postural_task->getb();
        q += dq;
    }

    for(unsigned int i = 0; i < q_ref.size(); ++i)
        EXPECT_NEAR(q[i],q_ref[i],1E-4);

    dq.zero(); q.zero();

    for(unsigned int i = 0; i < 1000; ++i)
    {
        postural_task->update(q);
        aggregated_task->update(q);
        EXPECT_TRUE(aggregated_task->getA() == postural_task->getA());
        EXPECT_TRUE(aggregated_task->getb() == postural_task->getb()) << "aggregated_task b is " << aggregated_task->getb().toString() << std::endl
                                                                      << " while postural_task b is " << postural_task->getb().toString();
        dq = pinv(aggregated_task->getA(),1E-7)*aggregated_task->getLambda()*aggregated_task->getb();
        q += dq;
    }

    for(unsigned int i = 0; i < q_ref.size(); ++i)
        EXPECT_NEAR(q[i],q_ref[i],1E-4);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
