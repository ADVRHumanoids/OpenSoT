#include <idynutils/tests_utils.h>
#include <gtest/gtest.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <idynutils/idynutils.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/constraints/velocity/CoMVelocity.h>
#include <OpenSoT/constraints/velocity/ConvexHull.h>

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

    EXPECT_TRUE(Aggregated(task_list,q.size()).getTaskID() == postural_in_aggregated->getTaskID()+"+"+postural_in_aggregated->getTaskID());
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

TEST_F(testAggregatedTask, testConstraintsUpdate)
{
    iDynUtils robot;
    yarp::sig::Vector q(robot.iDyn3_model.getNrOfDOFs(), 0.0);
    for(unsigned int i = 0; i < q.size(); ++i)
        q[i] = tests_utils::getRandomAngle();
    robot.updateiDyn3Model(q, true);

    boost::shared_ptr<OpenSoT::tasks::velocity::Postural> taskPostural(new OpenSoT::tasks::velocity::Postural(q));
    boost::shared_ptr<OpenSoT::tasks::velocity::Cartesian> taskCartesianWaist(
            new OpenSoT::tasks::velocity::Cartesian("cartesian::Waist",q,robot, "Waist", "world"));

    boost::shared_ptr<OpenSoT::constraints::velocity::ConvexHull> constraintConvexHull(
            new OpenSoT::constraints::velocity::ConvexHull(q, robot, 0.05));
    boost::shared_ptr<OpenSoT::constraints::velocity::CoMVelocity> constraintCoMVelocity(
            new OpenSoT::constraints::velocity::CoMVelocity(yarp::sig::Vector(3,0.03), 0.01, q, robot));

    taskCartesianWaist->getConstraints().push_back(constraintCoMVelocity);

    std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList;
    taskList.push_back(taskPostural);
    boost::shared_ptr<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector> > _task0(
            new OpenSoT::tasks::Aggregated(taskList, q.size()));
    _task0->getConstraints().push_back(constraintConvexHull);
    EXPECT_EQ(_task0->getConstraints().size(), 1)<<"1"<<std::endl;
    _task0->getConstraints().push_back(constraintCoMVelocity);
    EXPECT_EQ(_task0->getConstraints().size(), 2)<<"2"<<std::endl;
    taskList.clear();

    taskList.push_back(taskCartesianWaist);
    boost::shared_ptr<OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector> > _task1(
            new OpenSoT::tasks::Aggregated(taskList, q.size()));
    EXPECT_EQ(_task1->getConstraints().size(), 1)<<"3.1"<<std::endl;
    _task1->getConstraints().push_back(constraintConvexHull);
    EXPECT_EQ(_task1->getConstraints().size(), 2)<<"3.2"<<std::endl;

    _task0->update(q);
    EXPECT_EQ(_task0->getConstraints().size(), 2)<<"4"<<std::endl;
    _task1->update(q);
    EXPECT_EQ(_task1->getConstraints().size(), 2)<<"5"<<std::endl;

    yarp::sig::Matrix A0 = _task0->getA();
    yarp::sig::Vector b0 = _task0->getb();
    std::list<boost::shared_ptr<OpenSoT::Constraint<yarp::sig::Matrix, yarp::sig::Vector>>> constraints0 =
            _task0->getConstraints();
    ASSERT_EQ(constraints0.size(), 2);
    yarp::sig::Matrix Aineq0_ch = constraints0.front()->getAineq();
    yarp::sig::Vector lA0_ch = constraints0.front()->getbLowerBound();
    yarp::sig::Vector uA0_ch = constraints0.front()->getbUpperBound();
    yarp::sig::Matrix Aineq0_comvel = constraints0.back()->getAineq();
    yarp::sig::Vector lA0_comvel = constraints0.back()->getbLowerBound();
    yarp::sig::Vector uA0_comvel = constraints0.back()->getbUpperBound();

    yarp::sig::Matrix A1 = _task1->getA();
    yarp::sig::Vector b1 = _task1->getb();
    std::list<boost::shared_ptr<OpenSoT::Constraint<yarp::sig::Matrix, yarp::sig::Vector>>> constraints1 =
            _task1->getConstraints();
    yarp::sig::Matrix Aineq1_ch = constraints1.front()->getAineq();
    yarp::sig::Vector lA1_ch = constraints1.front()->getbLowerBound();
    yarp::sig::Vector uA1_ch = constraints1.front()->getbUpperBound();
    yarp::sig::Matrix Aineq1_comvel = constraints1.back()->getAineq();
    yarp::sig::Vector lA1_comvel = constraints1.back()->getbLowerBound();
    yarp::sig::Vector uA1_comvel = constraints1.back()->getbUpperBound();

    EXPECT_TRUE(Aineq0_ch == Aineq1_ch);
    EXPECT_TRUE(Aineq0_comvel == Aineq1_comvel);
    EXPECT_TRUE(lA1_ch == lA0_ch);
    EXPECT_TRUE(uA1_ch == uA0_ch);
    EXPECT_TRUE(lA1_comvel == lA0_comvel);
    EXPECT_TRUE(uA1_comvel == uA0_comvel);

    for(unsigned int i = 0; i < q.size(); ++i)
        q[i] = tests_utils::getRandomAngle();
    robot.updateiDyn3Model(q, true);

    _task0->update(q);
    EXPECT_EQ(_task0->getConstraints().size(), 2)<<"6"<<std::endl;
    _task1->update(q);
    EXPECT_EQ(_task1->getConstraints().size(), 2)<<"7"<<std::endl;

    A0 = _task0->getA();
    b0 = _task0->getb();
    constraints0 = _task0->getConstraints();
    ASSERT_EQ(constraints0.size(), 2);
    Aineq0_ch = constraints0.front()->getAineq();
    lA0_ch = constraints0.front()->getbLowerBound();
    uA0_ch = constraints0.front()->getbUpperBound();
    Aineq0_comvel = constraints0.back()->getAineq();
    lA0_comvel = constraints0.back()->getbLowerBound();
    uA0_comvel = constraints0.back()->getbUpperBound();

    A1 = _task1->getA();
    b1 = _task1->getb();
    constraints1 = _task1->getConstraints();
    Aineq1_ch = constraints1.front()->getAineq();
    lA1_ch = constraints1.front()->getbLowerBound();
    uA1_ch = constraints1.front()->getbUpperBound();
    Aineq1_comvel = constraints1.back()->getAineq();
    lA1_comvel = constraints1.back()->getbLowerBound();
    uA1_comvel = constraints1.back()->getbUpperBound();

    EXPECT_TRUE(Aineq0_ch == Aineq1_ch);
    EXPECT_TRUE(Aineq0_comvel == Aineq1_comvel);
    EXPECT_TRUE(lA1_ch == lA0_ch);
    EXPECT_TRUE(uA1_ch == uA0_ch);
    EXPECT_TRUE(lA1_comvel == lA0_comvel);
    EXPECT_TRUE(uA1_comvel == uA0_comvel);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
