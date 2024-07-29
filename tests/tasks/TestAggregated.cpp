#include <gtest/gtest.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/constraints/velocity/CartesianVelocity.h>
#include <OpenSoT/constraints/velocity/ConvexHull.h>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/solvers/eHQP.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/SubTask.h>

#include "../common.h"


using namespace OpenSoT::tasks;



namespace {

class testAggregatedTask: public TestBase
{
protected:

    std::list< OpenSoT::tasks::Aggregated::TaskPtr > _tasks;
    Eigen::VectorXd q;

    testAggregatedTask():
        TestBase("coman_floating_base")
    {
        q = _model_ptr->generateRandomQ();
        _model_ptr->setJointPosition(q);
        _model_ptr->update();
        _tasks.push_back(Aggregated::TaskPtr(new velocity::Postural(*_model_ptr)));

        auto q1 = _model_ptr->generateRandomQ();
        _model_ptr->setJointPosition(q1);
        _model_ptr->update();

        _tasks.push_back(Aggregated::TaskPtr(new velocity::Postural(*_model_ptr)));
    }

    virtual ~testAggregatedTask() {
        _tasks.clear();
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testAggregatedTask, testConcatenateTaskIds)
{
    OpenSoT::tasks::velocity::Postural::Ptr postural_in_aggregated(
            new OpenSoT::tasks::velocity::Postural(*_model_ptr));
    std::list<Aggregated::TaskPtr> task_list;
    task_list.push_back(postural_in_aggregated);
    task_list.push_back(postural_in_aggregated);

    EXPECT_TRUE(Aggregated(task_list,_model_ptr->getNv()).getTaskID() == postural_in_aggregated->getTaskID()+"+"+postural_in_aggregated->getTaskID());
}

TEST_F(testAggregatedTask, testAggregatedTask_)
{

    Aggregated aggregated(_tasks, _model_ptr->getNv());
    Eigen::MatrixXd posturalAOne, posturalATwo;
    Eigen::VectorXd posturalbOne, posturalbTwo;

    posturalAOne = (*(_tasks.begin()))->getA();
    posturalATwo = (*(++_tasks.begin()))->getA();
    posturalbOne = (*(_tasks.begin()))->getb();
    posturalbTwo = (*(++_tasks.begin()))->getb();

    Eigen::MatrixXd PileA(posturalAOne.rows() + posturalATwo.rows(),posturalAOne.cols());
    PileA<<posturalAOne,
           posturalATwo;
    Eigen::VectorXd Pileb(posturalbOne.size() + posturalbTwo.size());
    Pileb<<posturalbOne,
           posturalbTwo;

    EXPECT_TRUE(aggregated.getA() == PileA);
    EXPECT_TRUE(aggregated.getb() == Pileb);

    EXPECT_TRUE(aggregated.getConstraints().size() == 0);

    double K = 0.1;
    aggregated.setLambda(K);
    EXPECT_DOUBLE_EQ(aggregated.getLambda(), K);

    std::cout<<"aggregated.getWeight(): "<<aggregated.getWeight()<<std::endl;
    EXPECT_TRUE(aggregated.getWeight() == Eigen::MatrixXd::Identity(_model_ptr->getNv()*2, _model_ptr->getNv()*2));


    Eigen::VectorXd q = _model_ptr->getNeutralQ();
    Eigen::VectorXd q_ref = _model_ptr->generateRandomQ();
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::tasks::velocity::Postural::Ptr postural_in_aggregated(
            new OpenSoT::tasks::velocity::Postural(*_model_ptr));
    postural_in_aggregated->setReference(q_ref);
    std::list<OpenSoT::Task<Eigen::MatrixXd,Eigen::VectorXd>::TaskPtr> task_list;
    task_list.push_back(postural_in_aggregated);
    OpenSoT::tasks::Aggregated::Ptr aggregated_task(
                new OpenSoT::tasks::Aggregated(task_list, _model_ptr->getNv()));
    aggregated_task->setLambda(0.1);
    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
            new OpenSoT::tasks::velocity::Postural(*_model_ptr));
    postural_task->setReference(q_ref);
    postural_task->setLambda(0.1);

    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack stack;
    stack.push_back(postural_task);

    OpenSoT::solvers::eHQP solver(stack);


//1. Here we use postural_task

    Eigen::VectorXd dq(_model_ptr->getNv());
    dq.setZero();
    for(unsigned int i = 0; i < 1000; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        postural_task->update();
        aggregated_task->update();
        EXPECT_TRUE(aggregated_task->getA() == postural_task->getA());
        EXPECT_TRUE(aggregated_task->getb() == postural_task->getb());

        EXPECT_TRUE(solver.solve(dq));

        q = _model_ptr->sum(q, dq);
    }

    for(unsigned int i = 0; i < q_ref.size(); ++i)
        EXPECT_NEAR(q[i],q_ref[i],1E-4);

    std::cout<<"q_ref: "<<q_ref.transpose()<<std::endl;
    std::cout<<"q    : "<<q.transpose()<<std::endl;

    dq.setZero(); q = _model_ptr->getNeutralQ();
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack stack2;
    stack2.push_back(aggregated_task);
    OpenSoT::solvers::eHQP solver2(stack2);


    for(unsigned int i = 0; i < 1000; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        postural_task->update();
        aggregated_task->update();
        EXPECT_TRUE(aggregated_task->getA() == postural_task->getA());
        EXPECT_TRUE(aggregated_task->getb() == postural_task->getb()) << "aggregated_task b is " << aggregated_task->getb() << std::endl
                                                                      << " while postural_task b is " << postural_task->getb();

        solver2.solve(dq);

        q = _model_ptr->sum(q, dq);
    }

    for(unsigned int i = 0; i < q_ref.size(); ++i)
        EXPECT_NEAR(q[i],q_ref[i],1E-4);

    std::cout<<"q_ref: "<<q_ref.transpose()<<std::endl;
    std::cout<<"q    : "<<q.transpose()<<std::endl;
}

bool matrixAreEqual(const Eigen::MatrixXd& m0,
                                  const Eigen::MatrixXd& m1)
{
    bool sizeAreCompatible = (m0.rows() == m1.rows() &&
                              m0.cols() == m1.cols());
    EXPECT_TRUE(sizeAreCompatible) << "Size of compared matrices "
                                   << "are not compatible";
    if(!sizeAreCompatible)
        return false;

    bool areEqual = true;
    for(unsigned int r = 0; r < m0.rows(); ++r)
        for(unsigned int c = 0; c < m0.cols(); ++c) {
            EXPECT_DOUBLE_EQ(m0(r,c), m1(r,c)) << "Elements in ("
                                               << r << "," << c
                                               << ") are not equal";

            using namespace testing::internal;
            bool elementAreEqual;
            FloatingPoint<double> lhs(m0(r,c));
            FloatingPoint<double> rhs(m1(r,c));
            elementAreEqual = lhs.AlmostEquals(rhs);

            areEqual = areEqual & elementAreEqual;

        }
    return areEqual;
}

bool vectorAreEqual(const Eigen::VectorXd& v0,
                                  const Eigen::VectorXd& v1)
{
    bool sizeAreCompatible = (v0.size() == v1.size());
    EXPECT_TRUE(sizeAreCompatible) << "Size of compared vectors "
                                   << "are not equal";
    if(!sizeAreCompatible)
        return false;

    bool areEqual = true;
    for(unsigned int s = 0; s < v0.size(); ++s) {
        EXPECT_DOUBLE_EQ(v0(s), v1(s)) << "Elements in  ("
                                           << s
                                           << ") are not equal";

        using namespace testing::internal;
        bool elementAreEqual;
        FloatingPoint<double> lhs(v0(s));
        FloatingPoint<double> rhs(v1(s));
        elementAreEqual = lhs.AlmostEquals(rhs);

        areEqual = areEqual & elementAreEqual;

    }
    return areEqual;
}

TEST_F(testAggregatedTask, testAggregatedCost)
{

    Eigen::VectorXd q = _model_ptr->getNeutralQ();

    Eigen::VectorXd qmin, qmax;
    _model_ptr->getJointLimits(qmin, qmax);

    q = _model_ptr->generateRandomQ();
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::tasks::velocity::Cartesian::Ptr Cartesian1(
            new OpenSoT::tasks::velocity::Cartesian("cartesian::r_sole",
                                                    *_model_ptr.get(), "r_sole", "world"));

    OpenSoT::tasks::velocity::Cartesian::Ptr Cartesian2(
            new OpenSoT::tasks::velocity::Cartesian("cartesian::l_sole",
                                                    *_model_ptr.get(), "l_sole", "world"));

    auto ACartesian = Cartesian1 + Cartesian2;


    Eigen::VectorXd dq(_model_ptr->getNv());
    dq.setRandom();

    Cartesian1->update();
    Cartesian2->update();

    double Cartesian1_cost = Cartesian1->computeCost(dq);
    double Cartesian2_cost = Cartesian2->computeCost(dq);
    double ACartesian_cost = ACartesian->computeCost(dq);

    std::cout<<"Cartesian1 cost: "<<Cartesian1_cost<<std::endl;
    std::cout<<"Cartesian2 cost: "<<Cartesian2_cost<<std::endl;
    std::cout<<"ACartesian cost: "<<ACartesian_cost<<std::endl;

    EXPECT_DOUBLE_EQ(ACartesian_cost, Cartesian1_cost + Cartesian2_cost);

}

TEST_F(testAggregatedTask, testConstraintsUpdate)
{
    Eigen::VectorXd q = _model_ptr->getNeutralQ();
    for(unsigned int r = 0; r < 1e3; ++r)
    {
        Eigen::VectorXd qmin, qmax;
        _model_ptr->getJointLimits(qmin, qmax);

        q = _model_ptr->generateRandomQ();
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        OpenSoT::tasks::velocity::Postural::Ptr taskPostural(new OpenSoT::tasks::velocity::Postural(*_model_ptr));
        OpenSoT::tasks::velocity::Cartesian::Ptr taskCartesianWaist(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::Waist", *_model_ptr, "Waist", "world"));

        std::list<std::string> _links_in_contact;
        _links_in_contact.push_back("l_foot_lower_left_link");
        _links_in_contact.push_back("l_foot_lower_right_link");
        _links_in_contact.push_back("l_foot_upper_left_link");
        _links_in_contact.push_back("l_foot_upper_right_link");
        _links_in_contact.push_back("r_foot_lower_left_link");
        _links_in_contact.push_back("r_foot_lower_right_link");
        _links_in_contact.push_back("r_foot_upper_left_link");
        _links_in_contact.push_back("r_foot_upper_right_link");

        OpenSoT::constraints::velocity::ConvexHull::Ptr constraintConvexHull(
                new OpenSoT::constraints::velocity::ConvexHull(*_model_ptr, _links_in_contact, 0.05));
        Eigen::VectorXd comlims(3);
        comlims<<0.03,0.03,0.03;
        OpenSoT::constraints::velocity::CartesianVelocity::Ptr constraintCoMVelocity(
                new OpenSoT::constraints::velocity::CartesianVelocity(comlims, 0.01,
                        std::make_shared<OpenSoT::tasks::velocity::CoM>(*_model_ptr)));

        taskCartesianWaist->getConstraints().push_back(constraintConvexHull);

        std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList;
        taskList.push_back(taskPostural);
        OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr _task0(
                new OpenSoT::tasks::Aggregated(taskList, _model_ptr->getNv()));
        _task0->getConstraints().push_back(constraintConvexHull);
        EXPECT_EQ(_task0->getConstraints().size(), 1)<<"1"<<std::endl;
        _task0->getConstraints().push_back(constraintCoMVelocity);
        EXPECT_EQ(_task0->getConstraints().size(), 2)<<"2"<<std::endl;
        taskList.clear();

        taskList.push_back(taskCartesianWaist);
        OpenSoT::tasks::Aggregated::Ptr _task1(
                new OpenSoT::tasks::Aggregated(taskList, _model_ptr->getNv()));
        EXPECT_EQ(_task1->getConstraints().size(), 1)<<"3.1"<<std::endl;
        _task1->getConstraints().push_back(constraintCoMVelocity);
        EXPECT_EQ(_task1->getConstraints().size(), 2)<<"3.2"<<std::endl;

        _task0->update();
        EXPECT_EQ(_task0->getConstraints().size(), 2)<<"4"<<std::endl;
        _task1->update();
        EXPECT_EQ(_task1->getConstraints().size(), 2)<<"5"<<std::endl;
        ASSERT_EQ(_task1->getOwnConstraints().size(), 1);
        ASSERT_EQ(_task1->getAggregatedConstraints().size(), 1);

        Eigen::MatrixXd A0 = _task0->getA();
        Eigen::VectorXd b0 = _task0->getb();
        std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> constraints0 =
                _task0->getConstraints();
        ASSERT_EQ(constraints0.size(), 2);
        Eigen::MatrixXd Aineq0_ch = constraints0.front()->getAineq();
        Eigen::VectorXd lA0_ch = constraints0.front()->getbLowerBound();
        Eigen::VectorXd uA0_ch = constraints0.front()->getbUpperBound();
        Eigen::MatrixXd Aineq0_comvel = constraints0.back()->getAineq();
        Eigen::VectorXd lA0_comvel = constraints0.back()->getbLowerBound();
        Eigen::VectorXd uA0_comvel = constraints0.back()->getbUpperBound();

        Eigen::MatrixXd A1 = _task1->getA();
        Eigen::VectorXd b1 = _task1->getb();
        std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> constraints1 =
                _task1->getConstraints();
        Eigen::MatrixXd Aineq1_ch = constraints1.front()->getAineq();
        Eigen::VectorXd lA1_ch = constraints1.front()->getbLowerBound();
        Eigen::VectorXd uA1_ch = constraints1.front()->getbUpperBound();
        Eigen::MatrixXd Aineq1_comvel = constraints1.back()->getAineq();
        Eigen::VectorXd lA1_comvel = constraints1.back()->getbLowerBound();
        Eigen::VectorXd uA1_comvel = constraints1.back()->getbUpperBound();

        ASSERT_TRUE(matrixAreEqual(Aineq0_ch,
                                                Aineq1_ch)) << "@r " << r << ": "
                                                            << "Aineq0 and Aineq1 "
                                                            << "are not equal: "
                                                            << "Aineq0_ch:" << std::endl
                                                            << Aineq0_ch << std::endl
                                                            << "Aineq1_ch:" << std::endl
                                                            << Aineq1_ch << std::endl;

        EXPECT_TRUE(matrixAreEqual(Aineq0_comvel,
                                   Aineq1_comvel)) << "Aineq0_comvel and Aineq1_comvel "
                                                   << "are not equal";

        EXPECT_TRUE(vectorAreEqual(lA0_ch,
                                   lA1_ch))    << "lA0_ch and lA1_ch "
                                               << "are not equal";

        EXPECT_TRUE(vectorAreEqual(uA0_ch,
                                   uA1_ch))    << "uA0_ch and uA1_ch "
                                               << "are not equal";

        EXPECT_TRUE(vectorAreEqual(lA0_comvel,
                                   lA1_comvel))    << "lA0_comvel and lA1_comvel "
                                                   << "are not equal";

        EXPECT_TRUE(vectorAreEqual(uA0_comvel,
                                                uA1_comvel))    << "uA0_comvel and uA1_comvel "
                                                                << "are not equal";



        q = _model_ptr->generateRandomQ();
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        _task0->update();
        EXPECT_EQ(_task0->getConstraints().size(), 2)<<"6"<<std::endl;
        _task1->update();
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
        ASSERT_EQ(constraints1.size(), 2);
        Aineq1_ch = constraints1.front()->getAineq();
        lA1_ch = constraints1.front()->getbLowerBound();
        uA1_ch = constraints1.front()->getbUpperBound();
        Aineq1_comvel = constraints1.back()->getAineq();
        lA1_comvel = constraints1.back()->getbLowerBound();
        uA1_comvel = constraints1.back()->getbUpperBound();


        ASSERT_TRUE(matrixAreEqual(Aineq0_ch,
                                   Aineq1_ch)) << "Aineq0 and Aineq1 "
                                               << "are not equal: "
                                               << "Aineq0_ch:" << std::endl
                                               << Aineq0_ch << std::endl
                                               << "Aineq1_ch:" << std::endl
                                               << Aineq1_ch << std::endl;

        EXPECT_TRUE(matrixAreEqual(Aineq0_comvel,
                                   Aineq1_comvel)) << "Aineq0_comvel and Aineq1_comvel "
                                                   << "are not equal";

        EXPECT_TRUE(vectorAreEqual(lA0_ch,
                                   lA1_ch))    << "lA0_ch and lA1_ch "
                                               << "are not equal";

        EXPECT_TRUE(vectorAreEqual(uA0_ch,
                                   uA1_ch))    << "uA0_ch and uA1_ch "
                                               << "are not equal";

        EXPECT_TRUE(vectorAreEqual(lA0_comvel,
                                   lA1_comvel))    << "lA0_comvel and lA1_comvel "
                                                   << "are not equal";

        EXPECT_TRUE(vectorAreEqual(uA0_comvel,
                                                uA1_comvel))    << "uA0_comvel and uA1_comvel "
                                                                << "are not equal";
    }
}

TEST_F(testAggregatedTask, testWeightsUpdate)
{
    Eigen::VectorXd qmin, qmax;
    _model_ptr->getJointLimits(qmin, qmax);
    q = _model_ptr->generateRandomQ();
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::tasks::velocity::Postural::Ptr postural(new OpenSoT::tasks::velocity::Postural(*_model_ptr));
    OpenSoT::tasks::velocity::Cartesian::Ptr waist(
            new OpenSoT::tasks::velocity::Cartesian("cartesian::Waist",
                                                    *_model_ptr, "Waist", "world"));
    OpenSoT::tasks::velocity::Cartesian::Ptr lwrist(
            new OpenSoT::tasks::velocity::Cartesian("cartesian::l_wrist",
                                                    *_model_ptr, "l_wrist", "world"));

    //1) Test that aggregated weights work
    Eigen::MatrixXd W(6,6); W.setIdentity(); W *=3.;
    W*waist;
    EXPECT_TRUE(matrixAreEqual(W, waist->getWeight()));
    std::cout<<"W: \n"<<W<<std::endl;
    std::cout<<"waist->getWeight(): \n"<<waist->getWeight()<<std::endl;
    auto t1 = (lwrist + waist);
    std::cout<<"t1->getWeight(): \n"<<t1->getWeight()<<std::endl;
    std::cout<<"waist->getWeight(): \n"<<waist->getWeight()<<std::endl;
    std::cout<<"lwrist->getWeight(): \n"<<lwrist->getWeight()<<std::endl;
    EXPECT_TRUE(matrixAreEqual(waist->getWeight(), t1->getWeight().block(6,6,6,6)));
    EXPECT_TRUE(matrixAreEqual(lwrist->getWeight(), t1->getWeight().block(0,0,6,6)));
    W = Eigen::MatrixXd::Identity(W.rows(), W.cols());
    waist->setWeight(10*W);
    lwrist->setWeight(20*W);
    t1->update();
    std::cout<<"t1->getWeight(): \n"<<t1->getWeight()<<std::endl;
    std::cout<<"waist->getWeight(): \n"<<waist->getWeight()<<std::endl;
    std::cout<<"lwrist->getWeight(): \n"<<lwrist->getWeight()<<std::endl;
    EXPECT_TRUE(matrixAreEqual(waist->getWeight(), t1->getWeight().block(6,6,6,6)));
    EXPECT_TRUE(matrixAreEqual(lwrist->getWeight(), t1->getWeight().block(0,0,6,6)));
    Eigen::MatrixXd W2(12,12); W2.setOnes();
    t1->setWeight(W2);
    t1->update();
    std::cout<<"t1->getWeight(): \n"<<t1->getWeight()<<std::endl;
    std::cout<<"waist->getWeight(): \n"<<waist->getWeight()<<std::endl;
    std::cout<<"lwrist->getWeight(): \n"<<lwrist->getWeight()<<std::endl;
    EXPECT_TRUE(matrixAreEqual(waist->getWeight(), t1->getWeight().block(6,6,6,6)));
    EXPECT_TRUE(matrixAreEqual(lwrist->getWeight(), t1->getWeight().block(0,0,6,6)));
    waist->setWeight(10*W);
    lwrist->setWeight(20*W);
    t1->update();
    std::cout<<"t1->getWeight(): \n"<<t1->getWeight()<<std::endl;
    std::cout<<"waist->getWeight(): \n"<<waist->getWeight()<<std::endl;
    std::cout<<"lwrist->getWeight(): \n"<<lwrist->getWeight()<<std::endl;
    EXPECT_TRUE(matrixAreEqual(waist->getWeight(), t1->getWeight().block(6,6,6,6)));
    EXPECT_TRUE(matrixAreEqual(lwrist->getWeight(), t1->getWeight().block(0,0,6,6)));

    //2) Multiple aggregates and subtasks
    OpenSoT::tasks::velocity::Cartesian::Ptr rwrist(
            new OpenSoT::tasks::velocity::Cartesian("cartesian::r_wrist",
                                                    *_model_ptr, "r_wrist", "world"));
    W.setOnes(6,6);
    rwrist->setWeight(3.*W);

    std::list<unsigned int> id = {1,2};
    auto s1 = rwrist%id;
    s1->update();

    auto t2 = (t1+s1);
    t2->update();

    std::cout<<"t1->getWeight(): \n"<<t1->getWeight()<<std::endl;
    std::cout<<"waist->getWeight(): \n"<<waist->getWeight()<<std::endl;
    std::cout<<"lwrist->getWeight(): \n"<<lwrist->getWeight()<<std::endl;
    std::cout<<"s1->getWeight(): \n"<<s1->getWeight()<<std::endl;
    std::cout<<"rwrist->getWeight(): \n"<<rwrist->getWeight()<<std::endl;
    std::cout<<"t2->getWeight(): \n"<<t2->getWeight()<<std::endl;

    EXPECT_TRUE(matrixAreEqual(t2->getWeight().block(0,0,6,6), lwrist->getWeight()));
    EXPECT_TRUE(matrixAreEqual(t2->getWeight().block(6,6,6,6), waist->getWeight()));
    EXPECT_TRUE(matrixAreEqual(t2->getWeight().block(12,12,2,2), s1->getWeight()));
    EXPECT_TRUE(matrixAreEqual(t2->getWeight().block(12,12,2,2), rwrist->getWeight().block(1,1,2,2)));

    Eigen::MatrixXd W3(14,14);
    W3.setIdentity();
    t2->setWeight(W3);
    std::cout<<"t1->getWeight(): \n"<<t1->getWeight()<<std::endl;
    std::cout<<"waist->getWeight(): \n"<<waist->getWeight()<<std::endl;
    std::cout<<"lwrist->getWeight(): \n"<<lwrist->getWeight()<<std::endl;
    std::cout<<"s1->getWeight(): \n"<<s1->getWeight()<<std::endl;
    std::cout<<"rwrist->getWeight(): \n"<<rwrist->getWeight()<<std::endl;
    std::cout<<"t2->getWeight(): \n"<<t2->getWeight()<<std::endl;

    EXPECT_TRUE(matrixAreEqual(t2->getWeight().block(0,0,6,6), lwrist->getWeight()));
    EXPECT_TRUE(matrixAreEqual(t2->getWeight().block(6,6,6,6), waist->getWeight()));
    EXPECT_TRUE(matrixAreEqual(t2->getWeight().block(12,12,2,2), s1->getWeight()));
    EXPECT_TRUE(matrixAreEqual(t2->getWeight().block(12,12,2,2), rwrist->getWeight().block(1,1,2,2)));

}

TEST_F(testAggregatedTask, testSingleTask)
{

    Eigen::VectorXd qmin, qmax;
    _model_ptr->getJointLimits(qmin, qmax);
    q = _model_ptr->generateRandomQ();
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::tasks::velocity::Cartesian::Ptr waist(
            new OpenSoT::tasks::velocity::Cartesian("cartesian::Waist",
                                                    *_model_ptr, "Waist", "world"));

    OpenSoT::tasks::Aggregated::Ptr aggr = std::make_shared<OpenSoT::tasks::Aggregated>(waist, _model_ptr->getNv());

    aggr->update();

    EXPECT_TRUE(aggr->getA() == waist->getA());
    EXPECT_TRUE(aggr->getb() == waist->getb());
    EXPECT_TRUE(aggr->getWeight() == waist->getWeight());
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
