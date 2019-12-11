#include <gtest/gtest.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/constraints/velocity/CoMVelocity.h>
#include <OpenSoT/constraints/velocity/ConvexHull.h>
#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/solvers/eHQP.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/SubTask.h>


std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman_RBDL.yaml";

std::string _path_to_cfg = robotology_root + relative_path;
XBot::ModelInterface::Ptr _model_ptr;

using namespace OpenSoT::tasks;


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

namespace {

class testAggregatedTask: public ::testing::Test
{
protected:

    std::list< OpenSoT::tasks::Aggregated::TaskPtr > _tasks;
    Eigen::VectorXd q;

    testAggregatedTask()
    {
        q.setRandom(6);

        for(unsigned int i = 0; i < q.size(); ++i)
            q[i] = getRandomAngle();

        _tasks.push_back(Aggregated::TaskPtr(new velocity::Postural(q)));
        _tasks.push_back(Aggregated::TaskPtr(new velocity::Postural(2.*q)));
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
            new OpenSoT::tasks::velocity::Postural(q));
    std::list<Aggregated::TaskPtr> task_list;
    task_list.push_back(postural_in_aggregated);
    task_list.push_back(postural_in_aggregated);

    EXPECT_TRUE(Aggregated(task_list,q.size()).getTaskID() == postural_in_aggregated->getTaskID()+"+"+postural_in_aggregated->getTaskID());
}

TEST_F(testAggregatedTask, testAggregatedTask_)
{

    Aggregated aggregated(_tasks, q.size());
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
    EXPECT_TRUE(aggregated.getWeight() == Eigen::MatrixXd::Identity(q.size()*2, q.size()*2));



   _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

    Eigen::VectorXd q(_model_ptr->getJointNum()); q.setZero(q.size());
    Eigen::VectorXd q_ref(q.size());
    q_ref = M_PI*Eigen::VectorXd::Ones(q.size());

    OpenSoT::tasks::velocity::Postural::Ptr postural_in_aggregated(
            new OpenSoT::tasks::velocity::Postural(q));
    postural_in_aggregated->setReference(q_ref);
    std::list<OpenSoT::Task<Eigen::MatrixXd,Eigen::VectorXd>::TaskPtr> task_list;
    task_list.push_back(postural_in_aggregated);
    OpenSoT::tasks::Aggregated::Ptr aggregated_task(
                new OpenSoT::tasks::Aggregated(task_list, q.size()));
    aggregated_task->setLambda(0.1);
    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
            new OpenSoT::tasks::velocity::Postural(q));
    postural_task->setReference(q_ref);
    postural_task->setLambda(0.1);

    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack stack;
    stack.push_back(postural_task);

    OpenSoT::solvers::eHQP solver(stack);


//1. Here we use postural_task

    Eigen::VectorXd dq(q.size());
    dq.setZero(dq.size());
    for(unsigned int i = 0; i < 1000; ++i)
    {
        postural_task->update(q);
        aggregated_task->update(q);
        EXPECT_TRUE(aggregated_task->getA() == postural_task->getA());
        EXPECT_TRUE(aggregated_task->getb() == postural_task->getb());

        solver.solve(dq);

        q += dq;
    }

    for(unsigned int i = 0; i < q_ref.size(); ++i)
        EXPECT_NEAR(q[i],q_ref[i],1E-4);

    std::cout<<"q_ref: "<<q_ref.transpose()<<std::endl;
    std::cout<<"q    : "<<q.transpose()<<std::endl;



    dq.setZero(dq.size()); q.setZero(q.size());

    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::Stack stack2;
    stack2.push_back(aggregated_task);
    OpenSoT::solvers::eHQP solver2(stack2);


    for(unsigned int i = 0; i < 1000; ++i)
    {
        postural_task->update(q);
        aggregated_task->update(q);
        EXPECT_TRUE(aggregated_task->getA() == postural_task->getA());
        EXPECT_TRUE(aggregated_task->getb() == postural_task->getb()) << "aggregated_task b is " << aggregated_task->getb() << std::endl
                                                                      << " while postural_task b is " << postural_task->getb();

        solver2.solve(dq);

        q += dq;
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

TEST_F(testAggregatedTask, testConstraintsUpdate)
{
    _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

    if(_model_ptr)
        std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
    else
        std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;



    Eigen::VectorXd q(_model_ptr->getJointNum());
    q.setZero(q.size());
    for(unsigned int r = 0; r < 1e3; ++r)
    {
        Eigen::VectorXd qmin(q.size()), qmax(q.size());
        _model_ptr->getJointLimits(qmin, qmax);

        q = getRandomAngles(qmin, qmax, q.size());
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        OpenSoT::tasks::velocity::Postural::Ptr taskPostural(new OpenSoT::tasks::velocity::Postural(q));
        OpenSoT::tasks::velocity::Cartesian::Ptr taskCartesianWaist(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::Waist",
                                                        q,*(_model_ptr.get()), "Waist", "world"));

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
                new OpenSoT::constraints::velocity::ConvexHull(
                        q, *(_model_ptr.get()),
                        _links_in_contact, 0.05));
        Eigen::VectorXd comlims(3);
        comlims<<0.03,0.03,0.03;
        OpenSoT::constraints::velocity::CoMVelocity::Ptr constraintCoMVelocity(
                new OpenSoT::constraints::velocity::CoMVelocity(comlims, 0.01,
                        q, *(_model_ptr.get())));

        taskCartesianWaist->getConstraints().push_back(constraintConvexHull);

        std::list<OpenSoT::tasks::Aggregated::TaskPtr> taskList;
        taskList.push_back(taskPostural);
        OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr _task0(
                new OpenSoT::tasks::Aggregated(taskList, q.size()));
        _task0->getConstraints().push_back(constraintConvexHull);
        EXPECT_EQ(_task0->getConstraints().size(), 1)<<"1"<<std::endl;
        _task0->getConstraints().push_back(constraintCoMVelocity);
        EXPECT_EQ(_task0->getConstraints().size(), 2)<<"2"<<std::endl;
        taskList.clear();

        taskList.push_back(taskCartesianWaist);
        OpenSoT::tasks::Aggregated::Ptr _task1(
                new OpenSoT::tasks::Aggregated(taskList, q.size()));
        EXPECT_EQ(_task1->getConstraints().size(), 1)<<"3.1"<<std::endl;
        _task1->getConstraints().push_back(constraintCoMVelocity);
        EXPECT_EQ(_task1->getConstraints().size(), 2)<<"3.2"<<std::endl;

        _task0->update(q);
        EXPECT_EQ(_task0->getConstraints().size(), 2)<<"4"<<std::endl;
        _task1->update(q);
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



        q = getRandomAngles(qmin, qmax, q.size());
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

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
    _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

    Eigen::VectorXd qmin(_model_ptr->getJointNum()), qmax(_model_ptr->getJointNum());
    _model_ptr->getJointLimits(qmin, qmax);
    q = getRandomAngles(qmin, qmax, qmin.size());
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    OpenSoT::tasks::velocity::Postural::Ptr postural(new OpenSoT::tasks::velocity::Postural(q));
    OpenSoT::tasks::velocity::Cartesian::Ptr waist(
            new OpenSoT::tasks::velocity::Cartesian("cartesian::Waist",
                                                    q,*(_model_ptr.get()), "Waist", "world"));
    OpenSoT::tasks::velocity::Cartesian::Ptr lwrist(
            new OpenSoT::tasks::velocity::Cartesian("cartesian::l_wrist",
                                                    q,*(_model_ptr.get()), "l_wrist", "world"));

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
    t1->update(q);
    std::cout<<"t1->getWeight(): \n"<<t1->getWeight()<<std::endl;
    std::cout<<"waist->getWeight(): \n"<<waist->getWeight()<<std::endl;
    std::cout<<"lwrist->getWeight(): \n"<<lwrist->getWeight()<<std::endl;
    EXPECT_TRUE(matrixAreEqual(waist->getWeight(), t1->getWeight().block(6,6,6,6)));
    EXPECT_TRUE(matrixAreEqual(lwrist->getWeight(), t1->getWeight().block(0,0,6,6)));
    Eigen::MatrixXd W2(12,12); W2.setOnes();
    t1->setWeight(W2);
    t1->update(q);
    std::cout<<"t1->getWeight(): \n"<<t1->getWeight()<<std::endl;
    std::cout<<"waist->getWeight(): \n"<<waist->getWeight()<<std::endl;
    std::cout<<"lwrist->getWeight(): \n"<<lwrist->getWeight()<<std::endl;
    EXPECT_TRUE(matrixAreEqual(waist->getWeight(), t1->getWeight().block(6,6,6,6)));
    EXPECT_TRUE(matrixAreEqual(lwrist->getWeight(), t1->getWeight().block(0,0,6,6)));
    waist->setWeight(10*W);
    lwrist->setWeight(20*W);
    t1->update(q);
    std::cout<<"t1->getWeight(): \n"<<t1->getWeight()<<std::endl;
    std::cout<<"waist->getWeight(): \n"<<waist->getWeight()<<std::endl;
    std::cout<<"lwrist->getWeight(): \n"<<lwrist->getWeight()<<std::endl;
    EXPECT_TRUE(matrixAreEqual(waist->getWeight(), t1->getWeight().block(6,6,6,6)));
    EXPECT_TRUE(matrixAreEqual(lwrist->getWeight(), t1->getWeight().block(0,0,6,6)));

    //2) Multiple aggregates and subtasks
    OpenSoT::tasks::velocity::Cartesian::Ptr rwrist(
            new OpenSoT::tasks::velocity::Cartesian("cartesian::r_wrist",
                                                    q,*(_model_ptr.get()), "r_wrist", "world"));
    W.setOnes(6,6);
    rwrist->setWeight(3.*W);

    std::list<unsigned int> id = {1,2};
    auto s1 = rwrist%id;
    s1->update(q);

    auto t2 = (t1+s1);
    t2->update(q);

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

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
