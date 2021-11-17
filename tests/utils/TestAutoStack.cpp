#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <gtest/gtest.h>

std::string relative_path = OPENSOT_TEST_PATH "configs/coman/configs/config_coman_RBDL.yaml";
std::string _path_to_cfg = relative_path;

namespace {

class testAutoStack: public ::testing::Test
{
protected:
    XBot::ModelInterface::Ptr _robot;
    OpenSoT::DefaultHumanoidStack::Ptr DHS;

    testAutoStack()
    {
        _robot = XBot::ModelInterface::getModel(_path_to_cfg);

        if(_robot)
            std::cout<<"pointer address: "<<_robot.get()<<std::endl;
        else
            std::cout<<"pointer is NULL "<<_robot.get()<<std::endl;


        DHS.reset(new OpenSoT::DefaultHumanoidStack(*_robot,
              3e-3,
              "Waist",
              "LSoftHand", "RSoftHand",
              "l_sole", "r_sole", 0.3,
              Eigen::VectorXd::Zero(_robot->getJointNum())));
    }

    virtual ~testAutoStack() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(testAutoStack, test_getOperationalSpaceTask_with_task_id)
{
    using namespace OpenSoT;
    std::string task_id = "CoM";

    AutoStack::Ptr auto_stack = (DHS->right2LeftLeg)/
            (DHS->com + DHS->leftArm)/
            DHS->postural;
    auto_stack->update(Eigen::VectorXd::Zero(_robot->getJointNum()));

    OpenSoT::solvers::iHQP::TaskPtr com_task = auto_stack->getOperationalSpaceTask(task_id);
    EXPECT_TRUE(com_task != NULL);
    std::shared_ptr<OpenSoT::tasks::velocity::CoM> task_CoM =
            std::dynamic_pointer_cast<OpenSoT::tasks::velocity::CoM>(com_task);
    EXPECT_TRUE(task_CoM != NULL);
    EXPECT_TRUE(task_CoM->getTaskID().compare(task_id) == 0);

    task_id = "mammeta";
    OpenSoT::solvers::iHQP::TaskPtr mammeta_task = auto_stack->getOperationalSpaceTask(task_id);
    EXPECT_TRUE(mammeta_task == NULL);

    task_id = "cartesian::l_wrist";
    OpenSoT::solvers::iHQP::TaskPtr Cartesian_task = auto_stack->getOperationalSpaceTask(task_id);
    EXPECT_TRUE(Cartesian_task != NULL);
    std::shared_ptr<OpenSoT::tasks::velocity::Cartesian> left_arm =
            std::dynamic_pointer_cast<OpenSoT::tasks::velocity::Cartesian>(Cartesian_task);
    EXPECT_TRUE(left_arm != NULL);
    EXPECT_TRUE(left_arm->getTaskID().compare(task_id) == 0);

    task_id = "cartesian:r2l_sole";
    OpenSoT::solvers::iHQP::TaskPtr Cartesian_task2 = auto_stack->getOperationalSpaceTask(task_id);
    EXPECT_TRUE(Cartesian_task2 != NULL);
    std::shared_ptr<OpenSoT::tasks::velocity::Cartesian> right_left_leg =
            std::dynamic_pointer_cast<OpenSoT::tasks::velocity::Cartesian>(Cartesian_task2);
    EXPECT_TRUE(right_left_leg != NULL);
    EXPECT_TRUE(right_left_leg->getTaskID().compare(task_id) == 0);
}

TEST_F(testAutoStack, test_getOperationalSpaceTask_with_links)
{
    using namespace OpenSoT;
    std::string base_link = "world";
    std::string distal_link = "CoM";

    AutoStack::Ptr auto_stack = (DHS->right2LeftLeg)/
            (DHS->com + DHS->leftArm)/
            DHS->postural;
    auto_stack->update(Eigen::VectorXd::Zero(_robot->getJointNum()));

    OpenSoT::solvers::iHQP::TaskPtr com_task = auto_stack->getOperationalSpaceTask(base_link, distal_link);
    EXPECT_TRUE(com_task != NULL);
    std::shared_ptr<OpenSoT::tasks::velocity::CoM> task_CoM =
            std::dynamic_pointer_cast<OpenSoT::tasks::velocity::CoM>(com_task);
    EXPECT_TRUE(task_CoM != NULL);
    EXPECT_TRUE(task_CoM->getBaseLink().compare(base_link) == 0);
    EXPECT_TRUE(task_CoM->getDistalLink().compare(distal_link) == 0);

    base_link = "mammeta";
    OpenSoT::solvers::iHQP::TaskPtr mammeta_task = auto_stack->getOperationalSpaceTask(base_link, distal_link);
    EXPECT_TRUE(mammeta_task == NULL);

    base_link = "world";
    distal_link = "LSoftHand";
    OpenSoT::solvers::iHQP::TaskPtr Cartesian_task = auto_stack->getOperationalSpaceTask(base_link, distal_link);
    EXPECT_TRUE(Cartesian_task != NULL);
    std::shared_ptr<OpenSoT::tasks::velocity::Cartesian> left_arm =
            std::dynamic_pointer_cast<OpenSoT::tasks::velocity::Cartesian>(Cartesian_task);
    EXPECT_TRUE(left_arm != NULL);
    EXPECT_TRUE(left_arm->getBaseLink().compare(base_link) == 0);
    EXPECT_TRUE(left_arm->getDistalLink().compare(distal_link) == 0);

    base_link = "l_sole";
    distal_link = "r_sole";
    OpenSoT::solvers::iHQP::TaskPtr Cartesian_task2 = auto_stack->getOperationalSpaceTask(base_link, distal_link);
    EXPECT_TRUE(Cartesian_task2 != NULL);
    std::shared_ptr<OpenSoT::tasks::velocity::Cartesian> right_left_leg =
            std::dynamic_pointer_cast<OpenSoT::tasks::velocity::Cartesian>(Cartesian_task2);
    EXPECT_TRUE(right_left_leg != NULL);
    EXPECT_TRUE(right_left_leg->getBaseLink().compare(base_link) == 0);
    EXPECT_TRUE(right_left_leg->getDistalLink().compare(distal_link) == 0);
}

TEST_F(testAutoStack, testOperatorPlus )
{
    using namespace OpenSoT;
    tasks::Aggregated::Ptr aggr1 = DHS->leftArm + DHS->rightArm;
    Eigen::MatrixXd W1 = aggr1->getWeight();
    W1 = 3.*W1;
    aggr1->setWeight(W1);
    EXPECT_EQ(aggr1->getTaskList().size(), 2);

    tasks::Aggregated::Ptr aggr2 = DHS->leftLeg + DHS->rightLeg;
    Eigen::MatrixXd W2 = aggr2->getWeight();
    W2 = 5.*W2;
    aggr2->setWeight(W2);
    EXPECT_EQ(aggr2->getTaskList().size(), 2);

    tasks::Aggregated::Ptr aggr3 = aggr1 + aggr2;
    Eigen::MatrixXd W3 = aggr3->getWeight();
    EXPECT_TRUE(W3.block(0,0,W1.rows(),W1.cols()) == W1);
    EXPECT_TRUE(W3.block(W1.rows(), W1.cols(), W2.rows(), W2.cols()) == W2);

    Eigen::MatrixXd matBlock0_des(W1.rows(), W2.cols());
    matBlock0_des.setZero(W1.rows(), W2.cols());
    Eigen::MatrixXd matBlock0 = W3.block(0, W1.cols(), W1.rows(), W2.cols());

    EXPECT_TRUE(matBlock0_des == matBlock0);

    Eigen::MatrixXd matBlock1_des(W2.rows(), W1.cols());
    matBlock1_des.setZero(W2.rows(), W1.cols());
    Eigen::MatrixXd matBlock1 = W3.block(W1.rows(),0, W2.rows(),W1.cols());
    EXPECT_TRUE(matBlock1_des == matBlock1);
    EXPECT_EQ(aggr3->getTaskList().size(), 4);

    aggr1->setLambda(.1);
    aggr2->setLambda(.2);

    tasks::Aggregated::Ptr aggr4 = aggr1 + aggr2;
    EXPECT_EQ(aggr4->getTaskList().size(), 2);

    tasks::Aggregated::Ptr aggr5 = DHS->leftArm + DHS->rightArm
                                 + DHS->leftLeg + DHS->rightLeg;
    EXPECT_EQ(aggr5->getTaskList().size(), 4);

    tasks::Aggregated::Ptr aggr6 = aggr1 + DHS->leftLeg;
    EXPECT_EQ(aggr6->getTaskList().size(), 3);

    tasks::Aggregated::Ptr aggr7 = DHS->leftLeg + aggr1;
    EXPECT_EQ(aggr7->getTaskList().size(), 3);

    /*
     * checking constraints are carried over in the right way
     */
    tasks::Aggregated::Ptr aggr8 = DHS->leftLeg + DHS->rightLeg;
    aggr8->getConstraints().push_back(DHS->jointLimits);

    tasks::Aggregated::Ptr aggr9 = DHS->leftLeg + DHS->rightLeg;
    aggr9->getConstraints().push_back(DHS->comVelocity);

    tasks::Aggregated::Ptr aggr10 = DHS->leftLeg + DHS->rightLeg;
    aggr10->getConstraints().push_back(DHS->velocityLimits);
    aggr10->getConstraints().push_back(DHS->comVelocity);

    EXPECT_EQ(aggr8->getConstraints().size(),1);
    EXPECT_EQ(aggr9->getConstraints().size(),1);
    EXPECT_EQ(aggr10->getConstraints().size(),2);
    EXPECT_EQ((aggr8+aggr9)->getConstraints().size(),2);
    EXPECT_EQ((aggr8+aggr10)->getConstraints().size(),3);
    EXPECT_EQ((aggr9+aggr10)->getConstraints().size(),2);
}

TEST_F(testAutoStack, testOperatorFraction)
{
    using namespace OpenSoT;

    tasks::Aggregated::Ptr aggr1 = DHS->leftArm + DHS->rightArm;
    EXPECT_TRUE(aggr1->getTaskList().size() == 2);

    AutoStack::Ptr auto1 = DHS->leftArm / DHS->rightArm;
    EXPECT_TRUE(auto1->getStack().size() == 2);

    AutoStack::Ptr auto2(new AutoStack(solvers::iHQP::Stack(1,aggr1)));
    EXPECT_EQ(auto2->getStack().size(), 1);

    AutoStack::Ptr auto3 = auto2 / DHS->leftLeg;
    ASSERT_EQ(auto3->getStack().size(), 2);
    EXPECT_TRUE(auto3->getStack()[1] == DHS->leftLeg);

    AutoStack::Ptr auto4 = DHS->leftLeg / auto2;
    ASSERT_EQ(auto4->getStack().size(), 2);
    EXPECT_TRUE(auto4->getStack()[0] == DHS->leftLeg);

    AutoStack::Ptr auto5 = auto3 / auto4;
    ASSERT_TRUE(auto5->getStack().size() == 4);
    EXPECT_TRUE(auto5->getStack()[0] == aggr1);
    EXPECT_TRUE(auto5->getStack()[1] == DHS->leftLeg);
    EXPECT_TRUE(auto5->getStack()[2] == DHS->leftLeg);
    EXPECT_TRUE(auto5->getStack()[3] == aggr1);
}

TEST_F(testAutoStack, testOperatorRedirection)
{
    using namespace OpenSoT;

    tasks::Aggregated::Ptr aggr1 = DHS->leftArm + DHS->rightArm;
    tasks::Aggregated::TaskPtr leftArm = DHS->leftArm;
    AutoStack::Ptr auto1 = DHS->leftArm / DHS->rightArm;

    tasks::Aggregated::Ptr aggr1Copy = aggr1 << DHS->velocityLimits;
    EXPECT_TRUE(aggr1->getConstraints().size() == 1);
    ASSERT_TRUE(aggr1->getConstraints().front() == DHS->velocityLimits);
    EXPECT_TRUE(aggr1Copy->getConstraints().size() == 1);
    ASSERT_TRUE(aggr1Copy->getConstraints().front() == DHS->velocityLimits);

    tasks::Aggregated::TaskPtr leftArmCopy = leftArm << DHS->velocityLimits;
    EXPECT_TRUE(leftArm->getConstraints().size() == 1);
    ASSERT_TRUE(leftArm->getConstraints().front() == DHS->velocityLimits);
    EXPECT_TRUE(leftArmCopy->getConstraints().size() == 1);
    ASSERT_TRUE(leftArmCopy->getConstraints().front() == DHS->velocityLimits);

    aggr1 << DHS->jointLimits;
    ASSERT_TRUE(aggr1->getConstraints().size() == 2);
    ASSERT_TRUE(aggr1->getConstraints().back() == DHS->jointLimits);

    leftArm << DHS->jointLimits;
    EXPECT_TRUE(leftArm->getConstraints().size() == 2);
    ASSERT_TRUE(leftArm->getConstraints().back() == DHS->jointLimits);

    auto1 << DHS->velocityLimits;
    EXPECT_TRUE(auto1->getBoundsList().size() == 1);

    auto1 << DHS->comVelocity << DHS->jointLimits;
    EXPECT_TRUE(auto1->getBoundsList().size() == 3);

    AutoStack::Ptr auto2 =
    (leftArm + (DHS->rightArm << DHS->comVelocity))
        / (DHS->rightLeg + DHS->leftLeg) << DHS->jointLimits << DHS->velocityLimits;
    EXPECT_TRUE(auto2->getBoundsList().size() == 2);
    EXPECT_TRUE(auto2->getStack().size() == 2);
    EXPECT_TRUE(std::dynamic_pointer_cast<tasks::Aggregated>(auto2->getStack()[0])->getTaskList().size() == 2);
    EXPECT_EQ(std::dynamic_pointer_cast<tasks::Aggregated>(auto2->getStack()[0])->getTaskList().front()->getConstraints().size(), 2);
    EXPECT_EQ(std::dynamic_pointer_cast<tasks::Aggregated>(auto2->getStack()[0])->getTaskList().back()->getConstraints().size(), 1);
    EXPECT_EQ(std::dynamic_pointer_cast<tasks::Aggregated>(auto2->getStack()[1])->getTaskList().size(), 2);
    EXPECT_EQ(std::dynamic_pointer_cast<tasks::Aggregated>(auto2->getStack()[1])->getTaskList().front()->getConstraints().size(), 0);
}

TEST_F(testAutoStack, testTaskConstructor)
{
    using namespace OpenSoT;

    AutoStack::Ptr autostack(new AutoStack(DHS->postural));
    autostack = autostack<<DHS->jointLimits<<DHS->velocityLimits;

    EXPECT_TRUE(autostack->getStack().size() == 1);
    EXPECT_TRUE(autostack->getBoundsList().size() == 2);
}

TEST_F(testAutoStack, testOperatorModulo)
{
    using namespace OpenSoT;

    std::list<unsigned int> indices;
    indices.push_back(0);
    indices.push_back(2);
    indices.push_back(3);

    OpenSoT::SubTask::Ptr sub_task = DHS->leftArm%indices;

    EXPECT_EQ(sub_task->getA().rows(), 3);
    EXPECT_EQ(sub_task->getA().cols(), DHS->leftArm->getA().cols());
    EXPECT_EQ(sub_task->getb().size(), 3);
    EXPECT_EQ(sub_task->getWeight().rows(), 3);
    EXPECT_EQ(sub_task->getWeight().cols(), 3);

    KDL::Frame ref; ref.Identity();
    DHS->leftArm->setReference(ref);

    sub_task->update(Eigen::VectorXd::Zero(DHS->leftArm->getA().cols()));

    EXPECT_TRUE(sub_task->getA().row(0) == DHS->leftArm->getA().row(0));
    EXPECT_TRUE(sub_task->getA().row(1) == DHS->leftArm->getA().row(2));
    EXPECT_TRUE(sub_task->getA().row(2) == DHS->leftArm->getA().row(3));

    EXPECT_TRUE(sub_task->getb()[0] == DHS->leftArm->getb()[0]);
    EXPECT_TRUE(sub_task->getb()[1] == DHS->leftArm->getb()[2]);
    EXPECT_TRUE(sub_task->getb()[2] == DHS->leftArm->getb()[3]);



    std::list<unsigned int> indices2;
    indices.push_back(0);
    OpenSoT::AutoStack::Ptr autostack = (DHS->leftArm%indices + DHS->rightArm%indices2)/(DHS->com);
    EXPECT_EQ(autostack->getStack()[0]->getA().rows(), indices.size() + indices2.size());
    std::cout<<"autostack->getStack()[0]->getA().rows(): "<<autostack->getStack()[0]->getA().rows()<<std::endl;
}

TEST_F(testAutoStack, testOperatorTimes)
{
    using namespace OpenSoT;

    Eigen::MatrixXd W(6,6);
    double acc = 0;
    for(unsigned int i = 0; i < 6; ++i)
    {
        for(unsigned int j = 0; j < 6; ++j){
            W(i,j) = acc;
            acc++;}
    }

    OpenSoT::tasks::Aggregated::TaskPtr TASK = W*(DHS->leftArm);


    Eigen::MatrixXd WleftArm = DHS->leftArm->getWeight();
    Eigen::MatrixXd WTASK = TASK->getWeight();

    std::cout<<"W: \n"<<W<<std::endl;
    std::cout<<"WleftArm: \n"<<WleftArm<<std::endl;
    std::cout<<"WTASK: \n"<<WTASK<<std::endl;

    for(unsigned int i = 0; i < 6; ++i)
    {
        for(unsigned int j = 0; j < 6; ++j)
        {
            EXPECT_EQ(WleftArm(i,j), W(i,j));
            EXPECT_EQ(WTASK(i,j), W(i,j));
        }
    }

    double w = 10.;
    OpenSoT::tasks::Aggregated::TaskPtr TASK2 = w*TASK;
    WleftArm = DHS->leftArm->getWeight();
    WTASK = TASK->getWeight();
    Eigen::MatrixXd WTASK2 = TASK2->getWeight();

    std::cout<<"WleftArm: \n"<<WleftArm<<std::endl;
    std::cout<<"WTASK: \n"<<WTASK<<std::endl;
    std::cout<<"WTASK2: \n"<<WTASK2<<std::endl;

    for(unsigned int i = 0; i < 6; ++i)
    {
        for(unsigned int j = 0; j < 6; ++j)
        {
            EXPECT_EQ(WleftArm(i,j), w*W(i,j));
            EXPECT_EQ(WTASK(i,j), w*W(i,j));
            EXPECT_EQ(WTASK2(i,j), w*W(i,j));
        }
    }

    std::cout<<"WleftArm: \n"<<DHS->leftArm->getWeight()<<std::endl;
    std::cout<<"WrightArm: \n"<<DHS->rightArm->getWeight()<<std::endl;

    std::cout<<"---------"<<std::endl;

    OpenSoT::tasks::Aggregated::Ptr TASKS = 10*(DHS->leftArm + DHS->rightArm);
    WleftArm = DHS->leftArm->getWeight();
    Eigen::MatrixXd WrightArm = DHS->rightArm->getWeight();
    WTASK = TASK->getWeight();
    WTASK2 = TASK2->getWeight();
    Eigen::MatrixXd WTASKS = TASKS->getWeight();

    std::cout<<"WleftArm: \n"<<WleftArm<<std::endl;
    std::cout<<"WTASK: \n"<<WTASK<<std::endl;
    std::cout<<"WTASK2: \n"<<WTASK2<<std::endl;
    std::cout<<"WrightArm: \n"<<WrightArm<<std::endl;
    std::cout<<"WTASKS: \n"<<WTASKS<<std::endl;

    for(unsigned int i = 0; i < DHS->leftArm->getWeight().rows(); ++i)
    {
        for(unsigned int j = 0; j < DHS->leftArm->getWeight().cols(); ++j)
        {
            EXPECT_EQ(WTASKS(i,j),WleftArm(i,j));
            EXPECT_EQ(WTASKS(i,j),WTASK(i,j));
        }
    }

    for(unsigned int i = 0; i < DHS->rightArm->getWeight().rows(); ++i)
    {
        for(unsigned int j = 0; j < DHS->rightArm->getWeight().cols(); ++j)
        {
            EXPECT_EQ(WTASKS(WleftArm.rows()+i,WleftArm.cols()+j),WrightArm(i,j));
        }
    }

}

TEST_F(testAutoStack, testOperatorStackEqual)
{
    OpenSoT::AutoStack::Ptr autostack;
    autostack /= DHS->leftArm;

    EXPECT_TRUE(autostack.get());
    EXPECT_TRUE(autostack->getStack().size()==1);

    autostack/=DHS->rightArm;
    EXPECT_TRUE(autostack->getStack().size()==2);

    EXPECT_TRUE(autostack->getStack()[0]->getA() == DHS->leftArm->getA());
    EXPECT_TRUE(autostack->getStack()[1]->getA() == DHS->rightArm->getA());
}

TEST_F(testAutoStack, testOperatorTaskToConstraint)
{
    //1) "TaskPtr << TaskPtr"
    OpenSoT::tasks::Aggregated::TaskPtr Task = DHS->leftArm << DHS->rightArm;
    auto constraints = Task->getConstraints();
    EXPECT_EQ(constraints.size(), 1);
    auto constraint = *(constraints.begin());
    EXPECT_TRUE(constraint->getAineq() == DHS->rightArm->getA());
    EXPECT_TRUE(constraint->getbUpperBound() == DHS->rightArm->getb());
    EXPECT_TRUE(constraint->getbLowerBound() == DHS->rightArm->getb());

    //NOTE: DHS->leftArm now has one internal constraint!
    std::cout<<"DHS->leftArm->getConstraints().size(): "<< DHS->leftArm->getConstraints().size()<<std::endl;
    DHS->leftArm->getConstraints().pop_back();
    std::cout<<"pop constraint from DHS->leftArm"<<std::endl;
    std::cout<<"DHS->leftArm->getConstraints().size(): "<< DHS->leftArm->getConstraints().size()<<std::endl;
    std::cout<<"Task->getConstraints().size(): "<< Task->getConstraints().size()<<std::endl;


    //2) "TaskPtr << Aggregated::Ptr"
    OpenSoT::tasks::Aggregated::TaskPtr Task2 = DHS->leftLeg << (DHS->leftArm + DHS->rightArm);
    auto constraints2 = Task2->getConstraints();
    EXPECT_EQ(constraints2.size(), 1);
    auto constraint2 = *(constraints2.begin());
    EXPECT_TRUE(constraint2->getAineq() == (DHS->leftArm + DHS->rightArm)->getA());
    EXPECT_TRUE(constraint2->getbUpperBound() == (DHS->leftArm + DHS->rightArm)->getb());
    EXPECT_TRUE(constraint2->getbLowerBound() == (DHS->leftArm + DHS->rightArm)->getb());

    //NOTE: DHS->leftLeg now has one internal constraint!
    std::cout<<"DHS->leftLeg->getConstraints().size(): "<< DHS->leftLeg->getConstraints().size()<<std::endl;
    DHS->leftLeg->getConstraints().pop_back();
    std::cout<<"pop constraint from DHS->leftLeg"<<std::endl;
    std::cout<<"DHS->leftLeg->getConstraints().size(): "<< DHS->leftLeg->getConstraints().size()<<std::endl;
    std::cout<<"Task2->getConstraints().size(): "<< Task2->getConstraints().size()<<std::endl;

    //3) "Aggregated::Ptr << TaskPtr"
    OpenSoT::tasks::Aggregated::TaskPtr Task3 =  (DHS->leftArm + DHS->rightArm) << DHS->leftLeg;
    auto constraints3 = Task3->getConstraints();
    EXPECT_EQ(constraints3.size(), 1);
    auto constraint3 = *(constraints3.begin());
    EXPECT_TRUE(constraint3->getAineq() == DHS->leftLeg->getA());
    EXPECT_TRUE(constraint3->getbUpperBound() == DHS->leftLeg->getb());
    EXPECT_TRUE(constraint3->getbLowerBound() == DHS->leftLeg->getb());

    //4) "Aggregated::Ptr << Aggregated::Ptr"
    OpenSoT::tasks::Aggregated::TaskPtr Task4 =  (DHS->leftArm + DHS->rightArm) << (DHS->leftLeg + DHS->rightLeg);
    auto constraints4 = Task4->getConstraints();
    EXPECT_EQ(constraints4.size(), 1);
    auto constraint4 = *(constraints4.begin());
    EXPECT_TRUE(constraint4->getAineq() == (DHS->leftLeg + DHS->rightLeg)->getA());
    EXPECT_TRUE(constraint4->getbUpperBound() == (DHS->leftLeg + DHS->rightLeg)->getb());
    EXPECT_TRUE(constraint4->getbLowerBound() == (DHS->leftLeg + DHS->rightLeg)->getb());

    //5) "Stack << TaskPtr"
    OpenSoT::AutoStack::Ptr stack = (DHS->leftArm / DHS->rightArm) << DHS->leftLeg;
    auto constraints5 = stack->getBoundsList();
    EXPECT_EQ(constraints5.size(), 1);
    auto constraint5 = *(constraints5.begin());
    EXPECT_TRUE(constraint5->getAineq() == DHS->leftLeg->getA());
    EXPECT_TRUE(constraint5->getbUpperBound() == DHS->leftLeg->getb());
    EXPECT_TRUE(constraint5->getbLowerBound() == DHS->leftLeg->getb());

    //6) "Stack << Aggregated::Ptr"
    OpenSoT::AutoStack::Ptr stack2 = (DHS->leftArm / DHS->rightArm) << (DHS->leftLeg + DHS->rightLeg);
    auto constraints6 = stack2->getBoundsList();
    EXPECT_EQ(constraints6.size(), 1);
    auto constraint6 = *(constraints6.begin());
    EXPECT_TRUE(constraint6->getAineq() == (DHS->leftLeg + DHS->rightLeg)->getA());
    EXPECT_TRUE(constraint6->getbUpperBound() == (DHS->leftLeg + DHS->rightLeg)->getb());
    EXPECT_TRUE(constraint6->getbLowerBound() == (DHS->leftLeg + DHS->rightLeg)->getb());

}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
