#include <xbot2_interface/xbotinterface2.h>
#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/tasks/velocity/Manipulability.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <qpOASES.hpp>
#include <fstream>
#include "../../common.h"

using namespace OpenSoT::constraints::velocity;
using namespace OpenSoT::tasks::velocity;

namespace {

class testManipolability: public TestBase
{
protected:
    testManipolability() : TestBase("coman_floating_base")
    {
         //_log.open("testMinimizeAcceleration.m");
    }

    virtual ~testManipolability() {
         //_log.close();
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

double computeManipIndex(const Eigen::MatrixXd& A)
{
    return sqrt((A*A.transpose()).determinant());
}

TEST_F(testManipolability, testManipolabilityTask)
{
    Eigen::VectorXd q = _model_ptr->getNeutralQ();

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    /// Cartesian Tasks
    Eigen::Affine3d TL_init;
    _model_ptr->getPose("Waist", "l_wrist", TL_init);
    Eigen::Affine3d TR_init;
    _model_ptr->getPose("Waist", "r_wrist", TR_init);

    Cartesian::Ptr cartesian_task_L(new Cartesian("cartesian::left_wrist",
        *(_model_ptr.get()),"l_wrist", "Waist"));
    Cartesian::Ptr cartesian_task_R(new Cartesian("cartesian::right_wrist",
        *(_model_ptr.get()),"r_wrist", "Waist"));
    std::list< OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr > task_list;
    task_list.push_back(cartesian_task_L);
    task_list.push_back(cartesian_task_R);
    OpenSoT::tasks::Aggregated::Ptr cartesian_task(
                new OpenSoT::tasks::Aggregated(task_list, _model_ptr->getNv()));


    /// Postural Task
    Postural::Ptr postural_task(new Postural(*_model_ptr));

    /// Manipulability task
    Manipulability::Ptr manipulability_task_L(new Manipulability(
                                                  *(_model_ptr.get()), cartesian_task_L));
    Manipulability::Ptr manipulability_task_R(new Manipulability(
                                                  *(_model_ptr.get()), cartesian_task_R));
    std::list< OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr > manip_list;
    manip_list.push_back(manipulability_task_L);
    manip_list.push_back(manipulability_task_R);
    OpenSoT::tasks::Aggregated::Ptr manipulability_task(
                new OpenSoT::tasks::Aggregated(manip_list, _model_ptr->getNv()));

    /// Constraints set to the Cartesian Task
    int t = 1000;
    Eigen::VectorXd qmin, qmax;
    _model_ptr->getJointLimits(qmin, qmax);
    JointLimits::Ptr joint_limits(
        new JointLimits(*_model_ptr, qmax,
                           qmin, 0.2));
    VelocityLimits::Ptr joint_velocity_limits(
                new VelocityLimits(*_model_ptr, M_PI/2.0, (double)(1.0/t)));

    std::list< OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr > joint_constraints_list;
    joint_constraints_list.push_back(joint_limits);
    joint_constraints_list.push_back(joint_velocity_limits);
    OpenSoT::constraints::Aggregated::Ptr joint_constraints(
                new OpenSoT::constraints::Aggregated(joint_constraints_list, _model_ptr->getNv()));

    std::vector< OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr > stack_of_tasks;
    stack_of_tasks.push_back(cartesian_task);
    stack_of_tasks.push_back(postural_task);

    OpenSoT::solvers::iHQP sot(stack_of_tasks, joint_constraints);

    Eigen::VectorXd dq(_model_ptr->getNv());
    dq.setZero();

    for(unsigned int i = 0; i < 3*t; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();


        cartesian_task->update(Eigen::VectorXd(0));
        postural_task->update(Eigen::VectorXd(0));
        manipulability_task->update(Eigen::VectorXd(0));
        joint_constraints->update(Eigen::VectorXd(0));

        sot.solve(dq);
        q = _model_ptr->sum(q, dq);

        double manip_index_L = computeManipIndex(cartesian_task_L->getA());
        double manip_index_R = computeManipIndex(cartesian_task_R->getA());

        ASSERT_NEAR(manipulability_task_L->ComputeManipulabilityIndex(), manip_index_L, 1e-10);
        ASSERT_NEAR(manipulability_task_R->ComputeManipulabilityIndex(), manip_index_R, 1e-10);
    }
    double manip_index_L = computeManipIndex(cartesian_task_L->getA());
    double manip_index_R = computeManipIndex(cartesian_task_R->getA());

    std::cout<<"MANIP INDEX L: "<<manip_index_L<<std::endl;
    std::cout<<"MANIP INDEX R: "<<manip_index_R<<std::endl;

    _model_ptr->setJointPosition(q);
    _model_ptr->update();


    std::cout<<"INITIAL CONFIG: "<<TL_init.matrix()<<std::endl;
    Eigen::Affine3d TL;
    _model_ptr->getPose("Waist", "l_wrist", TL);
    std::cout<<"FINAL CONFIG: "<<TL.matrix()<<std::endl;

    for(unsigned int i = 0; i <= 3; ++i){
        for(unsigned int j = 0; j <= 3; ++j)
            EXPECT_NEAR(TL_init(i,j), TL(i,j), 1E-3);
    }

    std::cout<<"INITIAL CONFIG: "<<TR_init.matrix()<<std::endl;
    Eigen::Affine3d TR;
    _model_ptr->getPose("Waist", "r_wrist", TR);
    std::cout<<"FINAL CONFIG: "<<TR.matrix()<<std::endl;

    for(unsigned int i = 0; i <= 3; ++i){
        for(unsigned int j = 0; j <= 3; ++j)
            EXPECT_NEAR(TR_init(i,j), TR(i,j), 1E-3);
    }


    stack_of_tasks.clear();
    stack_of_tasks.push_back(cartesian_task);
    stack_of_tasks.push_back(manipulability_task);
    OpenSoT::solvers::iHQP sot_manip(stack_of_tasks, joint_constraints);

    dq.setZero();
    for(unsigned int i = 0; i < 3*t; ++i)
    {
        _model_ptr->setJointPosition(q);
        _model_ptr->update();


        cartesian_task->update(Eigen::VectorXd(0));
        postural_task->update(Eigen::VectorXd(0));
        manipulability_task->update(Eigen::VectorXd(0));
        joint_constraints->update(Eigen::VectorXd(0));

        sot_manip.solve(dq);
        q = _model_ptr->sum(q, dq);

        double manip_index_L = computeManipIndex(cartesian_task_L->getA());
        double manip_index_R = computeManipIndex(cartesian_task_R->getA());
        EXPECT_NEAR(manipulability_task_L->ComputeManipulabilityIndex(), manip_index_L, 1e-10);
        EXPECT_NEAR(manipulability_task_R->ComputeManipulabilityIndex(), manip_index_R, 1e-10);
    }
    double new_manip_index_L = computeManipIndex(cartesian_task_L->getA());
    double new_manip_index_R = computeManipIndex(cartesian_task_R->getA());

    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    std::cout<<"INITIAL CONFIG: "<<TL_init.matrix()<<std::endl;
    _model_ptr->getPose("Waist", "l_wrist", TL);
    std::cout<<"FINAL CONFIG: "<<TL.matrix()<<std::endl;

    for(unsigned int i = 0; i <= 3; ++i){
        for(unsigned int j = 0; j <= 3; ++j)
            EXPECT_NEAR(TL_init(i,j), TL(i,j), 1E-3);
    }

    std::cout<<"INITIAL CONFIG: "<<TR_init.matrix()<<std::endl;
    _model_ptr->getPose("Waist", "r_wrist", TR);
    std::cout<<"FINAL CONFIG: "<<TR.matrix()<<std::endl;

    for(unsigned int i = 0; i <= 3; ++i){
        for(unsigned int j = 0; j <= 3; ++j)
            EXPECT_NEAR(TR_init(i,j), TR(i,j), 1E-3);
    }

    std::cout<<std::endl;

    std::cout<<"INITIAL MANIP INDEX L: "<<manip_index_L<<std::endl;
    std::cout<<"FINAL MANIP INDEX L: "<<new_manip_index_L<<std::endl;
    std::cout<<"INITIAL MANIP INDEX R: "<<manip_index_R<<std::endl;
    std::cout<<"FINAL MANIP INDEX R: "<<new_manip_index_R<<std::endl;

    EXPECT_TRUE(manip_index_L <= new_manip_index_L);
    EXPECT_TRUE(manip_index_R <= new_manip_index_R);

}
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
