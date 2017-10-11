#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/velocity/MinimizeAcceleration.h>
#include <qpOASES.hpp>
#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>

using namespace OpenSoT::constraints;
using namespace OpenSoT::tasks::velocity;


class testMinimizeAcceleration: public ::testing::Test
{
protected:

    testMinimizeAcceleration()
    {
    }

    virtual ~testMinimizeAcceleration() {
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

Eigen::VectorXd getGoodInitialPosition(const XBot::ModelInterface::Ptr _model_ptr) {
    Eigen::VectorXd _q(_model_ptr->getJointNum());
    _q.setZero(_q.size());
    _q[_model_ptr->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("LShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LShLat")] = 10.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("LElbj")] = -80.0*M_PI/180.0;

    _q[_model_ptr->getDofIndex("RShSag")] =  20.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RShLat")] = -10.0*M_PI/180.0;
    _q[_model_ptr->getDofIndex("RElbj")] = -80.0*M_PI/180.0;
    return _q;
}


TEST_F(testMinimizeAcceleration, testMinimizeAccelerationInCartesianTask)
{
    XBot::ModelInterface::Ptr _model_ptr;
    std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
    std::string relative_path = "/external/OpenSoT-lite/tests/configs/coman/configs/config_coman.yaml";

    std::string _path_to_cfg = robotology_root + relative_path;

    _model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

    if(_model_ptr)
        std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
    else
        std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;

    XBot::ModelInterface::Ptr _model_ptr2;

    _model_ptr2 = XBot::ModelInterface::getModel(_path_to_cfg);

    if(_model_ptr2)
        std::cout<<"pointer address: "<<_model_ptr2.get()<<std::endl;
    else
        std::cout<<"pointer is NULL "<<_model_ptr2.get()<<std::endl;

   Eigen::VectorXd q = getGoodInitialPosition(_model_ptr);

   _model_ptr->setJointPosition(q);
   _model_ptr->update();

   _model_ptr2->setJointPosition(q);
   _model_ptr2->update();

    /// Cartesian Task
//    Eigen::MatrixXd T_init = idynutils.iDynTree_model.getPosition(
//                          idynutils.iDynTree_model.getLinkIndex("Waist"),
//                          idynutils.iDynTree_model.getLinkIndex("l_wrist"));
    Eigen::Affine3d T_init;
    _model_ptr->getPose("l_wrist", "Waist", T_init);
    OpenSoT::tasks::velocity::Cartesian::Ptr cartesian_task(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist",
                              q, *(_model_ptr.get()),"l_wrist", "Waist"));
    Eigen::MatrixXd T_ref = T_init.matrix();
    T_ref(0,3) = T_ref(0,3) + 0.1;
    T_ref(1,3) = T_ref(1,3) + 0.1;
    T_ref(2,3) = T_ref(2,3) + 0.1;
    cartesian_task->setReference(T_ref);
    cartesian_task->update(q);

    OpenSoT::tasks::velocity::Cartesian::Ptr cartesian_task2(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist",
                                    q, *(_model_ptr2.get()),"l_wrist", "Waist"));
    Eigen::MatrixXd T_ref2 = T_init.matrix();
    T_ref2(0,3) = T_ref2(0,3) + 0.1;
    T_ref2(1,3) = T_ref2(1,3) + 0.1;
    T_ref2(2,3) = T_ref2(2,3) + 0.1;
    cartesian_task2->setReference(T_ref2);
    cartesian_task2->update(q);

    /// Postural Task
    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
                new OpenSoT::tasks::velocity::Postural(q));
    postural_task->setReference(q);

    /// MinAcc Task
    OpenSoT::tasks::velocity::MinimizeAcceleration::Ptr minacc_task(
                new OpenSoT::tasks::velocity::MinimizeAcceleration(q));

    int t = 100;
    Eigen::VectorXd qmax, qmin;
    _model_ptr->getJointLimits(qmin, qmax);
    /// Constraints set to the Cartesian Task
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits(
        new OpenSoT::constraints::velocity::JointLimits(
                    q, qmax,qmin, 0.2));

    _model_ptr2->getJointLimits(qmin, qmax);
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits2(
        new OpenSoT::constraints::velocity::JointLimits(
                    q, qmax,qmin, 0.2));

    OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_velocity_limits(
                new OpenSoT::constraints::velocity::VelocityLimits(M_PI/2.0, (double)(1.0/t), q.size()));
    OpenSoT::constraints::velocity::VelocityLimits::Ptr joint_velocity_limits2(
                new OpenSoT::constraints::velocity::VelocityLimits(M_PI/2.0, (double)(1.0/t), q.size()));

    std::list< OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr > joint_constraints_list;
    joint_constraints_list.push_back(joint_limits);
    joint_constraints_list.push_back(joint_velocity_limits);

    std::list< OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr > joint_constraints_list2;
    joint_constraints_list2.push_back(joint_limits2);
    joint_constraints_list2.push_back(joint_velocity_limits2);

    OpenSoT::constraints::Aggregated::Ptr joint_constraints(
                new OpenSoT::constraints::Aggregated(joint_constraints_list, q.size()));

    OpenSoT::constraints::Aggregated::Ptr joint_constraints2(
                new OpenSoT::constraints::Aggregated(joint_constraints_list2, q.size()));

    //Create the SoT
    std::vector< OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr > stack_of_tasks;
    stack_of_tasks.push_back(cartesian_task);
    stack_of_tasks.push_back(postural_task);

    std::vector< OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr > stack_of_tasks2;
    stack_of_tasks2.push_back(cartesian_task2);
    stack_of_tasks2.push_back(minacc_task);

    OpenSoT::solvers::QPOases_sot sot(stack_of_tasks, joint_constraints);

    OpenSoT::solvers::QPOases_sot sot2(stack_of_tasks2, joint_constraints2);

    //Solve SoT
    Eigen::VectorXd dq(q.size()); dq.setZero(q.size());
    Eigen::VectorXd dq2(q.size()); dq2.setZero(q.size());
    Eigen::VectorXd q2(q.size()); q2 = q;

    Eigen::VectorXd ddq(q.size()); ddq = dq;
    Eigen::VectorXd ddq2(q.size()); ddq2 = dq2;

    double integrator1 = 0.0;
    double integrator2 = 0.0;

    for(unsigned int i = 0; i < 5*t; ++i)
    {
        Eigen::VectorXd dq_old(q.size());
        dq_old = dq;
        Eigen::VectorXd dq_old2(q.size());
        dq_old2 = dq2;

        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        _model_ptr2->setJointPosition(q);
        _model_ptr2->update();


        cartesian_task->update( q);
        postural_task->update( q);
        joint_constraints->update( q);

        cartesian_task2->update( q2);
        minacc_task->update( q2);
        for(unsigned int i = 0; i < dq_old2.size(); ++i)
            ASSERT_NEAR(minacc_task->getb()[i], dq_old2[i], 1E-14);
        joint_constraints2->update( q2);

        sot.solve(dq);
        q += dq;
        ddq = dq - dq_old;


        sot2.solve(dq2);
        q2 += dq2;
        ddq2 = dq2 - dq_old2;

        for(unsigned int j = 0; j < ddq.size(); ++j){
            integrator1 += ddq.norm();
            integrator2 += ddq2.norm();
        }
    }

    std::cout<<"integrator 1 = "<<integrator1<<std::endl;
    std::cout<<"integrator 2 = "<<integrator2<<std::endl;
    ASSERT_TRUE(integrator2 <= integrator1);

    std::cout<<"**************T1*************"<<std::endl;
    _model_ptr->setJointPosition(q);
    _model_ptr->update();

    std::cout<<"INITIAL CONFIG: "<<
               T_init.matrix()<<std::endl;
    Eigen::Affine3d T;
    _model_ptr->getPose("l_wrist", "Waist", T);
    std::cout<<"FINAL CONFIG: "<<T.matrix()<<std::endl;
    std::cout<<"DESIRED CONFIG: "<<T_ref<<std::endl;

    for(unsigned int i = 0; i <= 3; ++i){
        for(unsigned int j = 0; j <= 3; ++j)
            EXPECT_NEAR(T(i,j), T_ref(i,j), 1E-3);
    }

    std::cout<<std::endl;

    std::cout<<"**************T2*************"<<std::endl;
    _model_ptr2->setJointPosition(q);
    _model_ptr2->update();

    std::cout<<"INITIAL CONFIG: "<<T_init.matrix()<<std::endl;
    Eigen::Affine3d T2;
    _model_ptr2->getPose("l_wrist", "Waist", T2);
    std::cout<<"FINAL CONFIG: "<<T2.matrix()<<std::endl;
    std::cout<<"DESIRED CONFIG: "<<T_ref<<std::endl;

    for(unsigned int i = 0; i <= 3; ++i){
        for(unsigned int j = 0; j <= 3; ++j)
            EXPECT_NEAR(T2(i,j), T_ref(i,j), 1E-3);
    }

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
