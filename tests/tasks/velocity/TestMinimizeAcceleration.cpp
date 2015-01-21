#include <idynutils/idynutils.h>
#include <idynutils/tests_utils.h>
#include <idynutils/comanutils.h>
#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/velocity/all.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/velocity/all.h>
#include <OpenSoT/tasks/velocity/MinimizeAcceleration.h>
#include <qpOASES.hpp>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>
#include <fstream>

using namespace OpenSoT::constraints::velocity;
using namespace OpenSoT::tasks::velocity;
using namespace yarp::math;

class testMinimizeAcceleration: public ::testing::Test
{
protected:
    std::ofstream _log;

    testMinimizeAcceleration()
    {
         _log.open("testMinimizeAcceleration.m");
    }

    virtual ~testMinimizeAcceleration() {
         _log.close();
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

yarp::sig::Vector getGoodInitialPosition(iDynUtils& idynutils) {
    yarp::sig::Vector q(idynutils.iDyn3_model.getNrOfDOFs(), 0.0);
    yarp::sig::Vector leg(idynutils.left_leg.getNrOfDOFs(), 0.0);
    leg[0] = -25.0 * M_PI/180.0;
    leg[3] =  50.0 * M_PI/180.0;
    leg[5] = -25.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(leg, q, idynutils.left_leg);
    idynutils.fromRobotToIDyn(leg, q, idynutils.right_leg);
    yarp::sig::Vector arm(idynutils.left_arm.getNrOfDOFs(), 0.0);
    arm[0] = 20.0 * M_PI/180.0;
    arm[1] = 10.0 * M_PI/180.0;
    arm[3] = -80.0 * M_PI/180.0;
    idynutils.fromRobotToIDyn(arm, q, idynutils.left_arm);
    arm[1] = -arm[1];
    idynutils.fromRobotToIDyn(arm, q, idynutils.right_arm);
    return q;
}

TEST_F(testMinimizeAcceleration, testMinimizeAccelerationInCartesianTask)
{
    iDynUtils idynutils("coman",
                        std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                        std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");
    iDynUtils idynutils2("coman",
                         std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
                         std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf");

    yarp::sig::Vector q = getGoodInitialPosition(idynutils);
    idynutils.updateiDyn3Model(q, true);
    idynutils2.updateiDyn3Model(q, true);

    /// Cartesian Task
    yarp::sig::Matrix T_init = idynutils.iDyn3_model.getPosition(
                          idynutils.iDyn3_model.getLinkIndex("Waist"),
                          idynutils.iDyn3_model.getLinkIndex("l_wrist"));
    boost::shared_ptr<Cartesian> cartesian_task(new Cartesian("cartesian::left_wrist", q, idynutils,"l_wrist", "Waist"));
    yarp::sig::Matrix T_ref = T_init;
    T_ref(0,3) = T_ref(0,3) + 0.1;
    T_ref(1,3) = T_ref(1,3) + 0.1;
    T_ref(2,3) = T_ref(2,3) + 0.1;
    cartesian_task->setReference(T_ref);
    cartesian_task->update(q);

    boost::shared_ptr<Cartesian> cartesian_task2(new Cartesian("cartesian::left_wrist", q, idynutils2,"l_wrist", "Waist"));
    yarp::sig::Matrix T_ref2 = T_init;
    T_ref2(0,3) = T_ref2(0,3) + 0.1;
    T_ref2(1,3) = T_ref2(1,3) + 0.1;
    T_ref2(2,3) = T_ref2(2,3) + 0.1;
    cartesian_task2->setReference(T_ref2);
    cartesian_task2->update(q);

    /// Postural Task
    boost::shared_ptr<Postural> postural_task(new Postural(q));
    postural_task->setReference(q);

    /// MinAcc Task
    boost::shared_ptr<MinimizeAcceleration> minacc_task(new MinimizeAcceleration(q));

    int t = 100;
    /// Constraints set to the Cartesian Task
    boost::shared_ptr<JointLimits> joint_limits(
        new JointLimits(q, idynutils.iDyn3_model.getJointBoundMax(),
                           idynutils.iDyn3_model.getJointBoundMin(), 0.2));

    boost::shared_ptr<JointLimits> joint_limits2(
        new JointLimits(q, idynutils2.iDyn3_model.getJointBoundMax(),
                           idynutils2.iDyn3_model.getJointBoundMin(), 0.2));

    boost::shared_ptr<VelocityLimits> joint_velocity_limits(
                new VelocityLimits(M_PI/2.0, (double)(1.0/t), q.size()));
    boost::shared_ptr<VelocityLimits> joint_velocity_limits2(
                new VelocityLimits(M_PI/2.0, (double)(1.0/t), q.size()));

    std::list<boost::shared_ptr<OpenSoT::Constraint<Matrix, Vector>>> joint_constraints_list;
    joint_constraints_list.push_back(joint_limits);
    joint_constraints_list.push_back(joint_velocity_limits);

    std::list<boost::shared_ptr<OpenSoT::Constraint<Matrix, Vector>>> joint_constraints_list2;
    joint_constraints_list2.push_back(joint_limits2);
    joint_constraints_list2.push_back(joint_velocity_limits2);

    boost::shared_ptr<OpenSoT::constraints::Aggregated> joint_constraints(
                new OpenSoT::constraints::Aggregated(joint_constraints_list, q.size()));

    boost::shared_ptr<OpenSoT::constraints::Aggregated> joint_constraints2(
                new OpenSoT::constraints::Aggregated(joint_constraints_list2, q.size()));

    //Create the SoT
    std::vector<boost::shared_ptr<OpenSoT::Task<Matrix, Vector> >> stack_of_tasks;
    stack_of_tasks.push_back(cartesian_task);
    stack_of_tasks.push_back(postural_task);

    std::vector<boost::shared_ptr<OpenSoT::Task<Matrix, Vector> >> stack_of_tasks2;
    stack_of_tasks2.push_back(cartesian_task2);
    stack_of_tasks2.push_back(minacc_task);

    OpenSoT::solvers::QPOases_sot sot(stack_of_tasks, joint_constraints);

    OpenSoT::solvers::QPOases_sot sot2(stack_of_tasks2, joint_constraints2);

    //Solve SoT
    yarp::sig::Vector dq(q.size(), 0.0);
    yarp::sig::Vector dq2(q.size(), 0.0);
    yarp::sig::Vector q2(q.size(), 0.0); q2 = q;

    yarp::sig::Vector ddq(q.size(), 0.0); ddq = dq;
    yarp::sig::Vector ddq2(q.size(), 0.0); ddq2 = dq2;

    double integrator1 = 0.0;
    double integrator2 = 0.0;

    this->_log<<"clear all; clc"<<std::endl;
    this->_log<<"display('ddq1[1:10] ddq2[11:20] b1[21:26] b1[27:32]')"<<std::endl;
    this->_log<<"ddq = ["<<std::endl;
    for(unsigned int i = 0; i < 5*t; ++i)
    {
        yarp::sig::Vector dq_old(q.size(), 0.0);
        dq_old = dq;
        yarp::sig::Vector dq_old2(q.size(), 0.0);
        dq_old2 = dq2;

        idynutils.updateiDyn3Model(q, true);
        idynutils2.updateiDyn3Model(q2, true);

        cartesian_task->update(q);
        postural_task->update(q);
        joint_constraints->update(q);

        cartesian_task2->update(q2);
        minacc_task->update(q2);
        for(unsigned int i = 0; i < dq_old2.size(); ++i)
            ASSERT_NEAR(minacc_task->getb()[i], dq_old2[i], 1E-14);
        joint_constraints2->update(q2);

        sot.solve(dq);
        q += dq;
        ddq = dq - dq_old;
        this->_log<<ddq.subVector(idynutils.torso.joint_numbers[0],
                idynutils.torso.joint_numbers[idynutils.torso.joint_numbers.size()-1]).toString()
                <<" "<<
                ddq.subVector(idynutils.left_arm.joint_numbers[0],
                idynutils.left_arm.joint_numbers[idynutils.left_arm.joint_numbers.size()-1]).toString()<<" ";


        sot2.solve(dq2);
        q2 += dq2;
        ddq2 = dq2 - dq_old2;
        this->_log<<ddq2.subVector(idynutils2.torso.joint_numbers[0],
                idynutils2.torso.joint_numbers[idynutils2.torso.joint_numbers.size()-1]).toString()
                <<" "<<
                ddq2.subVector(idynutils2.left_arm.joint_numbers[0],
                idynutils2.left_arm.joint_numbers[idynutils.left_arm.joint_numbers.size()-1]).toString()<<" ";


        this->_log<<cartesian_task->getb().toString()<<" "<<cartesian_task2->getb().toString()<<std::endl;

        for(unsigned int j = 0; j < ddq.size(); ++j){
            integrator1 += yarp::math::norm(ddq);
            integrator2 += yarp::math::norm(ddq2);
        }
    }
    this->_log<<"]"<<std::endl;

    this->_log<<"for i=1:1:size(ddq,1);"<<
                "nddq1(i) = norm(ddq(i,1:10));"<<
                "nddq2(i) = norm(ddq(i,11:20));"<<
                "nb1(i) = norm(ddq(i,21:26));"<<
                "nb2(i) = norm(ddq(i,27:32));"<<
                "end"<<std::endl;
    this->_log<<"figure(); plot(nddq1, '-r'); hold on; plot(nddq2,'--b'); legend('norm(ddq1)', 'norm(ddq2)');";
    this->_log<<"figure(); plot(nb1, '-r'); hold on; plot(nb2,'--b'); legend('norm(cartesian_error1)', 'norm(cartesian_error2)');";

    std::cout<<"integrator 1 = "<<integrator1<<std::endl;
    std::cout<<"integrator 2 = "<<integrator2<<std::endl;
    ASSERT_TRUE(integrator2 <= integrator1);

    std::cout<<"**************T1*************"<<std::endl;
    idynutils.updateiDyn3Model(q);
    std::cout<<"INITIAL CONFIG: "<<std::endl;cartesian_utils::printHomogeneousTransform(T_init);
    yarp::sig::Matrix T = idynutils.iDyn3_model.getPosition(
                idynutils.iDyn3_model.getLinkIndex("Waist"),
                idynutils.iDyn3_model.getLinkIndex("l_wrist"));
    std::cout<<"FINAL CONFIG: "<<std::endl;cartesian_utils::printHomogeneousTransform(T);
    std::cout<<"DESIRED CONFIG: "<<std::endl;cartesian_utils::printHomogeneousTransform(T_ref);

    for(unsigned int i = 0; i <= 3; ++i){
        for(unsigned int j = 0; j <= 3; ++j)
            EXPECT_NEAR(T(i,j), T_ref(i,j), 1E-3);
    }

    std::cout<<std::endl;

    std::cout<<"**************T2*************"<<std::endl;
    idynutils2.updateiDyn3Model(q2);
    std::cout<<"INITIAL CONFIG: "<<std::endl;cartesian_utils::printHomogeneousTransform(T_init);
    yarp::sig::Matrix T2 = idynutils2.iDyn3_model.getPosition(
                idynutils.iDyn3_model.getLinkIndex("Waist"),
                idynutils.iDyn3_model.getLinkIndex("l_wrist"));
    std::cout<<"FINAL CONFIG: "<<std::endl;cartesian_utils::printHomogeneousTransform(T2);
    std::cout<<"DESIRED CONFIG: "<<std::endl;cartesian_utils::printHomogeneousTransform(T_ref);

    for(unsigned int i = 0; i <= 3; ++i){
        for(unsigned int j = 0; j <= 3; ++j)
            EXPECT_NEAR(T2(i,j), T_ref(i,j), 1E-3);
    }

}
