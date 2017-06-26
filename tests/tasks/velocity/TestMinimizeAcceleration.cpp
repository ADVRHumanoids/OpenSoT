#include <advr_humanoids_common_utils/conversion_utils_YARP.h>
#include <gtest/gtest.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/solvers/QPOases.h>
#include <OpenSoT/tasks/velocity/MinimizeAcceleration.h>
#include <qpOASES.hpp>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>
#include <fstream>
#include <ModelInterfaceIDYNUTILS/ModelInterfaceIDYNUTILS.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>

using namespace OpenSoT::constraints;
using namespace OpenSoT::tasks::velocity;
using namespace yarp::math;

typedef idynutils2 iDynUtils;
static void null_deleter(iDynUtils *) {}

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
    yarp::sig::Vector q(idynutils.iDynTree_model.getNrOfDOFs(), 0.0);
    q[idynutils.iDynTree_model.getDOFIndex("RHipSag")] = -25.0*M_PI/180.0;
    q[idynutils.iDynTree_model.getDOFIndex("RKneeSag")] = 50.0*M_PI/180.0;
    q[idynutils.iDynTree_model.getDOFIndex("RAnkSag")] = -25.0*M_PI/180.0;

    q[idynutils.iDynTree_model.getDOFIndex("LShSag")] =  20.0*M_PI/180.0;
    q[idynutils.iDynTree_model.getDOFIndex("LShLat")] = 10.0*M_PI/180.0;
    q[idynutils.iDynTree_model.getDOFIndex("LElbj")] = -80.0*M_PI/180.0;

    q[idynutils.iDynTree_model.getDOFIndex("RShSag")] =  20.0*M_PI/180.0;
    q[idynutils.iDynTree_model.getDOFIndex("RShLat")] = -10.0*M_PI/180.0;
    q[idynutils.iDynTree_model.getDOFIndex("RElbj")] = -80.0*M_PI/180.0;

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

    XBot::ModelInterfaceIDYNUTILS::Ptr _model_ptr;
    std::string robotology_root = std::getenv("ROBOTOLOGY_ROOT");
    std::string relative_path = "/external/OpenSoT/tests/configs/coman/configs/config_coman.yaml";

    std::string _path_to_cfg = robotology_root + relative_path;

    _model_ptr = std::dynamic_pointer_cast<XBot::ModelInterfaceIDYNUTILS>
            (XBot::ModelInterface::getModel(_path_to_cfg));
    _model_ptr->loadModel(boost::shared_ptr<iDynUtils>(&idynutils, &null_deleter));

    if(_model_ptr)
        std::cout<<"pointer address: "<<_model_ptr.get()<<std::endl;
    else
        std::cout<<"pointer is NULL "<<_model_ptr.get()<<std::endl;

    XBot::ModelInterfaceIDYNUTILS::Ptr _model_ptr2;

    _model_ptr2 = std::dynamic_pointer_cast<XBot::ModelInterfaceIDYNUTILS>
            (XBot::ModelInterface::getModel(_path_to_cfg));
    _model_ptr2->loadModel(boost::shared_ptr<iDynUtils>(&idynutils2, &null_deleter));

    if(_model_ptr2)
        std::cout<<"pointer address: "<<_model_ptr2.get()<<std::endl;
    else
        std::cout<<"pointer is NULL "<<_model_ptr2.get()<<std::endl;

    yarp::sig::Vector q = getGoodInitialPosition(idynutils);

    idynutils.updateiDynTreeModel(conversion_utils_YARP::toEigen(q), true);
    idynutils2.updateiDynTreeModel(conversion_utils_YARP::toEigen(q), true);

    /// Cartesian Task
    yarp::sig::Matrix T_init = idynutils.iDynTree_model.getPosition(
                          idynutils.iDynTree_model.getLinkIndex("Waist"),
                          idynutils.iDynTree_model.getLinkIndex("l_wrist"));
    OpenSoT::tasks::velocity::Cartesian::Ptr cartesian_task(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist",
                              conversion_utils_YARP::toEigen(q), *(_model_ptr.get()),"l_wrist", "Waist"));
    yarp::sig::Matrix T_ref = T_init;
    T_ref(0,3) = T_ref(0,3) + 0.1;
    T_ref(1,3) = T_ref(1,3) + 0.1;
    T_ref(2,3) = T_ref(2,3) + 0.1;
    cartesian_task->setReference(conversion_utils_YARP::toEigen(T_ref));
    cartesian_task->update(conversion_utils_YARP::toEigen(q));

    OpenSoT::tasks::velocity::Cartesian::Ptr cartesian_task2(
                new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist",
                                    conversion_utils_YARP::toEigen(q), *(_model_ptr2.get()),"l_wrist", "Waist"));
    yarp::sig::Matrix T_ref2 = T_init;
    T_ref2(0,3) = T_ref2(0,3) + 0.1;
    T_ref2(1,3) = T_ref2(1,3) + 0.1;
    T_ref2(2,3) = T_ref2(2,3) + 0.1;
    cartesian_task2->setReference(conversion_utils_YARP::toEigen(T_ref2));
    cartesian_task2->update(conversion_utils_YARP::toEigen(q));

    /// Postural Task
    OpenSoT::tasks::velocity::Postural::Ptr postural_task(
                new OpenSoT::tasks::velocity::Postural(conversion_utils_YARP::toEigen(q)));
    postural_task->setReference(conversion_utils_YARP::toEigen(q));

    /// MinAcc Task
    OpenSoT::tasks::velocity::MinimizeAcceleration::Ptr minacc_task(
                new OpenSoT::tasks::velocity::MinimizeAcceleration(conversion_utils_YARP::toEigen(q)));

    int t = 100;
    /// Constraints set to the Cartesian Task
    OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits(
        new OpenSoT::constraints::velocity::JointLimits(
                    conversion_utils_YARP::toEigen(q), idynutils.getJointBoundMax(),
                           idynutils.getJointBoundMin(), 0.2));

    OpenSoT::constraints::velocity::JointLimits::Ptr joint_limits2(
        new OpenSoT::constraints::velocity::JointLimits(
                    conversion_utils_YARP::toEigen(q), idynutils2.getJointBoundMax(),
                           idynutils2.getJointBoundMin(), 0.2));

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

        idynutils.updateiDynTreeModel(conversion_utils_YARP::toEigen(q), true);
        idynutils2.updateiDynTreeModel(conversion_utils_YARP::toEigen(q2), true);

        cartesian_task->update(conversion_utils_YARP::toEigen(q));
        postural_task->update(conversion_utils_YARP::toEigen(q));
        joint_constraints->update(conversion_utils_YARP::toEigen(q));

        cartesian_task2->update(conversion_utils_YARP::toEigen(q2));
        minacc_task->update(conversion_utils_YARP::toEigen(q2));
        for(unsigned int i = 0; i < dq_old2.size(); ++i)
            ASSERT_NEAR(minacc_task->getb()[i], dq_old2[i], 1E-14);
        joint_constraints2->update(conversion_utils_YARP::toEigen(q2));

        Eigen::VectorXd _dq(dq.size());
        _dq.setZero(dq.size());
        sot.solve(_dq);
        dq = conversion_utils_YARP::toYARP(_dq);
        q += dq;
        ddq = dq - dq_old;
        this->_log<<ddq.subVector(idynutils.iDynTree_model.getDOFIndex("WaistLat"),
                idynutils.iDynTree_model.getDOFIndex("WaistYaw")).toString()
                <<" "<<
                ddq.subVector(idynutils.iDynTree_model.getDOFIndex("LShSag"),
                idynutils.iDynTree_model.getDOFIndex("LWrj2")).toString()<<" ";

        Eigen::VectorXd _dq2(dq2.size());
        _dq2.setZero(dq2.size());
        sot2.solve(_dq2);
        dq2 = conversion_utils_YARP::toYARP(_dq2);
        q2 += dq2;
        ddq2 = dq2 - dq_old2;
        this->_log<<ddq2.subVector(idynutils2.iDynTree_model.getDOFIndex("WaistLat"),
                idynutils2.iDynTree_model.getDOFIndex("WaistYaw")).toString()
                <<" "<<
                ddq2.subVector(idynutils2.iDynTree_model.getDOFIndex("LShSag"),
                idynutils2.iDynTree_model.getDOFIndex("LWrj2")).toString()<<" ";

        this->_log<<cartesian_task->getb()<<" "<<cartesian_task2->getb()<<std::endl;

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
    idynutils.updateiDynTreeModel(conversion_utils_YARP::toEigen(q));
    std::cout<<"INITIAL CONFIG: "<<
               T_init.toString()<<std::endl;
    yarp::sig::Matrix T = idynutils.iDynTree_model.getPosition(
                idynutils.iDynTree_model.getLinkIndex("Waist"),
                idynutils.iDynTree_model.getLinkIndex("l_wrist"));
    std::cout<<"FINAL CONFIG: "<<T.toString()<<std::endl;
    std::cout<<"DESIRED CONFIG: "<<T_ref.toString()<<std::endl;

    for(unsigned int i = 0; i <= 3; ++i){
        for(unsigned int j = 0; j <= 3; ++j)
            EXPECT_NEAR(T(i,j), T_ref(i,j), 1E-3);
    }

    std::cout<<std::endl;

    std::cout<<"**************T2*************"<<std::endl;
    idynutils2.updateiDynTreeModel(conversion_utils_YARP::toEigen(q2));
    std::cout<<"INITIAL CONFIG: "<<T_init.toString()<<std::endl;
    yarp::sig::Matrix T2 = idynutils2.iDynTree_model.getPosition(
                idynutils.iDynTree_model.getLinkIndex("Waist"),
                idynutils.iDynTree_model.getLinkIndex("l_wrist"));
    std::cout<<"FINAL CONFIG: "<<T2.toString()<<std::endl;
    std::cout<<"DESIRED CONFIG: "<<T_ref.toString()<<std::endl;

    for(unsigned int i = 0; i <= 3; ++i){
        for(unsigned int j = 0; j <= 3; ++j)
            EXPECT_NEAR(T2(i,j), T_ref(i,j), 1E-3);
    }

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
