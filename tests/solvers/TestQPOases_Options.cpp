#include <gtest/gtest.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <qpOASES.hpp>
#include <fstream>
#include <xbot2_interface/xbotinterface2.h>
#include <chrono>
#include "../common.h"

#define GREEN "\033[0;32m"
#define DEFAULT "\033[0m"



namespace{

/** The logged file is organized as:
 *
 *  data = [error_x error_y error_z error_ox error_oy error_oz solve_time]
 *
**/

class testIKProblem
{
public:
    XBot::ModelInterface::Ptr _model_ptr;
    Eigen::Affine3d _T_initial_0;
    std::string _base_link_0;
    std::string _distal_link_0;
    OpenSoT::tasks::velocity::Cartesian::Ptr _cartesian_task_0;
    OpenSoT::tasks::velocity::Postural::Ptr _postural_task;
    OpenSoT::constraints::velocity::JointLimits::Ptr _joint_limits;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr _joint_velocity_limits;
    OpenSoT::constraints::Aggregated::Ptr _joint_constraints;
    OpenSoT::solvers::iHQP::Stack _stack_of_tasks;
    OpenSoT::solvers::iHQP::Ptr _sot;
    Eigen::VectorXd q;
    Eigen::MatrixXd T_ref_0;
    std::ofstream _log;
    std::string _type;
    std::string _variable_name;

    testIKProblem(const double t, const qpOASES::Options &options, const std::string type):
        _base_link_0("Waist"),
        _distal_link_0("l_wrist"),
        _type(type)
    {
        _model_ptr = GetTestModel("coman_floating_base");


        q = getGoodInitialPosition(_model_ptr);
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        _model_ptr->getPose(_distal_link_0,_base_link_0, _T_initial_0);
        _cartesian_task_0 = std::make_shared<OpenSoT::tasks::velocity::Cartesian>("cartesian::left_wrist",
                        *_model_ptr, _distal_link_0, _base_link_0);

        T_ref_0 = _T_initial_0.matrix();
        T_ref_0(0,3) = T_ref_0(0,3) + 0.1;

        _cartesian_task_0->setReference(T_ref_0);
        _cartesian_task_0->update(q);

        _postural_task = std::make_shared<OpenSoT::tasks::velocity::Postural>(*_model_ptr);

        Eigen::VectorXd q_min, q_max;
        _model_ptr->getJointLimits(q_min, q_max);

        _joint_limits = std::make_shared<OpenSoT::constraints::velocity::JointLimits>(*_model_ptr,
                               q_max,q_min);

        _joint_velocity_limits = std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(*_model_ptr, 0.3, (double)(1.0/t));

        std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> joint_constraints_list;
        joint_constraints_list.push_back(_joint_limits);
        joint_constraints_list.push_back(_joint_velocity_limits);

        _joint_constraints = std::make_shared<OpenSoT::constraints::Aggregated>(joint_constraints_list, _model_ptr->getNv());

        _stack_of_tasks.push_back(_cartesian_task_0);
        _stack_of_tasks.push_back(_postural_task);

        _sot = std::make_shared<OpenSoT::solvers::iHQP>(_stack_of_tasks, _joint_constraints);

        for(unsigned int i = 0; i < _sot->getNumberOfTasks(); ++i){
            _sot->setOptions(i, options);
            qpOASES::Options opti;
            boost::any any_opti;
            _sot->getOptions(i, any_opti);
            opti = boost::any_cast<qpOASES::Options>(any_opti);
            opti.print();}

        std::string file_name = "test_option_" + _type + ".m";
        _log.open(file_name.c_str());
        _variable_name = "data_test_" + type;
        _log<<_variable_name<<" = ["<<std::endl;
    }

    double solve(Eigen::VectorXd& dq, bool& solved)
    {
        auto tic = std::chrono::high_resolution_clock::now();
        solved = _sot->solve(dq);
        auto toc = std::chrono::high_resolution_clock::now();

        double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(toc-tic).count()*1e6;
        _log<<dt<<std::endl;
        return dt;
    }

    void update()
    {
        _cartesian_task_0->update(Eigen::VectorXd(0));
        _postural_task->update(Eigen::VectorXd(0));
        _joint_constraints->update(Eigen::VectorXd(0));

        _log<<_cartesian_task_0->getb()<<" ";
    }

    ~testIKProblem()
    {
        _log<<" ];"<<std::endl;

        _log<<"figure(); plot("<<_variable_name<<"(:,1:3)); title('Position Error Task1'); xlabel('sample'); ylabel('m'); legend('e_x','e_y','e_z');"<<std::endl;
        _log<<"figure(); plot("<<_variable_name<<"(:,4:6)); title('Orientation Error Task1'); xlabel('sample'); ylabel('rad'); legend('e_x','e_y','e_z');"<<std::endl;
        _log<<"figure(); plot("<<_variable_name<<"(:,7)); title('Time to solve a single iteration'); xlabel('sample'); ylabel('sec');"<<std::endl;
        _log<<"mean_time = mean("<<_variable_name<<"(:,7))"<<std::endl;

        _log.close();
    }

    Eigen::VectorXd getGoodInitialPosition(XBot::ModelInterface::Ptr _model_ptr) {
        Eigen::VectorXd _q = _model_ptr->getNeutralQ();
        _q[_model_ptr->getDofIndex("RHipSag")+1 ] = -25.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RKneeSag")+1 ] = 50.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RAnkSag")+1 ] = -25.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("LHipSag")+1 ] = -25.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LKneeSag")+1 ] = 50.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LAnkSag")+1 ] = -25.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("LShSag")+1 ] =  20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LShLat")+1 ] = 10.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LElbj")+1 ] = -80.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("RShSag")+1 ] =  20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RShLat")+1 ] = -10.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RElbj")+1 ] = -80.0*M_PI/180.0;
        return _q;
    }




    void checkTask()
    {
        Eigen::Affine3d T_0;
        _model_ptr->getPose(_distal_link_0,_base_link_0, T_0);

        std::cout<<GREEN<<"Arm Initial Pose: "<<_T_initial_0.matrix()<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"Arm Desired Pose: "<<T_ref_0<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"Arm Pose: "<<T_0.matrix()<<DEFAULT<<std::endl;
        for(unsigned int i = 0; i < 3; ++i)
            EXPECT_NEAR(T_0.matrix()(i,3), T_ref_0(i,3), 1E-3);
        for(unsigned int i = 0; i < 3; ++i)
            for(unsigned int j = 0; j < 3; ++j)
                EXPECT_NEAR(T_0.matrix()(i,j), T_ref_0(i,j), 1E-2);
    }
};

/** The logged file is organized as:
 *
 *  data = [error_x1 error_y1 error_z1 error_ox1 error_oy1 error_oz1 error_x2 error_y2 error_z2 error_ox2 error_oy2 error_oz2 solve_time]
 *
**/

class testIKProblem2
{
public:
    XBot::ModelInterface::Ptr _model_ptr;
    Eigen::Affine3d _T_initial_0;
    std::string _base_link_0;
    std::string _distal_link_0;
    Eigen::Affine3d _T_initial_1;
    std::string _base_link_1;
    std::string _distal_link_1;
    OpenSoT::tasks::velocity::Cartesian::Ptr _cartesian_task_0;
    OpenSoT::tasks::velocity::Cartesian::Ptr _cartesian_task_1;
    OpenSoT::tasks::velocity::Postural::Ptr _postural_task;
    OpenSoT::constraints::velocity::JointLimits::Ptr _joint_limits;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr _joint_velocity_limits;
    OpenSoT::constraints::Aggregated::Ptr _joint_constraints;
    OpenSoT::solvers::iHQP::Stack _stack_of_tasks;
    OpenSoT::solvers::iHQP::Ptr _sot;
    Eigen::VectorXd q;
    Eigen::MatrixXd T_ref_0;
    Eigen::MatrixXd T_ref_1;
    std::ofstream _log;
    std::string _type;
    std::string _variable_name;

    testIKProblem2(const double t, const qpOASES::Options &options, const std::string type):
        _base_link_0("Waist"),
        _distal_link_0("l_wrist"),
        _base_link_1("Waist"),
        _distal_link_1("r_wrist"),
        _type(type)
    {
        _model_ptr = GetTestModel("coman_floating_base");



        q = getGoodInitialPosition(_model_ptr);
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        _model_ptr->getPose(_distal_link_0,_base_link_0, _T_initial_0);
        _model_ptr->getPose(_distal_link_1,_base_link_1, _T_initial_1);


        _cartesian_task_0 =std::make_shared<OpenSoT::tasks::velocity::Cartesian>("cartesian::left_wrist",
                    *_model_ptr,
                    _distal_link_0, _base_link_0);
        _cartesian_task_1 = std::make_shared<OpenSoT::tasks::velocity::Cartesian>("cartesian::right_wrist",
                    *_model_ptr,
                    _distal_link_1, _base_link_1);

        T_ref_0 = _T_initial_0.matrix();
        T_ref_0(0,3) = T_ref_0(0,3) + 0.1;

        T_ref_1 = _T_initial_1.matrix();
        T_ref_1(1,3) = T_ref_1(1,3) - 0.05;

        _cartesian_task_0->setReference(T_ref_0);
        _cartesian_task_0->update(q);

        _cartesian_task_1->setReference(T_ref_1);
        _cartesian_task_1->update(q);

        _postural_task = std::make_shared<OpenSoT::tasks::velocity::Postural>(*_model_ptr);

        Eigen::VectorXd q_min, q_max;
        _model_ptr->getJointLimits(q_min, q_max);
        _joint_limits = std::make_shared<OpenSoT::constraints::velocity::JointLimits>(*_model_ptr,
                        q_max,q_min);

        _joint_velocity_limits = std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(*_model_ptr,
                                                                                                  0.3, (double)(1.0/t));

        std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> joint_constraints_list;
        joint_constraints_list.push_back(_joint_limits);
        joint_constraints_list.push_back(_joint_velocity_limits);

        _joint_constraints = std::make_shared<OpenSoT::constraints::Aggregated>(joint_constraints_list, _model_ptr->getNv());

        _stack_of_tasks.push_back(_cartesian_task_0);
        _stack_of_tasks.push_back(_cartesian_task_1);
        _stack_of_tasks.push_back(_postural_task);

        _sot = std::make_shared<OpenSoT::solvers::iHQP>(_stack_of_tasks, _joint_constraints);

        for(unsigned int i = 0; i < _sot->getNumberOfTasks(); ++i){
            _sot->setOptions(i, options);
            qpOASES::Options opti;
            boost::any any_opti;
            _sot->getOptions(i, any_opti);
            opti = boost::any_cast<qpOASES::Options>(any_opti);
            opti.print();}

        std::string file_name = "test_option_" + _type + ".m";
        _log.open(file_name.c_str());
        _variable_name = "data_test_" + type;
        _log<<_variable_name<<" = ["<<std::endl;
    }

    double solve(Eigen::VectorXd& dq, bool& solved)
    {
        auto tic = std::chrono::high_resolution_clock::now();
        solved = _sot->solve(dq);
        auto toc = std::chrono::high_resolution_clock::now();

        double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(toc-tic).count()*1e6;
        _log<<dt<<std::endl;
        return dt;
    }

    void update()
    {
        _cartesian_task_0->update(Eigen::VectorXd(0));
        _cartesian_task_1->update(Eigen::VectorXd(0));
        _postural_task->update(Eigen::VectorXd(0));
        _joint_constraints->update(Eigen::VectorXd(0));

        _log<<_cartesian_task_0->getb()<<" "<<_cartesian_task_1->getb()<<" ";
    }

    ~testIKProblem2()
    {
        _log<<" ];"<<std::endl;

        _log<<"figure(); plot("<<_variable_name<<"(:,1:3)); title('Position Error Task1'); xlabel('sample'); ylabel('m'); legend('e_x','e_y','e_z');"<<std::endl;
        _log<<"figure(); plot("<<_variable_name<<"(:,4:6)); title('Orientation Error Task1'); xlabel('sample'); ylabel('rad'); legend('e_x','e_y','e_z');"<<std::endl;
        _log<<"figure(); plot("<<_variable_name<<"(:,7:9)); title('Position Error Task2'); xlabel('sample'); ylabel('m'); legend('e_x','e_y','e_z');"<<std::endl;
        _log<<"figure(); plot("<<_variable_name<<"(:,10:12)); title('Orientation Error Task2'); xlabel('sample'); ylabel('rad'); legend('e_x','e_y','e_z');"<<std::endl;

        _log<<"figure(); plot("<<_variable_name<<"(:,13)); title('Time to solve a single iteration'); xlabel('sample'); ylabel('sec');"<<std::endl;
        _log<<"mean_time = mean("<<_variable_name<<"(:,13))"<<std::endl;

        _log.close();
    }


    Eigen::VectorXd getGoodInitialPosition(XBot::ModelInterface::Ptr _model_ptr) {
        Eigen::VectorXd _q = _model_ptr->getNeutralQ();
        _q[_model_ptr->getDofIndex("RHipSag") +1] = -25.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RKneeSag")+1] = 50.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RAnkSag")+1] = -25.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("LHipSag")+1] = -25.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LKneeSag")+1] = 50.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LAnkSag")+1] = -25.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("LShSag")+1] =  20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LShLat")+1] = 10.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LElbj")+1] = -80.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("RShSag")+1] =  20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RShLat")+1] = -10.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RElbj")+1] = -80.0*M_PI/180.0;
        return _q;
    }

    void checkTask()
    {
        Eigen::Affine3d T_0;
        _model_ptr->getPose(_distal_link_0,_base_link_0, T_0);


        std::cout<<GREEN<<"TASK1: "<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"Arm Initial Pose: "<<_T_initial_0.matrix()<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"Arm Desired Pose: "<<T_ref_0<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"Arm Pose: "<<T_0.matrix()<<DEFAULT<<std::endl;
        for(unsigned int i = 0; i < 3; ++i)
            EXPECT_NEAR(T_0.matrix()(i,3), T_ref_0(i,3), 1E-3);
        for(unsigned int i = 0; i < 3; ++i)
            for(unsigned int j = 0; j < 3; ++j)
                EXPECT_NEAR(T_0.matrix()(i,j), T_ref_0(i,j), 1E-2);

        Eigen::Affine3d T_1;
        _model_ptr->getPose(_distal_link_1,_base_link_1, T_1);


        std::cout<<std::endl;
        std::cout<<GREEN<<"TASK2: "<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"Arm Initial Pose: "<<_T_initial_1.matrix()<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"Arm Desired Pose: "<<T_ref_1<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"Arm Pose: "<<DEFAULT<<T_1.matrix()<<std::endl;
        for(unsigned int i = 0; i < 3; ++i)
            EXPECT_NEAR(T_1.matrix()(i,3), T_ref_1(i,3), 1E-3);
        for(unsigned int i = 0; i < 3; ++i)
            for(unsigned int j = 0; j < 3; ++j)
                EXPECT_NEAR(T_1.matrix()(i,j), T_ref_1(i,j), 1E-2);
    }
};

/** The logged file is organized as:
 *
 *  data = [error_x1 error_y1 error_z1 error_ox1 error_oy1 error_oz1 error_x2 error_y2 error_z2 error_ox2 error_oy2 error_oz2 solve_time]
 *
**/

class testIKProblem3
{
public:
    XBot::ModelInterface::Ptr _model_ptr;
    Eigen::Affine3d _T_initial_0;
    std::string _base_link_0;
    std::string _distal_link_0;
    Eigen::Affine3d _T_initial_1;
    std::string _base_link_1;
    std::string _distal_link_1;
    OpenSoT::tasks::velocity::Cartesian::Ptr _cartesian_task_0;
    OpenSoT::tasks::velocity::Cartesian::Ptr _cartesian_task_1;
    OpenSoT::tasks::velocity::Postural::Ptr _postural_task;
    OpenSoT::constraints::velocity::JointLimits::Ptr _joint_limits;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr _joint_velocity_limits;
    OpenSoT::constraints::Aggregated::Ptr _joint_constraints;
    OpenSoT::solvers::iHQP::Stack _stack_of_tasks;
    OpenSoT::solvers::iHQP::Ptr _sot;
    Eigen::VectorXd q;
    Eigen::MatrixXd T_ref_0;
    Eigen::MatrixXd T_ref_1;
    std::ofstream _log;
    std::string _type;
    std::string _variable_name;
    OpenSoT::tasks::Aggregated::Ptr _cartesian_tasks;

    testIKProblem3(const double t, const qpOASES::Options &options, const std::string type):
        _base_link_0("Waist"),
        _distal_link_0("l_wrist"),
        _base_link_1("Waist"),
        _distal_link_1("r_wrist"),
        _type(type)
    {
        _model_ptr = GetTestModel("coman_floating_base");


        q = getGoodInitialPosition(_model_ptr);
        _model_ptr->setJointPosition(q);
        _model_ptr->update();

        _model_ptr->getPose(_distal_link_0,_base_link_0, _T_initial_0);
        _model_ptr->getPose(_distal_link_1,_base_link_1, _T_initial_1);
        _cartesian_task_0 = std::make_shared<OpenSoT::tasks::velocity::Cartesian>("cartesian::left_wrist",
                *_model_ptr,
                    _distal_link_0, _base_link_0);
        _cartesian_task_1 = std::make_shared<OpenSoT::tasks::velocity::Cartesian>("cartesian::right_wrist",
                    *_model_ptr,
                    _distal_link_1, _base_link_1);

        T_ref_0 = _T_initial_0.matrix();
        T_ref_0(0,3) = T_ref_0(0,3) + 0.1;

        T_ref_1 = _T_initial_1.matrix();
        T_ref_1(1,3) = T_ref_1(1,3) - 0.05;

        _cartesian_task_0->setReference(T_ref_0);
        _cartesian_task_0->update(q);

        _cartesian_task_1->setReference(T_ref_1);
        _cartesian_task_1->update(q);

        _postural_task = std::make_shared<OpenSoT::tasks::velocity::Postural>(*_model_ptr);

        Eigen::VectorXd q_min, q_max;
        _model_ptr->getJointLimits(q_min, q_max);
        _joint_limits = std::make_shared<OpenSoT::constraints::velocity::JointLimits>(*_model_ptr, q_max,q_min);

        _joint_velocity_limits = std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(*_model_ptr, 0.3, (double)(1.0/t));

        std::list<OpenSoT::Constraint<Eigen::MatrixXd, Eigen::VectorXd>::ConstraintPtr> joint_constraints_list;
        joint_constraints_list.push_back(_joint_limits);
        joint_constraints_list.push_back(_joint_velocity_limits);

        _joint_constraints = std::make_shared<OpenSoT::constraints::Aggregated>(joint_constraints_list, _model_ptr->getNv());

        std::list<OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr> cartesian_task_list;
        cartesian_task_list.push_back(_cartesian_task_0);
        cartesian_task_list.push_back(_cartesian_task_1);

        _cartesian_tasks = std::make_shared<OpenSoT::tasks::Aggregated>(cartesian_task_list, _model_ptr->getNv());

        _stack_of_tasks.push_back(_cartesian_tasks);
        _stack_of_tasks.push_back(_postural_task);

        _sot = std::make_shared<OpenSoT::solvers::iHQP>(_stack_of_tasks, _joint_constraints);

        for(unsigned int i = 0; i < _sot->getNumberOfTasks(); ++i){
            _sot->setOptions(i, options);
            qpOASES::Options opti;
            boost::any any_opti;
            _sot->getOptions(i, any_opti);
            opti = boost::any_cast<qpOASES::Options>(any_opti);
            opti.print();}

        std::string file_name = "test_option_" + _type + ".m";
        _log.open(file_name.c_str());
        _variable_name = "data_test_" + type;
        _log<<_variable_name<<" = ["<<std::endl;
    }

    double solve(Eigen::VectorXd& dq, bool& solved)
    {
        auto tic = std::chrono::high_resolution_clock::now();
        solved = _sot->solve(dq);
        auto toc = std::chrono::high_resolution_clock::now();

        double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(toc-tic).count()*1e6;
        _log<<dt<<std::endl;
        return dt;
    }

    void update()
    {
        _cartesian_tasks->update(Eigen::VectorXd(0));
        _postural_task->update(Eigen::VectorXd(0));
        _joint_constraints->update(Eigen::VectorXd(0));

        _log<<_cartesian_task_0->getb()<<" "<<_cartesian_task_1->getb()<<" ";
    }

    ~testIKProblem3()
    {
        _log<<" ];"<<std::endl;

        _log<<"figure(); plot("<<_variable_name<<"(:,1:3)); title('Position Error Task1'); xlabel('sample'); ylabel('m'); legend('e_x','e_y','e_z');"<<std::endl;
        _log<<"figure(); plot("<<_variable_name<<"(:,4:6)); title('Orientation Error Task1'); xlabel('sample'); ylabel('rad'); legend('e_x','e_y','e_z');"<<std::endl;
        _log<<"figure(); plot("<<_variable_name<<"(:,7:9)); title('Position Error Task2'); xlabel('sample'); ylabel('m'); legend('e_x','e_y','e_z');"<<std::endl;
        _log<<"figure(); plot("<<_variable_name<<"(:,10:12)); title('Orientation Error Task2'); xlabel('sample'); ylabel('rad'); legend('e_x','e_y','e_z');"<<std::endl;

        _log<<"figure(); plot("<<_variable_name<<"(:,13)); title('Time to solve a single iteration'); xlabel('sample'); ylabel('sec');"<<std::endl;

        _log<<"mean_time = mean("<<_variable_name<<"(:,13))"<<std::endl;

        _log.close();
    }


    Eigen::VectorXd getGoodInitialPosition(XBot::ModelInterface::Ptr _model_ptr) {
        Eigen::VectorXd _q = _model_ptr->getNeutralQ();
        _q[_model_ptr->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("LShSag")] =  20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LShLat")] = 10.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("LElbj")] = -80.0*M_PI/180.0;

        _q[_model_ptr->getDofIndex("RShSag")] =  20.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RShLat")] = -10.0*M_PI/180.0;
        _q[_model_ptr->getDofIndex("RElbj")] = -80.0*M_PI/180.0;
        return _q;
    }

    void checkTask()
    {
        Eigen::Affine3d T_0;
        _model_ptr->getPose(_distal_link_0,_base_link_0, T_0);


        std::cout<<GREEN<<"TASK1: "<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"Arm Initial Pose: "<<_T_initial_0.matrix()<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"Arm Desired Pose: "<<T_ref_0<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"Arm Pose: "<<T_0.matrix()<<DEFAULT<<std::endl;
        for(unsigned int i = 0; i < 3; ++i)
            EXPECT_NEAR(T_0.matrix()(i,3), T_ref_0(i,3), 1E-3);
        for(unsigned int i = 0; i < 3; ++i)
            for(unsigned int j = 0; j < 3; ++j)
                EXPECT_NEAR(T_0.matrix()(i,j), T_ref_0(i,j), 1E-2);

        Eigen::Affine3d T_1;
        _model_ptr->getPose(_distal_link_1,_base_link_1, T_1);


        std::cout<<std::endl;
        std::cout<<GREEN<<"TASK2: "<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"Arm Initial Pose: "<<_T_initial_1.matrix()<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"Arm Desired Pose: "<<T_ref_1<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"Arm Pose: "<<DEFAULT<<T_1.matrix()<<std::endl;
        for(unsigned int i = 0; i < 3; ++i)
            EXPECT_NEAR(T_1.matrix()(i,3), T_ref_1(i,3), 1E-3);
        for(unsigned int i = 0; i < 3; ++i)
            for(unsigned int j = 0; j < 3; ++j)
                EXPECT_NEAR(T_1.matrix()(i,j), T_ref_1(i,j), 1E-2);
    }


};

class testQPOases_Options: public ::testing::Test
{
protected:
    testQPOases_Options()
    {

    }

    virtual ~testQPOases_Options() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }
};

TEST_F(testQPOases_Options, testOptionMPC)
{
    unsigned int T = 1000;

    qpOASES::Options opt0;
    opt0.setToMPC();
    opt0.printLevel = qpOASES::PL_NONE;
    opt0.enableRegularisation = qpOASES::BT_TRUE;
    opt0.epsRegularisation *= 2E-2;

    opt0.ensureConsistency();

    testIKProblem test0(double(T), opt0, "mpc_cartesian");

    std::cout<<GREEN<<"test0 options:"<<DEFAULT<<std::endl;


    Eigen::VectorXd dq(test0._model_ptr->getNv());dq.setZero();
    double acc = 0.0;
    for(unsigned int t = 0; t < T; ++t)
    {
        test0._model_ptr->setJointPosition(test0.q);
        test0._model_ptr->update();

        test0.update();

        bool solved;
        acc += test0.solve(dq, solved);
        if(!solved)
            dq.setZero();
        test0.q = test0._model_ptr->sum(test0.q, dq);
    }
    acc = acc/double(T);
    std::cout<<GREEN<<"Average time to solve one step is: "<<acc<<" [s]"<<DEFAULT<<std::endl;

    test0.checkTask();


    testIKProblem2 test1(double(T), opt0, "mpc_2cartesians_stacked");

    std::cout<<GREEN<<"test1 options:"<<DEFAULT<<std::endl;

    dq.setZero();
    acc = 0.0;
    for(unsigned int t = 0; t < T; ++t)
    {
        test1._model_ptr->setJointPosition(test1.q);
        test1._model_ptr->update();

        test1.update();

        bool solved;
        acc += test1.solve(dq, solved);
        if(!solved)
            dq.setZero();
        test1.q = test1._model_ptr->sum(test1.q, dq);
    }
    acc = acc/double(T);
    std::cout<<GREEN<<"Average time to solve one step is: "<<acc<<" [s]"<<DEFAULT<<std::endl;

    test1.checkTask();


    testIKProblem3 test2(double(T), opt0, "mpc_2cartesians_augmented");

    std::cout<<GREEN<<"test2 options:"<<DEFAULT<<std::endl;

    dq.setZero();
    acc = 0.0;
    for(unsigned int t = 0; t < T; ++t)
    {
        test2._model_ptr->setJointPosition(test2.q);
        test2._model_ptr->update();

        test2.update();

        bool solved;
        acc += test2.solve(dq, solved);
        if(!solved)
            dq.setZero();
        test2.q = test2._model_ptr->sum(test2.q, dq);
    }
    acc = acc/double(T);
    std::cout<<GREEN<<"Average time to solve one step is: "<<acc<<" [s]"<<DEFAULT<<std::endl;

    test2.checkTask();
}

TEST_F(testQPOases_Options, testOptionReliable)
{
    unsigned int T = 1000;

    qpOASES::Options opt0;
    opt0.setToReliable();
    opt0.initialStatusBounds = qpOASES::ST_INACTIVE;
    opt0.printLevel = qpOASES::PL_NONE;
    opt0.enableRegularisation = qpOASES::BT_TRUE;
    opt0.epsRegularisation *= 2E-2;

    opt0.ensureConsistency();

    testIKProblem test0(double(T), opt0,"reliable_cartesian");

    std::cout<<GREEN<<"test0 options:"<<DEFAULT<<std::endl;

    Eigen::VectorXd dq(test0.q.size());dq.setZero(dq.size());
    double acc = 0.0;
    for(unsigned int t = 0; t < T; ++t)
    {
        test0._model_ptr->setJointPosition(test0.q);
        test0._model_ptr->update();

        test0.update();

        bool solved;
        acc += test0.solve(dq, solved);
        if(!solved)
            dq.setZero(dq.size());
        test0.q = test0._model_ptr->sum(test0.q, dq);
    }
    acc = acc/double(T);
    std::cout<<GREEN<<"Average time to solve one step is: "<<acc<<" [s]"<<DEFAULT<<std::endl;

    test0.checkTask();

    testIKProblem2 test1(double(T), opt0, "reliable_2cartesians_stacked");

    std::cout<<GREEN<<"test1 options:"<<DEFAULT<<std::endl;

    dq.setZero(dq.size());
    acc = 0.0;
    for(unsigned int t = 0; t < T; ++t)
    {
        test1._model_ptr->setJointPosition(test1.q);
        test1._model_ptr->update();
        test1.update();

        bool solved;
        acc += test1.solve(dq, solved);
        if(!solved)
            dq.setZero(dq.size());
        test1.q = test1._model_ptr->sum(test1.q, dq);
    }
    acc = acc/double(T);
    std::cout<<GREEN<<"Average time to solve one step is: "<<acc<<" [s]"<<DEFAULT<<std::endl;

    test1.checkTask();

    testIKProblem3 test2(double(T), opt0, "reliable_2cartesians_augmented");

    std::cout<<GREEN<<"test2 options:"<<DEFAULT<<std::endl;

    dq.setZero(dq.size());
    acc = 0.0;
    for(unsigned int t = 0; t < T; ++t)
    {
        test2._model_ptr->setJointPosition(test2.q);
        test2._model_ptr->update();
        test2.update();

        bool solved;
        acc += test2.solve(dq, solved);
        if(!solved)
            dq.setZero(dq.size());
        test2.q = test2._model_ptr->sum(test2.q, dq);
    }
    acc = acc/double(T);
    std::cout<<GREEN<<"Average time to solve one step is: "<<acc<<" [s]"<<DEFAULT<<std::endl;

    test2.checkTask();
}

TEST_F(testQPOases_Options, testOptionDefault)
{
    unsigned int T = 1000;

    qpOASES::Options opt0;
    opt0.setToDefault();
    opt0.printLevel = qpOASES::PL_NONE;
    opt0.initialStatusBounds = qpOASES::ST_INACTIVE;
    opt0.enableRegularisation = qpOASES::BT_TRUE;
    opt0.epsRegularisation *= 2E-2;

    opt0.ensureConsistency();

    testIKProblem test0(double(T), opt0, "default_cartesian");

    std::cout<<GREEN<<"test0 options:"<<DEFAULT<<std::endl;

    Eigen::VectorXd dq(test0.q.size());dq.setZero(dq.size());
    double acc = 0.0;
    for(unsigned int t = 0; t < T; ++t)
    {
        test0._model_ptr->setJointPosition(test0.q);
        test0._model_ptr->update();

        test0.update();

        bool solved;
        acc += test0.solve(dq, solved);
        if(!solved)
            dq.setZero(dq.size());
        test0.q = test0._model_ptr->sum(test0.q, dq);
    }
    acc = acc/double(T);
    std::cout<<GREEN<<"Average time to solve one step is: "<<acc<<" [s]"<<DEFAULT<<std::endl;

    test0.checkTask();

    testIKProblem2 test1(double(T), opt0, "default_2cartesians_stacked");

    std::cout<<GREEN<<"test1 options:"<<DEFAULT<<std::endl;

    dq.setZero(dq.size());
    acc = 0.0;
    for(unsigned int t = 0; t < T; ++t)
    {
        test1._model_ptr->setJointPosition(test1.q);
        test1._model_ptr->update();
        test1.update();

        bool solved;
        acc += test1.solve(dq, solved);
        if(!solved)
            dq.setZero(dq.size());
        test1.q = test1._model_ptr->sum(test1.q, dq);
    }
    acc = acc/double(T);
    std::cout<<GREEN<<"Average time to solve one step is: "<<acc<<" [s]"<<DEFAULT<<std::endl;

    test1.checkTask();

    testIKProblem3 test2(double(T), opt0, "default_2cartesians_augmented");

    std::cout<<GREEN<<"test2 options:"<<DEFAULT<<std::endl;

    dq.setZero(dq.size());
    acc = 0.0;
    for(unsigned int t = 0; t < T; ++t)
    {
        test2._model_ptr->setJointPosition(test2.q);
        test2._model_ptr->update();

        test2.update();

        bool solved;
        acc += test2.solve(dq, solved);
        if(!solved)
            dq.setZero(dq.size());
        test2.q = test2._model_ptr->sum(test2.q, dq);
    }
    acc = acc/double(T);
    std::cout<<GREEN<<"Average time to solve one step is: "<<acc<<" [s]"<<DEFAULT<<std::endl;

    test2.checkTask();
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
