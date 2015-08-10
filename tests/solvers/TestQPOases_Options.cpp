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
#include <qpOASES.hpp>
#include <yarp/math/Math.h>
#include <yarp/sig/all.h>
#include <fstream>

using namespace yarp::math;

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
    iDynUtils _idynutils;
    yarp::sig::Matrix _T_initial_0;
    std::string _base_link_0;
    std::string _distal_link_0;
    OpenSoT::tasks::velocity::Cartesian::Ptr _cartesian_task_0;
    OpenSoT::tasks::velocity::Postural::Ptr _postural_task;
    OpenSoT::constraints::velocity::JointLimits::Ptr _joint_limits;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr _joint_velocity_limits;
    OpenSoT::constraints::Aggregated::Ptr _joint_constraints;
    OpenSoT::solvers::QPOases_sot::Stack _stack_of_tasks;
    OpenSoT::solvers::QPOases_sot::Ptr _sot;
    yarp::sig::Vector q;
    yarp::sig::Matrix T_ref_0;
    std::ofstream _log;
    std::string _type;
    std::string _variable_name;

    testIKProblem(const double t, const qpOASES::Options &options, const std::string type):
        _idynutils("coman", std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf"),
        _base_link_0("Waist"),
        _distal_link_0("l_wrist"),
        _type(type)
    {
        q = getGoodInitialPosition(_idynutils);
        _idynutils.updateiDyn3Model(q, true);

        _T_initial_0 = _idynutils.iDyn3_model.getPosition(
                              _idynutils.iDyn3_model.getLinkIndex(_base_link_0),
                              _idynutils.iDyn3_model.getLinkIndex(_distal_link_0));
        _cartesian_task_0 = OpenSoT::tasks::velocity::Cartesian::Ptr(
                    new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist", q, _idynutils,
                    _distal_link_0, _base_link_0));

        T_ref_0 = _T_initial_0;
        T_ref_0(0,3) = T_ref_0(0,3) + 0.1;

        _cartesian_task_0->setReference(T_ref_0);
        _cartesian_task_0->update(q);

        _postural_task = OpenSoT::tasks::velocity::Postural::Ptr(new OpenSoT::tasks::velocity::Postural(q));

        _joint_limits = OpenSoT::constraints::velocity::JointLimits::Ptr(
                    new OpenSoT::constraints::velocity::JointLimits(q, _idynutils.iDyn3_model.getJointBoundMax(),
                               _idynutils.iDyn3_model.getJointBoundMin()));

        _joint_velocity_limits = OpenSoT::constraints::velocity::VelocityLimits::Ptr(
                    new OpenSoT::constraints::velocity::VelocityLimits(0.3, (double)(1.0/t), q.size()));

        std::list<OpenSoT::Constraint<Matrix, Vector>::ConstraintPtr> joint_constraints_list;
        joint_constraints_list.push_back(_joint_limits);
        joint_constraints_list.push_back(_joint_velocity_limits);

        _joint_constraints = OpenSoT::constraints::Aggregated::Ptr(
                    new OpenSoT::constraints::Aggregated(joint_constraints_list, q.size()));

        _stack_of_tasks.push_back(_cartesian_task_0);
        _stack_of_tasks.push_back(_postural_task);

        _sot = OpenSoT::solvers::QPOases_sot::Ptr(new OpenSoT::solvers::QPOases_sot(_stack_of_tasks, _joint_constraints));

        for(unsigned int i = 0; i < _sot->getNumberOfTasks(); ++i){
            _sot->setOptions(i, options);
            qpOASES::Options opti;
            _sot->getOptions(i, opti);
            opti.print();}

        std::string file_name = "test_option_" + _type + ".m";
        _log.open(file_name.c_str());
        _variable_name = "data_test_" + type;
        _log<<_variable_name<<" = ["<<std::endl;
    }

    double solve(yarp::sig::Vector& dq, bool& solved)
    {
        double tic = yarp::os::Time::now();
        solved = _sot->solve(dq);
        double toc = yarp::os::Time::now();

        double dt = toc - tic;
        _log<<dt<<std::endl;
        return dt;
    }

    void update(const yarp::sig::Vector& q)
    {
        _cartesian_task_0->update(q);
        _postural_task->update(q);
        _joint_constraints->update(q);

        _log<<_cartesian_task_0->getb().toString()<<" ";
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


    yarp::sig::Vector getGoodInitialPosition(iDynUtils& idynutils)
    {
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

    void checkTask()
    {
        yarp::sig::Matrix T_0 = _idynutils.iDyn3_model.getPosition(
                              _idynutils.iDyn3_model.getLinkIndex(_base_link_0),
                              _idynutils.iDyn3_model.getLinkIndex(_distal_link_0));

        std::cout<<GREEN<<"Arm Initial Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(_T_initial_0);
        std::cout<<GREEN<<"Arm Desired Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(T_ref_0);
        std::cout<<GREEN<<"Arm Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(T_0);
        for(unsigned int i = 0; i < 3; ++i)
            EXPECT_NEAR(T_0(i,3), T_ref_0(i,3), 1E-3);
        for(unsigned int i = 0; i < 3; ++i)
            for(unsigned int j = 0; j < 3; ++j)
                EXPECT_NEAR(T_0(i,j), T_ref_0(i,j), 1E-2);
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
    iDynUtils _idynutils;
    yarp::sig::Matrix _T_initial_0;
    std::string _base_link_0;
    std::string _distal_link_0;
    yarp::sig::Matrix _T_initial_1;
    std::string _base_link_1;
    std::string _distal_link_1;
    OpenSoT::tasks::velocity::Cartesian::Ptr _cartesian_task_0;
    OpenSoT::tasks::velocity::Cartesian::Ptr _cartesian_task_1;
    OpenSoT::tasks::velocity::Postural::Ptr _postural_task;
    OpenSoT::constraints::velocity::JointLimits::Ptr _joint_limits;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr _joint_velocity_limits;
    OpenSoT::constraints::Aggregated::Ptr _joint_constraints;
    OpenSoT::solvers::QPOases_sot::Stack _stack_of_tasks;
    OpenSoT::solvers::QPOases_sot::Ptr _sot;
    yarp::sig::Vector q;
    yarp::sig::Matrix T_ref_0;
    yarp::sig::Matrix T_ref_1;
    std::ofstream _log;
    std::string _type;
    std::string _variable_name;

    testIKProblem2(const double t, const qpOASES::Options &options, const std::string type):
        _idynutils("coman", std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf"),
        _base_link_0("Waist"),
        _distal_link_0("l_wrist"),
        _base_link_1("Waist"),
        _distal_link_1("r_wrist"),
        _type(type)
    {
        q = getGoodInitialPosition(_idynutils);
        _idynutils.updateiDyn3Model(q, true);

        _T_initial_0 = _idynutils.iDyn3_model.getPosition(
                              _idynutils.iDyn3_model.getLinkIndex(_base_link_0),
                              _idynutils.iDyn3_model.getLinkIndex(_distal_link_0));
        _T_initial_1 = _idynutils.iDyn3_model.getPosition(
                              _idynutils.iDyn3_model.getLinkIndex(_base_link_1),
                              _idynutils.iDyn3_model.getLinkIndex(_distal_link_1));
        _cartesian_task_0 = OpenSoT::tasks::velocity::Cartesian::Ptr(
                    new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist", q, _idynutils,
                    _distal_link_0, _base_link_0));
        _cartesian_task_1 = OpenSoT::tasks::velocity::Cartesian::Ptr(
                    new OpenSoT::tasks::velocity::Cartesian("cartesian::right_wrist", q, _idynutils,
                    _distal_link_1, _base_link_1));

        T_ref_0 = _T_initial_0;
        T_ref_0(0,3) = T_ref_0(0,3) + 0.1;

        T_ref_1 = _T_initial_1;
        T_ref_1(1,3) = T_ref_1(1,3) - 0.05;

        _cartesian_task_0->setReference(T_ref_0);
        _cartesian_task_0->update(q);

        _cartesian_task_1->setReference(T_ref_1);
        _cartesian_task_1->update(q);

        _postural_task = OpenSoT::tasks::velocity::Postural::Ptr(new OpenSoT::tasks::velocity::Postural(q));

        _joint_limits = OpenSoT::constraints::velocity::JointLimits::Ptr(
                    new OpenSoT::constraints::velocity::JointLimits(q, _idynutils.iDyn3_model.getJointBoundMax(),
                               _idynutils.iDyn3_model.getJointBoundMin()));

        _joint_velocity_limits = OpenSoT::constraints::velocity::VelocityLimits::Ptr(
                    new OpenSoT::constraints::velocity::VelocityLimits(0.3, (double)(1.0/t), q.size()));

        std::list<OpenSoT::Constraint<Matrix, Vector>::ConstraintPtr> joint_constraints_list;
        joint_constraints_list.push_back(_joint_limits);
        joint_constraints_list.push_back(_joint_velocity_limits);

        _joint_constraints = OpenSoT::constraints::Aggregated::Ptr(
                    new OpenSoT::constraints::Aggregated(joint_constraints_list, q.size()));

        _stack_of_tasks.push_back(_cartesian_task_0);
        _stack_of_tasks.push_back(_cartesian_task_1);
        _stack_of_tasks.push_back(_postural_task);

        _sot = OpenSoT::solvers::QPOases_sot::Ptr(new OpenSoT::solvers::QPOases_sot(_stack_of_tasks, _joint_constraints));

        for(unsigned int i = 0; i < _sot->getNumberOfTasks(); ++i){
            _sot->setOptions(i, options);
            qpOASES::Options opti;
            _sot->getOptions(i, opti);
            opti.print();}

        std::string file_name = "test_option_" + _type + ".m";
        _log.open(file_name.c_str());
        _variable_name = "data_test_" + type;
        _log<<_variable_name<<" = ["<<std::endl;
    }

    double solve(yarp::sig::Vector& dq, bool& solved)
    {
        double tic = yarp::os::Time::now();
        solved = _sot->solve(dq);
        double toc = yarp::os::Time::now();

        double dt = toc - tic;
        _log<<dt<<std::endl;
        return dt;
    }

    void update(const yarp::sig::Vector& q)
    {
        _cartesian_task_0->update(q);
        _cartesian_task_1->update(q);
        _postural_task->update(q);
        _joint_constraints->update(q);

        _log<<_cartesian_task_0->getb().toString()<<" "<<_cartesian_task_1->getb().toString()<<" ";
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


    yarp::sig::Vector getGoodInitialPosition(iDynUtils& idynutils)
    {
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

    void checkTask()
    {
        yarp::sig::Matrix T_0 = _idynutils.iDyn3_model.getPosition(
                              _idynutils.iDyn3_model.getLinkIndex(_base_link_0),
                              _idynutils.iDyn3_model.getLinkIndex(_distal_link_0));

        std::cout<<GREEN<<"TASK1: "<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"Arm Initial Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(_T_initial_0);
        std::cout<<GREEN<<"Arm Desired Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(T_ref_0);
        std::cout<<GREEN<<"Arm Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(T_0);
        for(unsigned int i = 0; i < 3; ++i)
            EXPECT_NEAR(T_0(i,3), T_ref_0(i,3), 1E-3);
        for(unsigned int i = 0; i < 3; ++i)
            for(unsigned int j = 0; j < 3; ++j)
                EXPECT_NEAR(T_0(i,j), T_ref_0(i,j), 1E-2);

        yarp::sig::Matrix T_1 = _idynutils.iDyn3_model.getPosition(
                              _idynutils.iDyn3_model.getLinkIndex(_base_link_1),
                              _idynutils.iDyn3_model.getLinkIndex(_distal_link_1));

        std::cout<<std::endl;
        std::cout<<GREEN<<"TASK2: "<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"Arm Initial Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(_T_initial_1);
        std::cout<<GREEN<<"Arm Desired Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(T_ref_1);
        std::cout<<GREEN<<"Arm Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(T_1);
        for(unsigned int i = 0; i < 3; ++i)
            EXPECT_NEAR(T_1(i,3), T_ref_1(i,3), 1E-3);
        for(unsigned int i = 0; i < 3; ++i)
            for(unsigned int j = 0; j < 3; ++j)
                EXPECT_NEAR(T_1(i,j), T_ref_1(i,j), 1E-2);
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
    iDynUtils _idynutils;
    yarp::sig::Matrix _T_initial_0;
    std::string _base_link_0;
    std::string _distal_link_0;
    yarp::sig::Matrix _T_initial_1;
    std::string _base_link_1;
    std::string _distal_link_1;
    OpenSoT::tasks::velocity::Cartesian::Ptr _cartesian_task_0;
    OpenSoT::tasks::velocity::Cartesian::Ptr _cartesian_task_1;
    OpenSoT::tasks::velocity::Postural::Ptr _postural_task;
    OpenSoT::constraints::velocity::JointLimits::Ptr _joint_limits;
    OpenSoT::constraints::velocity::VelocityLimits::Ptr _joint_velocity_limits;
    OpenSoT::constraints::Aggregated::Ptr _joint_constraints;
    OpenSoT::tasks::Aggregated::Ptr _cartesian_tasks;
    OpenSoT::solvers::QPOases_sot::Stack _stack_of_tasks;
    OpenSoT::solvers::QPOases_sot::Ptr _sot;
    yarp::sig::Vector q;
    yarp::sig::Matrix T_ref_0;
    yarp::sig::Matrix T_ref_1;
    std::ofstream _log;
    std::string _type;
    std::string _variable_name;

    testIKProblem3(const double t, const qpOASES::Options &options, const std::string type):
        _idynutils("coman", std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.urdf",
            std::string(OPENSOT_TESTS_ROBOTS_DIR)+"coman/coman.srdf"),
        _base_link_0("Waist"),
        _distal_link_0("l_wrist"),
        _base_link_1("Waist"),
        _distal_link_1("r_wrist"),
        _type(type)
    {
        q = getGoodInitialPosition(_idynutils);
        _idynutils.updateiDyn3Model(q, true);

        _T_initial_0 = _idynutils.iDyn3_model.getPosition(
                              _idynutils.iDyn3_model.getLinkIndex(_base_link_0),
                              _idynutils.iDyn3_model.getLinkIndex(_distal_link_0));
        _T_initial_1 = _idynutils.iDyn3_model.getPosition(
                              _idynutils.iDyn3_model.getLinkIndex(_base_link_1),
                              _idynutils.iDyn3_model.getLinkIndex(_distal_link_1));
        _cartesian_task_0 = OpenSoT::tasks::velocity::Cartesian::Ptr(
                    new OpenSoT::tasks::velocity::Cartesian("cartesian::left_wrist", q, _idynutils,
                    _distal_link_0, _base_link_0));
        _cartesian_task_1 = OpenSoT::tasks::velocity::Cartesian::Ptr(
                    new OpenSoT::tasks::velocity::Cartesian("cartesian::right_wrist", q, _idynutils,
                    _distal_link_1, _base_link_1));

        T_ref_0 = _T_initial_0;
        T_ref_0(0,3) = T_ref_0(0,3) + 0.1;

        T_ref_1 = _T_initial_1;
        T_ref_1(1,3) = T_ref_1(1,3) - 0.05;

        _cartesian_task_0->setReference(T_ref_0);
        _cartesian_task_0->update(q);

        _cartesian_task_1->setReference(T_ref_1);
        _cartesian_task_1->update(q);

        _postural_task = OpenSoT::tasks::velocity::Postural::Ptr(new OpenSoT::tasks::velocity::Postural(q));

        _joint_limits = OpenSoT::constraints::velocity::JointLimits::Ptr(
                    new OpenSoT::constraints::velocity::JointLimits(q, _idynutils.iDyn3_model.getJointBoundMax(),
                               _idynutils.iDyn3_model.getJointBoundMin()));

        _joint_velocity_limits = OpenSoT::constraints::velocity::VelocityLimits::Ptr(
                    new OpenSoT::constraints::velocity::VelocityLimits(0.3, (double)(1.0/t), q.size()));

        std::list<OpenSoT::Constraint<Matrix, Vector>::ConstraintPtr> joint_constraints_list;
        joint_constraints_list.push_back(_joint_limits);
        joint_constraints_list.push_back(_joint_velocity_limits);

        _joint_constraints = OpenSoT::constraints::Aggregated::Ptr(
                    new OpenSoT::constraints::Aggregated(joint_constraints_list, q.size()));

        std::list<OpenSoT::Task<Matrix, Vector>::TaskPtr> cartesian_task_list;
        cartesian_task_list.push_back(_cartesian_task_0);
        cartesian_task_list.push_back(_cartesian_task_1);

        _cartesian_tasks = OpenSoT::tasks::Aggregated::Ptr(
                    new OpenSoT::tasks::Aggregated(cartesian_task_list, q.size()));

        _stack_of_tasks.push_back(_cartesian_tasks);
        _stack_of_tasks.push_back(_postural_task);

        _sot = OpenSoT::solvers::QPOases_sot::Ptr(new OpenSoT::solvers::QPOases_sot(_stack_of_tasks, _joint_constraints));

        for(unsigned int i = 0; i < _sot->getNumberOfTasks(); ++i){
            _sot->setOptions(i, options);
            qpOASES::Options opti;
            _sot->getOptions(i, opti);
            opti.print();}

        std::string file_name = "test_option_" + _type + ".m";
        _log.open(file_name.c_str());
        _variable_name = "data_test_" + type;
        _log<<_variable_name<<" = ["<<std::endl;
    }

    double solve(yarp::sig::Vector& dq, bool& solved)
    {
        double tic = yarp::os::Time::now();
        solved = _sot->solve(dq);
        double toc = yarp::os::Time::now();

        double dt = toc - tic;
        _log<<dt<<std::endl;
        return dt;
    }

    void update(const yarp::sig::Vector& q)
    {
        _cartesian_tasks->update(q);
        _postural_task->update(q);
        _joint_constraints->update(q);

        _log<<_cartesian_task_0->getb().toString()<<" "<<_cartesian_task_1->getb().toString()<<" ";
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


    yarp::sig::Vector getGoodInitialPosition(iDynUtils& idynutils)
    {
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

    void checkTask()
    {
        yarp::sig::Matrix T_0 = _idynutils.iDyn3_model.getPosition(
                              _idynutils.iDyn3_model.getLinkIndex(_base_link_0),
                              _idynutils.iDyn3_model.getLinkIndex(_distal_link_0));

        std::cout<<GREEN<<"TASK1: "<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"Arm Initial Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(_T_initial_0);
        std::cout<<GREEN<<"Arm Desired Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(T_ref_0);
        std::cout<<GREEN<<"Arm Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(T_0);
        for(unsigned int i = 0; i < 3; ++i)
            EXPECT_NEAR(T_0(i,3), T_ref_0(i,3), 1E-3);
        for(unsigned int i = 0; i < 3; ++i)
            for(unsigned int j = 0; j < 3; ++j)
                EXPECT_NEAR(T_0(i,j), T_ref_0(i,j), 1E-2);

        yarp::sig::Matrix T_1 = _idynutils.iDyn3_model.getPosition(
                              _idynutils.iDyn3_model.getLinkIndex(_base_link_1),
                              _idynutils.iDyn3_model.getLinkIndex(_distal_link_1));

        std::cout<<std::endl;
        std::cout<<GREEN<<"TASK2: "<<DEFAULT<<std::endl;
        std::cout<<GREEN<<"Arm Initial Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(_T_initial_1);
        std::cout<<GREEN<<"Arm Desired Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(T_ref_1);
        std::cout<<GREEN<<"Arm Pose: "<<DEFAULT<<std::endl; cartesian_utils::printHomogeneousTransform(T_1);
        for(unsigned int i = 0; i < 3; ++i)
            EXPECT_NEAR(T_1(i,3), T_ref_1(i,3), 1E-3);
        for(unsigned int i = 0; i < 3; ++i)
            for(unsigned int j = 0; j < 3; ++j)
                EXPECT_NEAR(T_1(i,j), T_ref_1(i,j), 1E-2);
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

    yarp::sig::Vector dq(test0.q.size(), 0.0);
    double acc = 0.0;
    for(unsigned int t = 0; t < T; ++t)
    {
        test0._idynutils.updateiDyn3Model(test0.q, true);

        test0.update(test0.q);

        bool solved;
        acc += test0.solve(dq, solved);
        if(!solved)
            dq.zero();

        test0.q = test0.q + dq;
    }
    acc = acc/double(T);
    std::cout<<GREEN<<"Average time to solve one step is: "<<acc<<" [s]"<<DEFAULT<<std::endl;

    test0.checkTask();


    testIKProblem2 test1(double(T), opt0, "mpc_2cartesians_stacked");

    std::cout<<GREEN<<"test1 options:"<<DEFAULT<<std::endl;

    dq.zero();
    acc = 0.0;
    for(unsigned int t = 0; t < T; ++t)
    {
        test1._idynutils.updateiDyn3Model(test1.q, true);

        test1.update(test1.q);

        bool solved;
        acc += test1.solve(dq, solved);
        if(!solved)
            dq.zero();

        test1.q = test1.q + dq;
    }
    acc = acc/double(T);
    std::cout<<GREEN<<"Average time to solve one step is: "<<acc<<" [s]"<<DEFAULT<<std::endl;

    test1.checkTask();


    testIKProblem3 test2(double(T), opt0, "mpc_2cartesians_augmented");

    std::cout<<GREEN<<"test2 options:"<<DEFAULT<<std::endl;

    dq.zero();
    acc = 0.0;
    for(unsigned int t = 0; t < T; ++t)
    {
        test2._idynutils.updateiDyn3Model(test2.q, true);

        test2.update(test2.q);

        bool solved;
        acc += test2.solve(dq, solved);
        if(!solved)
            dq.zero();

        test2.q = test2.q + dq;
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

    yarp::sig::Vector dq(test0.q.size(), 0.0);
    double acc = 0.0;
    for(unsigned int t = 0; t < T; ++t)
    {
        test0._idynutils.updateiDyn3Model(test0.q, true);

        test0.update(test0.q);

        bool solved;
        acc += test0.solve(dq, solved);
        if(!solved)
            dq.zero();

        test0.q = test0.q + dq;
    }
    acc = acc/double(T);
    std::cout<<GREEN<<"Average time to solve one step is: "<<acc<<" [s]"<<DEFAULT<<std::endl;

    test0.checkTask();

    testIKProblem2 test1(double(T), opt0, "reliable_2cartesians_stacked");

    std::cout<<GREEN<<"test1 options:"<<DEFAULT<<std::endl;

    dq.zero();
    acc = 0.0;
    for(unsigned int t = 0; t < T; ++t)
    {
        test1._idynutils.updateiDyn3Model(test1.q, true);

        test1.update(test1.q);

        bool solved;
        acc += test1.solve(dq, solved);
        if(!solved)
            dq.zero();

        test1.q = test1.q + dq;
    }
    acc = acc/double(T);
    std::cout<<GREEN<<"Average time to solve one step is: "<<acc<<" [s]"<<DEFAULT<<std::endl;

    test1.checkTask();

    testIKProblem3 test2(double(T), opt0, "reliable_2cartesians_augmented");

    std::cout<<GREEN<<"test2 options:"<<DEFAULT<<std::endl;

    dq.zero();
    acc = 0.0;
    for(unsigned int t = 0; t < T; ++t)
    {
        test2._idynutils.updateiDyn3Model(test2.q, true);

        test2.update(test2.q);

        bool solved;
        acc += test2.solve(dq, solved);
        if(!solved)
            dq.zero();

        test2.q = test2.q + dq;
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

    yarp::sig::Vector dq(test0.q.size(), 0.0);
    double acc = 0.0;
    for(unsigned int t = 0; t < T; ++t)
    {
        test0._idynutils.updateiDyn3Model(test0.q, true);

        test0.update(test0.q);

        bool solved;
        acc += test0.solve(dq, solved);
        if(!solved)
            dq.zero();

        test0.q = test0.q + dq;
    }
    acc = acc/double(T);
    std::cout<<GREEN<<"Average time to solve one step is: "<<acc<<" [s]"<<DEFAULT<<std::endl;

    test0.checkTask();

    testIKProblem2 test1(double(T), opt0, "default_2cartesians_stacked");

    std::cout<<GREEN<<"test1 options:"<<DEFAULT<<std::endl;

    dq.zero();
    acc = 0.0;
    for(unsigned int t = 0; t < T; ++t)
    {
        test1._idynutils.updateiDyn3Model(test1.q, true);

        test1.update(test1.q);

        bool solved;
        acc += test1.solve(dq, solved);
        if(!solved)
            dq.zero();

        test1.q = test1.q + dq;
    }
    acc = acc/double(T);
    std::cout<<GREEN<<"Average time to solve one step is: "<<acc<<" [s]"<<DEFAULT<<std::endl;

    test1.checkTask();

    testIKProblem3 test2(double(T), opt0, "default_2cartesians_augmented");

    std::cout<<GREEN<<"test2 options:"<<DEFAULT<<std::endl;

    dq.zero();
    acc = 0.0;
    for(unsigned int t = 0; t < T; ++t)
    {
        test2._idynutils.updateiDyn3Model(test2.q, true);

        test2.update(test2.q);

        bool solved;
        acc += test2.solve(dq, solved);
        if(!solved)
            dq.zero();

        test2.q = test2.q + dq;
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
