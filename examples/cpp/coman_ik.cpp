#include "../../tests/common.h"
#include <string>
#include <random>
#include <xbot2_interface/xbotinterface2.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/nHQP.h>
#include <OpenSoT/solvers/iHQP.h>
#include <qpSWIFT/qpSWIFT.h>
#include <matlogger2/matlogger2.h>

#include <chrono>
using namespace std::chrono;

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>



bool IS_ROSCORE_RUNNING;

#define NUMBER_OF_RUNS 30

/**
 * @brief removeMinMax remove min and max element from vector
 * @param vec
 */
void removeMinMax(std::vector<double>& vec)
{
    auto it = std::max_element(vec.begin(), vec.end());
    vec.erase(it);
    it = std::min_element(vec.begin(), vec.end());
    vec.erase(it);
}

/**
 * @brief The solver_statistics struct
 */
struct solver_statistics{
    solver_statistics(const std::string& back_end_id_, const std::vector<double>& solver_time_ms_, const unsigned int number_of_iterations_):
        back_end_id(back_end_id_), solver_time_ms(solver_time_ms_), number_of_iterations(number_of_iterations_){}

    std::vector<double> solver_time_ms; // each time a solve is called in a ik call
    unsigned int number_of_iterations;  // to the solution in a ik call
    std::string back_end_id;

    double solverTimeMean()
    {
        return accumulate(solver_time_ms.begin(), solver_time_ms.end(),double(0.0))/solver_time_ms.size();
    }
};

void publishJointStates(const Eigen::VectorXd& q, const Eigen::Affine3d& start, const Eigen::Affine3d& goal,
                        const XBot::ModelInterface::Ptr model, ros::NodeHandle& n)
{
    sensor_msgs::JointState msg;
    std::vector<std::string> joint_names = model->getJointNames();
    unsigned int i = 0;
    if(model->isFloatingBase())
        i = 1;
    for(i; i < joint_names.size(); ++i)
    {
        msg.name.push_back(joint_names[i]);
        msg.position.push_back(q[i]);
    }
    msg.header.stamp = ros::Time::now();
    static auto joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
    joint_state_pub.publish(msg);

    static tf::TransformBroadcaster br;
    tf::Transform transform_start, transform_goal;
    tf::Pose pose_start, pose_goal;
    tf::poseEigenToTF(start, pose_start);
    transform_start.setOrigin(pose_start.getOrigin());
    transform_start.setRotation(pose_start.getRotation());
    tf::poseEigenToTF(goal, pose_goal);
    transform_goal.setOrigin(pose_goal.getOrigin());
    transform_goal.setRotation(pose_goal.getRotation());

    Eigen::Affine3d floating_base;
    model->getFloatingBasePose(floating_base);
    tf::Transform transform_floating_base;
    tf::Pose pose_floating_base;
    tf::poseEigenToTF(floating_base, pose_floating_base);
    transform_floating_base.setOrigin(pose_floating_base.getOrigin());
    transform_floating_base.setRotation(pose_floating_base.getRotation());

    br.sendTransform(tf::StampedTransform(transform_floating_base, msg.header.stamp, "world", "base_link"));
    br.sendTransform(tf::StampedTransform(transform_start, msg.header.stamp, "world", "start"));
    br.sendTransform(tf::StampedTransform(transform_goal, msg.header.stamp, "world", "goal"));
}

/**
 * @brief areAlmostEqual check if two double numbers are the same up t certain precision
 * @param a first number
 * @param b second number
 * @param precision
 * @return true if a == b
 */
bool areAlmostEqual(double a, double b, int precision = 5) {
    double epsilon = std::pow(10, -precision);
    return std::fabs(a - b) < epsilon;
}

/**
 * @brief allElementsSame check if all elements are equal in a vector of double
 * @param vec of double
 * @return true if all elements are equal
 */
bool allElementsSame(const std::vector<double>& vec) {
    if (vec.empty())
        return true;

    double firstElement = vec[0];
    for (const auto& num : vec)
    {
        if (!areAlmostEqual(firstElement, num))
            return false;
    }
    return true;
}

/**
 * @brief solveIK resolve inverse kineamtics from a start and goal pose using Euler integration:
 *      \mathbf{q}_{k+1} = \mathbf{q}_{k} + \boldsymbol{\delta}\mathbf{q}
 * \boldsymbol{\delta}\mathbf{q} is computed from QP
 * @param q_start configuration to compute the start pose
 * @param q_goal configuration to compute the goal pose
 * @param TCP_frame controlled frame in world
 * @param model
 * @param stack
 * @param solver
 * @param max_iterations number of maximum iterations to the goal pose
 * @param norm_error_eps under this value the goal pose is considered reached
 * @param dT control loop time, used by ROS to publish joint states
 * @param n ROS node handle
 * @param back_end_name name of the used backend
 * @return solver statistics
 */
solver_statistics solveIK(const Eigen::VectorXd& q_start, const Eigen::Affine3d& TCP_world_pose_goal, const std::string& TCP_frame,
                          XBot::ModelInterface::Ptr model, OpenSoT::AutoStack::Ptr stack,
                          OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr solver,
                          const unsigned int max_iterations, const double norm_error_eps,
                          const double dT, std::shared_ptr<ros::NodeHandle> n, const std::string& back_end_name)
{
    /**
      * Update model with q_start and q_goal and retrieve initial and goal Cartesian pose
      **/
    model->setJointPosition(q_start);
    model->update();

    /**
     * @brief Retrieve the Cartesian task from the stack and set reference.
     */
    auto task = stack->getTask("r_wrist");
    OpenSoT::tasks::velocity::Cartesian::asCartesian(task)->setReference(TCP_world_pose_goal);

    /**
     * @brief ik loop
     */
    Eigen::VectorXd dq, q = q_start;
    dq.setZero(q.size());
    Eigen::Affine3d TCP_world_pose_init, TCP_world_pose;
    OpenSoT::tasks::velocity::Cartesian::asCartesian(task)->getActualPose(TCP_world_pose_init);
    double position_error_norm = (TCP_world_pose_init.matrix().block(0,3,3,1)-TCP_world_pose_goal.matrix().block(0,3,3,1)).norm();
    unsigned int iter = 0;
    std::vector<double> solver_time; //ms
    solver_time.reserve(max_iterations);
    std::vector<double> errors;
    while(position_error_norm > norm_error_eps && iter < max_iterations)
    {
        std::cout<<"position error norm: "<<position_error_norm<<" at iteration "<<iter<<std::endl;

        //1. update model with joint position
        model->setJointPosition(q);
        model->update();

        //2. update the stack
        stack->update(q);

        //3. solve the QP
        auto start = high_resolution_clock::now();
        bool success = solver->solve(dq);
        auto stop = high_resolution_clock::now();
        if(!success)
            dq.setZero();
        else
            solver_time.push_back(duration_cast<microseconds>(stop - start).count() * 1e-3);

        //4. update the state
        q += dq;

        if(IS_ROSCORE_RUNNING)
        {
            ros::Rate loop_rate(int(1./dT));
            loop_rate.sleep();
            publishJointStates(q, TCP_world_pose_init, TCP_world_pose_goal, model, *n.get());
            ros::spinOnce();
        }

        model->getPose(TCP_frame, TCP_world_pose);
        position_error_norm = (TCP_world_pose.matrix().block(0,3,3,1)-TCP_world_pose_goal.matrix().block(0,3,3,1)).norm();
        iter++;

        errors.push_back(position_error_norm);
        /**
          * We check if last 10 errors are equal, if true we abort
          */
        if(errors.size() == 10)
        {
            if(allElementsSame(errors))
                break;
            errors.clear();
        }
    }
    std::cout<<"position error norm: "<<position_error_norm<<" at iteration "<<iter<<std::endl;

    std::cout<<"TCP final pose in world: \n"<<TCP_world_pose.matrix()<<std::endl;
    std::cout<<"TCP goal position in world: \n"<<TCP_world_pose_goal.matrix().block(0,3,3,1).transpose()<<std::endl;

    if(IS_ROSCORE_RUNNING) usleep(500000);

    return solver_statistics(back_end_name, solver_time, iter);
}

using namespace OpenSoT::solvers;
std::string getBackEndString(const solver_back_ends solver_back_end)
{
    switch(solver_back_end)
    {
        case(solver_back_ends::qpOASES):    return "qpOASES";
        case(solver_back_ends::OSQP):       return "OSQP";
        case(solver_back_ends::eiQuadProg): return "eiQuadProg";
        case(solver_back_ends::qpSWIFT):    return "qpSWIFT";
        case(solver_back_ends::proxQP):     return "proxQP";
    }
    return "";
}

solver_back_ends getBackEndFromString(const std::string& str)
{
    if(str == "qpOASES")            return solver_back_ends::qpOASES;
    else if(str == "OSQP")          return solver_back_ends::OSQP;
    else if(str == "eiQuadProg")    return solver_back_ends::eiQuadProg;
    else if(str == "qpSWIFT")       return solver_back_ends::qpSWIFT;
    else if(str == "proxQP")        return solver_back_ends::proxQP;
}

void log(XBot::MatLogger2::Ptr logger, solver_statistics& stats)
{
    logger->add(stats.back_end_id + "solver_time_ms_mean", stats.solverTimeMean());
    logger->add(stats.back_end_id + "iterations", stats.number_of_iterations);
}

void setGoodInitialPosition(Eigen::VectorXd& q, const XBot::ModelInterface::Ptr model_ptr) {
    q[model_ptr->getDofIndex("RHipSag")] = -25.0*M_PI/180.0;
    q[model_ptr->getDofIndex("RKneeSag")] = 50.0*M_PI/180.0;
    q[model_ptr->getDofIndex("RAnkSag")] = -25.0*M_PI/180.0;

    q[model_ptr->getDofIndex("LHipSag")] = -25.0*M_PI/180.0;
    q[model_ptr->getDofIndex("LKneeSag")] = 50.0*M_PI/180.0;
    q[model_ptr->getDofIndex("LAnkSag")] = -25.0*M_PI/180.0;

    q[model_ptr->getDofIndex("LShSag")] =  20.0*M_PI/180.0;
    q[model_ptr->getDofIndex("LShLat")] = 20.0*M_PI/180.0;
    q[model_ptr->getDofIndex("LShYaw")] = -15.0*M_PI/180.0;
    q[model_ptr->getDofIndex("LElbj")] = -80.0*M_PI/180.0;

    q[model_ptr->getDofIndex("RShSag")] =  20.0*M_PI/180.0;
    q[model_ptr->getDofIndex("RShLat")] = -20.0*M_PI/180.0;
    q[model_ptr->getDofIndex("RShYaw")] = 15.0*M_PI/180.0;
    q[model_ptr->getDofIndex("RElbj")] = -80.0*M_PI/180.0;

}

/**
 * @brief positionRand return random position vector between min and max
 * @param p
 * @param min
 * @param max
 * @param engine
 */
void positionRand(Eigen::VectorXd& p, double min, double max, std::mt19937& engine)
{
    for(unsigned int i = 0; i < p.size(); ++i)
    {
        std::uniform_real_distribution<double> dist(min, max);
        p[i] = dist(engine);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coman_ik_node");
    IS_ROSCORE_RUNNING = ros::master::check();
    std::shared_ptr<ros::NodeHandle> n;
    if(IS_ROSCORE_RUNNING)
        n = std::make_shared<ros::NodeHandle>();

    /**
      * @brief Retrieve model from config file and generate random initial configuration from qmin and qmax
      **/
    XBot::ModelInterface::Ptr model_ptr = GetTestModel("coman");
    if(model_ptr->isFloatingBase())
        std::cout<<"floating base is true!"<<std::endl;
    std::cout<<"#DOFs: "<<model_ptr->getJointNum()<<std::endl;

    Eigen::VectorXd qmin, qmax;
    model_ptr->getJointLimits(qmin, qmax);

    Eigen::VectorXd dqlim;
    model_ptr->getVelocityLimits(dqlim);

    std::string TCP_frame = "r_wrist";
    unsigned int max_iter = 1000;
    double min_error = 1e-3;
    std::vector<solver_statistics> st;
    std::map<solver_back_ends, unsigned int> back_end_success; //store back-ends to test and total number of ik call success

    std::vector<std::string> stack_priorities = {"1_LEVEL", "2_LEVELS", "3_LEVELS", "4_LEVELS"};

    for(auto stack_priority : stack_priorities)
    {
        std::vector<std::string> front_ends = {"iHQP", "nHQP"};
        for(auto front_end : front_ends)
        {
            std::cout<<"USING FRONT-END: "<<front_end<<std::endl;

            back_end_success[solver_back_ends::qpOASES] = 0;
            back_end_success[solver_back_ends::OSQP] = 0;
            back_end_success[solver_back_ends::eiQuadProg] = 0;
            back_end_success[solver_back_ends::qpSWIFT] = 0;
            back_end_success[solver_back_ends::proxQP] = 0;


            XBot::MatLogger2::Ptr logger = XBot::MatLogger2::MakeLogger("/tmp/coman_ik_stats_" + stack_priority + "_" + front_end);
            logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

            /**
              * Outer loop: the ik is tested on NUMBER_OF_RUNS different start and goal configurations
              **/
            unsigned int total_runs = 0;
            for(unsigned int k = 0; k < NUMBER_OF_RUNS; ++k)
            {
                /**
                 * @brief We pass to the next configurations only if all the solvers reach the min_error in the Cartesian position task
                 * with less iterations than max_iter. This is checked at the end of this while loop.
                 */
                bool all_good = false;
                while(!all_good)
                {
                    st.clear();

                    /**
                     * @brief Given initial pose, we select random delta motion for the r_wrist and we assign as Cartesian goal
                     */
                    Eigen::VectorXd q(model_ptr->getJointNum()); q.setZero();
                    setGoodInitialPosition(q, model_ptr);

                    model_ptr->setJointPosition(q);
                    model_ptr->update();
                    Eigen::Affine3d floating_base_pose;
                    model_ptr->getPose("Waist", "r_sole", floating_base_pose);
                    std::cout<<"floating base pose: \n"<<floating_base_pose.matrix()<<std::endl;
                    model_ptr->setFloatingBasePose(floating_base_pose);
                    model_ptr->update();
                    model_ptr->getJointPosition(q);
                    std::cout<<"q: "<<q.transpose()<<std::endl;

                    Eigen::Affine3d w_T_TCP;
                    model_ptr->getPose(TCP_frame, w_T_TCP);
                    std::cout<<"Initial r_wrist pose: \n"<<w_T_TCP.matrix()<<std::endl;

                    std::random_device seeder;
                    std::mt19937 engine(seeder());
                    Eigen::VectorXd Dp; Dp.setZero(3);
                    positionRand(Dp, -0.2, 0.2, engine);
                    Eigen::Affine3d w_T_TCP_goal = w_T_TCP;
                    w_T_TCP_goal.translation() += Dp;

                    /**
                      * We loop on the solvers
                      **/
                    for(solver_back_ends solver_back_end : solver_back_ends_iterator())
                    {
                        if(back_end_success.find(solver_back_end) != back_end_success.end())
                        {
                            std::cout<<"USING BACK-END: "<<getBackEndString(solver_back_end)<<std::endl;

                            /**
                            * Creates one Cartesian task and one Postural task
                            */
                            model_ptr->setJointPosition(q);
                            model_ptr->update();

                           using namespace OpenSoT::tasks::velocity;
                           auto r_sole = std::make_shared<Cartesian>("r_sole", q, *model_ptr.get(), "r_sole", "world");
                           auto l_sole = std::make_shared<Cartesian>("l_sole", q, *model_ptr.get(), "l_sole", "world");
                           auto l_wrist = std::make_shared<Cartesian>("l_wrist", q, *model_ptr.get(), "l_wrist", "world");
                           l_wrist->setLambda(0.1);
                           auto r_wrist = std::make_shared<Cartesian>("r_wrist", q, *model_ptr.get(), TCP_frame, "world");
                           r_wrist->setLambda(0.1);
                           auto com = std::make_shared<CoM>(q, *model_ptr.get());
                           com->setLambda(0.1);


                           auto postural = std::make_shared<Postural>(*model_ptr, "postural");
                           postural->setLambda(0.01);

                           /**
                            * Creates constraints joint position and velocity limits
                            */
                           using namespace OpenSoT::constraints::velocity;
                           auto joint_limits = std::make_shared<JointLimits>(*model_ptr, qmax, qmin);

                           double dT = 0.01;
                           auto vel_limits = std::make_shared<VelocityLimits>(dqlim, dT);

                           OpenSoT::AutoStack::Ptr stack;
                           std::cout<<"Creating stack "<<stack_priority<<std::endl;
                           if(stack_priority == stack_priorities[0]){
                           /**
                             * We create a stack with ONE priority level:
                             **/
                            stack /= (0.1*l_wrist + r_wrist + com + 1e-4*postural);
                            stack<<joint_limits<<vel_limits<<(l_sole + r_sole);
                           }
                           else if (stack_priority == stack_priorities[1]){
                            /**
                             * We create a stack with TWO priority levels:
                             **/
                            stack = ((com + 0.1*l_wrist + r_wrist)/postural)<<joint_limits<<vel_limits<<(l_sole + r_sole);
                           }
                           else if (stack_priority == stack_priorities[2]){
                            /**
                             * We create a stack with THREE priority levels:
                             **/
                            stack = (com/(0.1*l_wrist + r_wrist)/postural)<<joint_limits<<vel_limits<<(l_sole + r_sole);
                           }
                           else{
                               /**
                                * We create a stack with FOUR priority levels:
                                **/
                               stack = (com/l_wrist/r_wrist/postural)<<joint_limits<<vel_limits<<(l_sole + r_sole);
                           }

                           stack->update(q);

                           double eps = 1e6;
                           //if(front_end == front_ends[1])
                           //    eps = 1e-6;
                           OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr solver;
                           bool solver_inited = false;
                           bool reset_eps = false;
                           while(!solver_inited)
                           {
                               if(front_end == front_ends[0])
                               {
                                   OpenSoT::solvers::iHQP::Ptr iHQPSolver;
                                   try
                                   {
                                        iHQPSolver = std::make_shared<OpenSoT::solvers::iHQP>(*stack, eps, solver_back_end);
                                   }
                                   catch(...)
                                   {
                                      eps *= 10;
                                      std::cout<<"Problem initializing solver, increasing eps..."<<std::endl;
                                      reset_eps = true;
                                   }

                                   if(reset_eps)
                                    iHQPSolver->setEpsRegularisation(1e-5);

                                   solver = iHQPSolver;
                                   solver_inited = true;
                               }
                               else
                               {
                                   OpenSoT::solvers::nHQP::Ptr nHQPSolver = std::make_shared<OpenSoT::solvers::nHQP>(stack->getStack(), stack->getBounds(), eps, solver_back_end);
                                   if(stack_priority == stack_priorities[0])
                                       nHQPSolver->setPerformAbRegularization(false);
                                   nHQPSolver->setPerformSelectiveNullSpaceRegularization(false);
                                   solver = nHQPSolver;
                                   solver_inited = true;
                               }
                           }

                           /**
                            * @brief Here we do the ik loop
                            */
                           solver_statistics stats =  solveIK(q, w_T_TCP_goal, TCP_frame, model_ptr, stack, solver, max_iter, min_error, dT, n, getBackEndString(solver_back_end));

                           /**
                             * We possibly remove the largest and smaller values
                             **/
                           if(stats.solver_time_ms.size() > 2)
                               removeMinMax(stats.solver_time_ms);

                           st.push_back(stats);
                        }

                    }
                    total_runs++;

                    /**
                      * @brief check if everything went fine using all the solvers
                      **/
                    all_good = true;
                    for(solver_statistics solver_stat : st)
                    {
                        if(solver_stat.number_of_iterations < max_iter && solver_stat.solver_time_ms.size() > 0){
                            std::cout<<"mean solver_time: "<<solver_stat.solverTimeMean()<<" [ms] using back-end: "<<solver_stat.back_end_id<<std::endl;
                            back_end_success[getBackEndFromString(solver_stat.back_end_id)]++;
                        }
                        else
                            all_good = false;
                    }
                    // if everything went good with all the solvers then the data are logged
                    if(all_good)
                    {
                        for(solver_statistics solver_stat : st)
                            log(logger, solver_stat);
                    }
                }
            }
            for(solver_statistics solver_stat : st){
                logger->add(solver_stat.back_end_id + "number_of_success", back_end_success[getBackEndFromString(solver_stat.back_end_id)]);
                std::cout<<solver_stat.back_end_id<<" number of success: "<<back_end_success[getBackEndFromString(solver_stat.back_end_id)]<<std::endl;
            }
            logger->add("total_runs", total_runs);
            std::cout<<"total runs: "<<total_runs<<std::endl;
        }
    }
    return 0;
}
