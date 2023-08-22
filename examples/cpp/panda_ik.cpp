#include <string>
#include <random>
#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/solvers/iHQP.h>

#include <chrono>
using namespace std::chrono;


std::string _path_to_cfg = OPENSOT_EXAMPLE_PATH "configs/panda/configs/config_panda.yaml";

void removeMinMax(std::vector<double>& vec)
{
    auto it = std::max_element(vec.begin(), vec.end());
    vec.erase(it);
    it = std::min_element(vec.begin(), vec.end());
    vec.erase(it);
}

void vectorRand(Eigen::VectorXd& vec, const Eigen::VectorXd& min, const Eigen::VectorXd& max, std::mt19937& engine)
{
    for(unsigned int i = 0; i < vec.size(); ++i)
    {
        std::uniform_real_distribution<double> dist(min[i], max[i]);
        vec[i] = dist(engine);
    }
}

struct solver_statistics{
    solver_statistics(const std::vector<double>& solver_time_ms_, const unsigned int number_of_iterations_):
        solver_time_ms(solver_time_ms_), number_of_iterations(number_of_iterations_){}
    std::vector<double> solver_time_ms;
    unsigned int number_of_iterations;
};

solver_statistics solveIK(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_goal, const std::string& TCP_frame,
                          XBot::ModelInterface::Ptr model, OpenSoT::AutoStack::Ptr stack,
                          OpenSoT::solvers::iHQP::Ptr solver,
                          const unsigned int max_iterations, const double norm_error_eps)
{
    /**
      * Update model with q_start and q_goal and retrieve initial and goal Cartesian pose
      **/
    model->setJointPosition(q_start);
    model->update();

    Eigen::Affine3d TCP_world_pose_init;
    model->getPose(TCP_frame, TCP_world_pose_init);
    std::cout<<"TCP_init pose in world: \n"<<TCP_world_pose_init.matrix()<<std::endl;

    model->setJointPosition(q_goal);
    model->update();
    Eigen::Affine3d TCP_world_pose_goal;
    model->getPose(TCP_frame, TCP_world_pose_goal);
    std::cout<<"TCP_goal pose in world: \n"<<TCP_world_pose_goal.matrix()<<std::endl;

    auto subtask = stack->getTask("TCP_0to2");
    auto task = OpenSoT::SubTask::asSubTask(subtask)->getTask();
    OpenSoT::tasks::velocity::Cartesian::asCartesian(task)->setReference(TCP_world_pose_goal);

    Eigen::VectorXd dq, q = q_start;
    Eigen::Affine3d TCP_world_pose = TCP_world_pose_init;
    double position_error_norm = (TCP_world_pose.matrix().block(0,3,3,1)-TCP_world_pose_goal.matrix().block(0,3,3,1)).norm();
    unsigned int iter = 0;
    std::vector<double> solver_time; //ms
    solver_time.reserve(max_iterations);
    while(position_error_norm > norm_error_eps && iter < max_iterations)
    {
        std::cout<<"position error norm: "<<position_error_norm<<" at iteration "<<iter<<std::endl;

        model->setJointPosition(q);
        model->update();

        stack->update(q);

        auto start = high_resolution_clock::now();
        bool success = solver->solve(dq);
        auto stop = high_resolution_clock::now();
        if(!success)
            dq.setZero();
        else
            solver_time.push_back(duration_cast<microseconds>(stop - start).count() * 1e-3);

        q += dq;

        model->getPose(TCP_frame, TCP_world_pose);
        position_error_norm = (TCP_world_pose.matrix().block(0,3,3,1)-TCP_world_pose_goal.matrix().block(0,3,3,1)).norm();
        iter++;
    }
    std::cout<<"position error norm: "<<position_error_norm<<" at iteration "<<iter<<std::endl;

    std::cout<<"TCP final pose in world: \n"<<TCP_world_pose.matrix()<<std::endl;
    std::cout<<"TCP goal position in world: \n"<<TCP_world_pose.matrix().block(0,3,3,1).transpose()<<std::endl;

    return solver_statistics(solver_time, iter);

}

int main()
{
    /**
      * @brief Retrieve model from config file and generate random initial configuration from qmin and qmax
      **/
    XBot::ModelInterface::Ptr model_ptr = XBot::ModelInterface::getModel(_path_to_cfg);

    Eigen::VectorXd qmin, qmax;
    model_ptr->getJointLimits(qmin, qmax);

    Eigen::VectorXd dqlim;
    model_ptr->getVelocityLimits(dqlim);

    Eigen::VectorXd q_init(model_ptr->getJointNum());
    std::random_device seeder;
    std::mt19937 engine(seeder());
    vectorRand(q_init, qmin, qmax, engine);
    std::cout<<"q_init: "<<q_init.transpose()<<std::endl;

    Eigen::VectorXd q_goal(model_ptr->getJointNum());
    vectorRand(q_goal, qmin, qmax, engine);
    std::cout<<"q_goal: "<<q_goal.transpose()<<std::endl;

    std::string TCP_frame = "panda_link8";


    using namespace OpenSoT::solvers;
    for(solver_back_ends solver_back_end : solver_back_ends_iterator())
    {
        if(solver_back_end == solver_back_ends::qpOASES || solver_back_end == solver_back_ends::OSQP ||
                solver_back_end == solver_back_ends::eiQuadProg)
        {
            std::string solver_back_end_string = "";
            switch(solver_back_end)
            {
                case(solver_back_ends::qpOASES): solver_back_end_string = "qpOASES"; break;
                case(solver_back_ends::OSQP): solver_back_end_string = "OSQP"; break;
            }

            std::cout<<"USING BACK-END: "<<solver_back_end_string<<std::endl;


            /**
            * Creates one Cartesian task and one Postural task
            */
            model_ptr->setJointPosition(q_init);
            model_ptr->update();

           using namespace OpenSoT::tasks::velocity;
           auto TCP = std::make_shared<Cartesian>("TCP", q_init, *model_ptr.get(), TCP_frame, "world");

           Eigen::VectorXd zeros = q_init;
           zeros.setZero();
           auto postural = std::make_shared<Postural>(q_init, "postural");
           postural->setReference(zeros);

           /**
            * Creates constraints joint position and velocity limits
            */
           using namespace OpenSoT::constraints::velocity;
           auto joint_limits = std::make_shared<JointLimits>(q_init, qmax, qmin);

           double dT = 0.01;
           auto vel_limits = std::make_shared<VelocityLimits>(dqlim, dT);

           /**
             * We create a stack with two pririty levels:
             *  1. Cartesian POSITION only
             *  2. Postural in joint space
             * and tow constraints: joint position and velocity limits
             **/
           std::list<unsigned int> position_ids = {0, 1, 2};
           OpenSoT::AutoStack::Ptr stack = ((TCP%position_ids)/postural)<<joint_limits<<vel_limits;
           stack->update(q_init);

           double eps = 1e6;
           OpenSoT::solvers::iHQP::Ptr solver;
           bool solver_inited = false;
           bool reset_eps = false;
           while(!solver_inited)
           {
               try{
                   solver = std::make_shared<OpenSoT::solvers::iHQP>(*stack, eps, solver_back_end);
                   solver_inited = true;
               }catch(...){
                   eps *= 10;
                   std::cout<<"Problem initializing solver, increasing eps..."<<std::endl;
                   reset_eps = true;
               }
           }
           if(reset_eps)
               solver->setEpsRegularisation(1e-7);

           unsigned int max_iter = 1000;
           double min_error = 1e-4;
           solver_statistics stats =  solveIK(q_init, q_goal, TCP_frame, model_ptr, stack, solver, max_iter, min_error);

           removeMinMax(stats.solver_time_ms);
           std::cout<<"mean solver_time: "<<accumulate(stats.solver_time_ms.begin(), stats.solver_time_ms.end(),double(0.0))/stats.solver_time_ms.size()<<" ms"<<std::endl;
        }
    }


    return 0;
}
