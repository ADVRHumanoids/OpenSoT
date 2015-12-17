#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <idynutils/idynutils.h>
#include <idynutils/RobotUtils.h>
#include <OpenSoT/interfaces/yarp/tasks/YCartesian.h>
#include <OpenSoT/interfaces/yarp/tasks/YCoM.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <OpenSoT/utils/VelocityAllocation.h>
#include <boost/program_options.hpp>
#include <OpenSoT/utils/logger/L.h>
#include <signal.h>

#define MODULE_NAME "example_python"
#define dT          35e-3

typedef boost::accumulators::accumulator_set<double,
                                            boost::accumulators::stats<boost::accumulators::tag::rolling_mean>
                                            > Accumulator;
OpenSoT::L::Ptr logger;
OpenSoT::flushers::Flusher::Ptr tauFlusher;
OpenSoT::flushers::Flusher::Ptr dynamicsFlusher;

void my_handler(int s){
    using namespace OpenSoT::flushers;

    if(tauFlusher && dynamicsFlusher)
    {
        /*                  */
        /*  CREATING PLOTS  */
        /*                  */

        // selecting torso joints for all plots
        OpenSoT::Indices tauIndices        = tauFlusher->getIndices(     DataFlusher<double>::ALL);
        OpenSoT::Indices dynConstr1Indices = dynamicsFlusher->getIndices(constraints::velocity::Dynamics::ESTIMATED_TORQUE);
        OpenSoT::Indices dynConstr2Indices = dynamicsFlusher->getIndices(constraints::velocity::Dynamics::ESTIMATED_TORQUE |
                                                                         constraints::velocity::Dynamics::BLOWERBOUND |
                                                                         constraints::velocity::Dynamics::BUPPERBOUND);
        {
            std::vector<std::list<unsigned int> > finalIndicesL(3);
            std::vector<std::vector<unsigned int> > indicesV(3);
            unsigned int n_dofs = logger->model.getJointNames().size();
            indicesV[0] = tauIndices.asVector();
            indicesV[1] = dynConstr1Indices.asVector();
            indicesV[2] = dynConstr2Indices.asVector();
            for(unsigned int i = 0; i < logger->model.torso.joint_numbers.size(); ++i)
            {
                unsigned int jIndex = logger->model.torso.joint_numbers[i];
                for(unsigned int j = 0; j < 3; ++j)
                    finalIndicesL[j].push_back(indicesV[j][jIndex]);

                finalIndicesL[2].push_back(indicesV[2][jIndex+n_dofs]);
                finalIndicesL[2].push_back(indicesV[2][jIndex+2*n_dofs]);
            }
            tauIndices = OpenSoT::Indices(finalIndicesL[0]);
            dynConstr1Indices = OpenSoT::Indices(finalIndicesL[1]);
            dynConstr2Indices = OpenSoT::Indices(finalIndicesL[2]);

        }

        logger->plotter->figure(10.24,7.68,"estimated torques for torso vs real torques");
        std::list<OpenSoT::plotters::Plottable> plottables;
        plottables.push_back(dynamicsFlusher->i(dynConstr1Indices));
        plottables.push_back(tauFlusher->i(tauIndices));
        logger->plotter->subplot(1,2,1);
        logger->plotter->plot_t(plottables);
        logger->plotter->title("Torque vs Estimated Torque");
        logger->plotter->autoLegend(plottables);
        logger->plotter->xlabel("t [s]");
        logger->plotter->ylabel("tau [Nm]");
        logger->plotter->savefig();

        logger->plotter->subplot(1,2,2);
        logger->plotter->plot_t(dynamicsFlusher->i(dynConstr2Indices));
        logger->plotter->title("Dynamics Constraint");
        logger->plotter->autoLegend(dynamicsFlusher->i(dynConstr2Indices));
        logger->plotter->xlabel("t [s]");
        logger->plotter->ylabel("Error");
        logger->plotter->savefig();
    }

    std::cout<<"Writing log files..."<<std::endl;
    logger.reset();
    std::cout<<"...log files written!"<<std::endl;
    exit(1);
}

int main(int argc, char **argv) {
    signal (SIGINT,my_handler);
    bool no_torque_limits;

    namespace po = boost::program_options;
        po::options_description desc("Options");
        desc.add_options()
          ("help", "this help message")
          ("no_torque_limits",
           po::bool_switch(&no_torque_limits));

    po::variables_map vm;
    try
    {
      po::store(po::parse_command_line(argc, argv, desc),
                vm); // can throw

      /** --help option
       */
      if ( vm.count("help")  )
      {
        std::cout << "example_python, a simple executable creating a stack that executes non-CLIK IK, with pYTask interfaces for tasks" << std::endl
                  << "USAGE: example_python [options] "
                  << std::endl << std::endl;
        std::cout << desc << std::endl;
        return 0;
      }

      po::notify(vm); // throws on error, so do after help in case
                      // there are any problems
    }
    catch(po::error& e)
    {
      std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
      std::cerr << desc << std::endl;
      return 1;
    }

    yarp::os::Network::init();
    Accumulator time_accumulator(boost::accumulators::tag::rolling_mean::window_size = 1000);
    RobotUtils robot( MODULE_NAME, "bigman",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.urdf",
                     std::string(OPENSOT_TESTS_ROBOTS_DIR)+"bigman/bigman.srdf");
    yarp::os::Time::delay(0.5);
    yarp::sig::Vector q = robot.sensePosition();
    yarp::sig::Vector dq = q*0.0;

    RobotUtils::ftPtrMap ft_sensors = robot.getftSensors();
    std::vector<iDynUtils::ft_measure> _ft_measurements;
    for(RobotUtils::ftPtrMap::iterator it = ft_sensors.begin();
        it != ft_sensors.end(); it++)
    {
        iDynUtils::ft_measure ft_measurement;
        ft_measurement.first = it->second->getReferenceFrame();
        _ft_measurements.push_back(ft_measurement);
    }
    RobotUtils::ftReadings ft_readings = robot.senseftSensors();
    for(unsigned int i = 0; i < ft_readings.size(); ++i)
        _ft_measurements[i].second = ft_readings[_ft_measurements[i].first];

    robot.idynutils.setFloatingBaseLink(robot.idynutils.left_leg.end_effector_name);
    robot.idynutils.updateiDyn3Model(q, dq, _ft_measurements, true);
    OpenSoT::DefaultHumanoidStack DHS(robot.idynutils, dT, q);

    /*                            */
    /*      CONFIGURING DHS       */
    /*                            */

    DHS.rightLeg->setLambda(0.6);   DHS.rightLeg->setOrientationErrorGain(1.0);
    DHS.leftLeg->setLambda(0.6);    DHS.leftLeg->setOrientationErrorGain(1.0);
    DHS.rightArm->setLambda(0.1);   DHS.rightArm->setOrientationErrorGain(0.6);
    DHS.leftArm->setLambda(0.1);    DHS.leftArm->setOrientationErrorGain(0.6);
    DHS.com_XY->setLambda(0.05);     DHS.postural->setLambda(0.05);
    DHS.comVelocity->setVelocityLimits(yarp::sig::Vector(0.1,3));
    DHS.selfCollisionAvoidance->setBoundScaling(0.6);
    DHS.velocityLimits->setVelocityLimits(0.6);

    yarp::sig::Matrix pW = DHS.postural->getWeight();
    for(unsigned int i_t = 0; i_t < 3; ++i_t)
        pW(robot.idynutils.torso.joint_numbers[i_t],
            robot.idynutils.torso.joint_numbers[i_t]) *= 1e3;
    for(unsigned int i_t = 0; i_t < 6; ++i_t)
    {
        double amt = 7.5e1;
        if(i_t == 3 || i_t == 4)
            amt = 3;
        pW(robot.idynutils.left_leg.joint_numbers[i_t],
           robot.idynutils.left_leg.joint_numbers[i_t]) *= amt;
        pW(robot.idynutils.right_leg.joint_numbers[i_t],
           robot.idynutils.right_leg.joint_numbers[i_t]) *= amt;
    }
    DHS.postural->setWeight(pW);

    yarp::sig::Vector tauLims = DHS.torqueLimits->getTorqueLimits();
    tauLims[robot.idynutils.torso.joint_numbers[0]] *= 0.6;
    tauLims[robot.idynutils.torso.joint_numbers[1]] *= 0.6;
    tauLims[robot.idynutils.torso.joint_numbers[2]] *= 0.6;

    DHS.torqueLimits->setTorqueLimits(tauLims);
    DHS.torqueLimits->setBoundScaling(0.2);

    std::list<std::pair<std::string,std::string> > whiteList;
    // lower body - arms collision whitelist for WalkMan (for upper-body manipulation tasks - i.e. not crouching)
    whiteList.push_back(std::pair<std::string,std::string>("LLowLeg","LSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("LHipMot","LSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("RLowLeg","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("RHipMot","RSoftHandLink"));

    // torso - arms collision whitelist for WalkMan
    whiteList.push_back(std::pair<std::string,std::string>("DWS","LSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("DWS","LWrMot2"));
    whiteList.push_back(std::pair<std::string,std::string>("DWS","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("DWS","RWrMot2"));
    whiteList.push_back(std::pair<std::string,std::string>("TorsoProtections","LElb"));
    whiteList.push_back(std::pair<std::string,std::string>("TorsoProtections","LSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("TorsoProtections","RElb"));
    whiteList.push_back(std::pair<std::string,std::string>("TorsoProtections","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("Waist","LSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("Waist","LWrMot2"));
    whiteList.push_back(std::pair<std::string,std::string>("Waist","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("Waist","RWrMot2"));

    // arm - am collision whitelist for WalkMan
    whiteList.push_back(std::pair<std::string,std::string>("LShr","RShr"));
    whiteList.push_back(std::pair<std::string,std::string>("LShr","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("LShr","RWrMot2"));
    whiteList.push_back(std::pair<std::string,std::string>("LSoftHandLink","RShr"));
    whiteList.push_back(std::pair<std::string,std::string>("LSoftHandLink","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("LSoftHandLink","RWrMot2"));
    whiteList.push_back(std::pair<std::string,std::string>("LWrMot2","RShr"));
    whiteList.push_back(std::pair<std::string,std::string>("LWrMot2","RSoftHandLink"));
    whiteList.push_back(std::pair<std::string,std::string>("LWrMot2","RWrMot2"));

    DHS.selfCollisionAvoidance->setCollisionWhiteList(whiteList);


    /*                            */
    /*       CREATING STACK       */
    /*                            */

    // defining a stack composed of size two,
    // where the task of first priority is an aggregated of leftLeg and rightLeg,
    // task of priority two is leftArm and rightArm,
    // and the stack is subject to bounds jointLimits and velocityLimits
    OpenSoT::AutoStack::Ptr autoStack = 
        ( DHS.rightLeg ) /
        ( (DHS.com_XY) << DHS.selfCollisionAvoidance << DHS.convexHull ) /
        ( (DHS.leftArm + DHS.rightArm ) << DHS.selfCollisionAvoidance ) /
        ( (DHS.postural) << DHS.selfCollisionAvoidance );
    autoStack << DHS.jointLimits;
    /*if(no_torque_limits)
    {
        std::cout << "Adding torque limits.." << std::endl;
        autoStack << DHS.torqueLimits; // << DHS.velocityLimits; commented since we are using VelocityALlocation
    }*/

    OpenSoT::VelocityAllocation(autoStack,
                                dT,
                                0.6,
                                0.9);

    // setting higher velocity limit to last stack --
    // TODO next feature of VelocityAllocation is a last_stack_speed ;)
    typedef std::list<OpenSoT::Constraint<yarp::sig::Matrix,yarp::sig::Vector>::ConstraintPtr>::iterator it_constraint;
    OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr lastTask = autoStack->getStack()[3];
    for(it_constraint i_c = lastTask->getConstraints().begin() ;
        i_c != lastTask->getConstraints().end() ; ++i_c) {
        if( boost::dynamic_pointer_cast<
                OpenSoT::constraints::velocity::VelocityLimits>(
                    *i_c))
            boost::dynamic_pointer_cast<
                            OpenSoT::constraints::velocity::VelocityLimits>(
                                *i_c)->setVelocityLimits(1.1);
    }


    /*                            */
    /*  CREATING TASK INTERFACES  */
    /*                            */

    OpenSoT::interfaces::yarp::tasks::YCartesian leftArm(robot.idynutils.getRobotName(),
                                                         MODULE_NAME, DHS.leftArm);
    OpenSoT::interfaces::yarp::tasks::YCartesian rightArm(robot.idynutils.getRobotName(),
                                                         MODULE_NAME, DHS.rightArm);
    /*
    OpenSoT::interfaces::yarp::tasks::YCartesian leftLeg(robot.idynutils.getRobotName(),
                                                         MODULE_NAME, DHS.leftLeg);
    OpenSoT::interfaces::yarp::tasks::YCartesian rightLeg(robot.idynutils.getRobotName(),
                                                         MODULE_NAME, DHS.rightLeg);
    */

    OpenSoT::interfaces::yarp::tasks::YCoM com(robot.idynutils.getRobotName(),
                                               MODULE_NAME, DHS.com);

    OpenSoT::solvers::QPOases_sot::Ptr solver;
    if(!no_torque_limits)
    {
        std::cout << "Adding torque limits.." << std::endl;
        solver.reset(new OpenSoT::solvers::QPOases_sot(autoStack->getStack(),
                                             autoStack->getBounds(),
                                             DHS.torqueLimits, 1e10));
    } else
        solver.reset(new OpenSoT::solvers::QPOases_sot(autoStack->getStack(),
                                             autoStack->getBounds(), 1e10));

    robot.setPositionDirectMode();
    yarp::os::Time::delay(2.5);

    double tic, toc, begin;
    int print_mean = 0;


    yarp::sig::Vector q_m(q.size(), 0.0), dq_m(q.size(), 0.0), tau_m(q.size(), 0.0);
    robot.sense(q_m, dq_m, tau_m);

    /*                    */
    /*  CREATING LOGGER   */
    /*                    */

    logger.reset(new OpenSoT::L(MODULE_NAME,robot.idynutils));
    logger->open("dynamics_log");
    dynamicsFlusher = logger->add(DHS.torqueLimits);
    tauFlusher = logger->add(tau_m.data(), q.size());

    begin = yarp::os::Time::now();

    while(true) {
        tic = yarp::os::Time::now() - begin;


        robot.sense(q_m, dq_m, tau_m);

        RobotUtils::ftReadings ft_readings = robot.senseftSensors();
        for(unsigned int i = 0; i < _ft_measurements.size(); ++i)
            _ft_measurements[i].second += (ft_readings[_ft_measurements[i].first]-_ft_measurements[i].second)*0.6;

        robot.idynutils.updateiDyn3Model(q, dq/dT, _ft_measurements, true);

        autoStack->update(q);
        if(!no_torque_limits)
        {
            using namespace yarp::math;
            DHS.torqueLimits->update(cat(q,dq/dT));
        }
        if(solver->solve(dq))
            q+=dq;
        else
            std::cout << "Error computing solve()" << std::endl;

        logger->udpate(tic, dq);

        robot.move(q);
        toc = yarp::os::Time::now() - begin;
        time_accumulator(toc-tic);

        // print mean every 5s
        if(((++print_mean)%int(5.0/dT))==0) {
            print_mean = 0;
            std::cout << "dt = "
                      << boost::accumulators::extract::rolling_mean(time_accumulator) << std::endl;

            std::cout << "l_wrist reference:" << DHS.leftArm->getReference().toString() << std::endl;
            std::cout << "r_wrist reference:" << DHS.rightArm->getReference().toString() << std::endl;
            std::cout << "Active Capsules Pairs: " << DHS.selfCollisionAvoidance->getbUpperBound().size() << std::endl;
            std::cout << "Configuration: " << q.toString() << std::endl;
        }
        yarp::os::Time::delay(dT-(toc-tic));
    }
}
