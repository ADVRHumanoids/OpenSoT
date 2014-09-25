/*
 * Copyright: (C) 2014 Walkman Consortium
 * Authors: Enrico Mingo, Alessio Rocchi
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include "sot_VelKinCon_ctrl.h"
#include <boost/foreach.hpp>
#include "task_solver.h"
#include "sot_VelKinCon_constants.h"
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>
#include <drc_shared/cartesian_utils.h>

#define toRad(X) (X*M_PI/180.0)
#define toDeg(X) (X*180.0/M_PI)
#define MilliSecToSec(X) (X/1000.0)

/** ******************************************* **/

using namespace iCub::iDynTree;
using namespace yarp::math;
using namespace yarp::os;
using namespace wb_sot;

sot_VelKinCon_ctrl::sot_VelKinCon_ctrl(const int period,    const bool _LEFT_ARM_IMPEDANCE,
                                                            const bool _RIGHT_ARM_IMPEDANCE,
                                                            const bool _TORSO_IMPEDANCE,
                                                            paramHelp::ParamHelperServer* _ph):
    RateThread(period),
    _dT((double)period/1000),
    IYarp(),
    q_ref(1),
    dq_ref(1),
    ddq_ref(1),
    tau_gravity(1),
    q(1),
    q_left_arm(1),
    q_right_arm(1),
    q_left_leg(1),
    q_right_leg(1),
    q_torso(1),
    right_arm_pos_ref(3, 0.0),
    left_arm_pos_ref(3, 0.0),
    worldT(4,4),
    t_elapsed(0.0),
    gradientGq(1,0.0),
    eRWrist_p(3, 0.0),
    eRWrist_o(3, 0.0),
    eLWrist_p(3, 0.0),
    eLWrist_o(3, 0.0),
    eSwingFoot_p(3, 0.0),
    eSwingFoot_o(3, 0.0),
    eCoM(3, 0.0),
    LEFT_ARM_IMPEDANCE(_LEFT_ARM_IMPEDANCE),
    RIGHT_ARM_IMPEDANCE(_RIGHT_ARM_IMPEDANCE),
    TORSO_IMPEDANCE(_TORSO_IMPEDANCE),
    paramHelper(_ph),
    _convex_hull()
{
    int nJ = idynutils.coman_iDyn3.getNrOfDOFs();

    gradientGq.resize(nJ);
    Q_postural.resize(nJ, nJ);
    Q_postural.eye();

    zero.resize(1);
    zero.zero();

    q.resize(nJ, 0.0);
    q_ref.resize(nJ, 0.0);
    dq_ref.resize(nJ,0.0);
    ddq_ref.resize(nJ, 0.0);

    tau_gravity.resize(nJ, 0.0);

    q_left_arm.resize(IYarp.left_arm.getNumberOfJoints(), 0.0);
    q_right_arm.resize(IYarp.right_arm.getNumberOfJoints(), 0.0);
    q_left_leg.resize(IYarp.left_leg.getNumberOfJoints(), 0.0);
    q_right_leg.resize(IYarp.right_leg.getNumberOfJoints(), 0.0);
    q_torso.resize(IYarp.torso.getNumberOfJoints(), 0.0);

    worldT.eye();

    velocity_bounds_scale = 1.0;
    postural_weight_strategy = 0;

    is_clik = false;
    update_world = true;
}

/** @todo move to drc_shared */
void sot_VelKinCon_ctrl::parameterUpdated(const ParamProxyInterface *pd)
{
    return;
}

/** @todo move to drc_shared */
void sot_VelKinCon_ctrl::commandReceived(const CommandDescription &cd,
                                         const Bottle &params, Bottle &reply)
{
    switch(cd.id)
    {
    case wb_sot::COMMAND_ID_HELP:
        paramHelper->getHelpMessage(reply);
        break;
    case wb_sot::COMMAND_ID_SAVE_PARAMS:
        {
            std::string fileName = CONF_NAME;
            yarp::os::ResourceFinder rf;
            rf.setDefaultContext(MODULE_NAME);
            std::string folderName = rf.getContextPath() + "/";
            std::string confPath = folderName + fileName;
            std::vector<int> configIds;
            for(unsigned int i = 0; i < PARAM_ID_SIZE; ++i)
                if( sot_VelKinCon_ParamDescr[i]->ioType.value == paramHelp::PARAM_IN_OUT ||
                    sot_VelKinCon_ParamDescr[i]->ioType.value == paramHelp::PARAM_CONFIG )
                    configIds.push_back(i);

            std::cout << "Saving to " << confPath;

            std::stringstream ss;
            boost::posix_time::ptime pt = boost::posix_time::second_clock::local_time();
            boost::posix_time::time_facet* output_facet = new boost::posix_time::time_facet("%Y%m%dT%H%M%S%F%q");
            ss.imbue(std::locale(ss.getloc(), output_facet));
            ss << pt;
            std::string confPathWithTimestamp = confPath + "." + ss.str();

            std::cout << " and " << confPathWithTimestamp;
            reply.addString("saving...");

            if( paramHelper->writeParamsOnFile( confPathWithTimestamp,
                                                configIds.data(),
                                                configIds.size())) {
                if(boost::filesystem::exists(confPath))
                           boost::filesystem::remove(confPath);
                paramHelper->writeParamsOnFile( confPath,
                                                configIds.data(),
                                                configIds.size());
                reply.addString("ok");
            } else
                reply.addString("failed!");
        }
        break;
    }
}

//Qui devo prendere la configurazione iniziale del robot!
/** @todo move the paramHelper stuff to the yarp_interface */
bool sot_VelKinCon_ctrl::threadInit()
{
    IYarp.cleanPorts();

    dq_ref.zero();
    getFeedBack();

    //Here we set as initial reference the measured value: this will be the postural task
    q_ref = q;

    idynutils.updateiDyn3Model(q,true);

    boundsJointLimits = wb_sot::bounds::velocity::JointLimits::BoundPointer(
                            new wb_sot::bounds::velocity::JointLimits(
                                q,
                                idynutils.coman_iDyn3.getJointBoundMax(),
                                idynutils.coman_iDyn3.getJointBoundMin()));
    boundsJointVelocity = wb_sot::bounds::velocity::VelocityLimits::BoundPointer(
                            new wb_sot::bounds::velocity::VelocityLimits(0.3,_dT,q.size()));

    bounds = wb_sot::bounds::Aggregated::BoundPointer(
                new wb_sot::bounds::Aggregated(boundsJointLimits, boundsJointVelocity, q.size()));

    taskCartesianLWrist = boost::shared_ptr<wb_sot::tasks::velocity::Cartesian>(
                new wb_sot::tasks::velocity::Cartesian("cartesian::l_wrist",q,idynutils,
                                                        idynutils.left_arm.end_effector_name,
                                                        "world"));
    taskCartesianRWrist = boost::shared_ptr<wb_sot::tasks::velocity::Cartesian>(
                            new wb_sot::tasks::velocity::Cartesian("cartesian::r_wrist",q,idynutils,
                                                                    idynutils.right_arm.end_effector_name,
                                                                    "world"));
    taskCartesianRSole = boost::shared_ptr<wb_sot::tasks::velocity::Cartesian>(
                            new wb_sot::tasks::velocity::Cartesian("cartesian::r_sole",q,idynutils,
                                                                    idynutils.right_leg.end_effector_name,
                                                                    idynutils.left_leg.end_effector_name));
    taskCoM = boost::shared_ptr<wb_sot::tasks::velocity::CoM>(
        new wb_sot::tasks::velocity::CoM(q));

    boundsCoMVelocity = wb_sot::bounds::velocity::CoMVelocity::BoundPointer(
        new wb_sot::bounds::velocity::CoMVelocity(yarp::sig::Vector(3,0.03),idynutils,_dT,q.size()));
    taskCoM->getConstraints().push_back(boundsCoMVelocity);

    boundsConvexHullVelocity = wb_sot::bounds::velocity::ConvexHull::BoundPointer(
        new wb_sot::bounds::velocity::ConvexHull(idynutils,q.size()));
    taskCoM->getConstraints().push_back(boundsConvexHullVelocity);


    std::list<wb_sot::tasks::velocity::Cartesian::TaskPointer> cartesianTask;
    std::list<wb_sot::tasks::Aggregated::TaskPointer> firstTask;
    cartesianTask.push_back(taskCartesianLWrist);
    cartesianTask.push_back(taskCartesianRWrist);
    cartesianTask.push_back(taskCartesianRSole);
    firstTask = cartesianTask;
    firstTask.push_back(taskCoM);

    taskFirstAggregated = wb_sot::tasks::Aggregated::TaskPointer(
        new wb_sot::tasks::Aggregated(firstTask,q.size()));
    taskCartesianAggregated = wb_sot::tasks::Aggregated::TaskPointer(
        new wb_sot::tasks::Aggregated(cartesianTask,q.size()));

    taskMinimumEffort = boost::shared_ptr<wb_sot::tasks::velocity::MinimumEffort>(
        new wb_sot::tasks::velocity::MinimumEffort(q));
    taskPostural = boost::shared_ptr<wb_sot::tasks::velocity::Postural>(
        new wb_sot::tasks::velocity::Postural(q));

    std::list<wb_sot::tasks::Aggregated::TaskPointer> secondTask;
    secondTask.push_back(taskMinimumEffort);
    secondTask.push_back(taskPostural);

    taskSecondAggregated = wb_sot::tasks::Aggregated::TaskPointer(
        new wb_sot::tasks::Aggregated(secondTask,q.size()));

    stack_of_2_tasks.push_back(taskFirstAggregated);
    stack_of_2_tasks.push_back(taskSecondAggregated);

    stack_of_3_tasks.push_back(taskCoM);
    stack_of_3_tasks.push_back(taskCartesianAggregated);
    stack_of_3_tasks.push_back(taskSecondAggregated);

    support_foot_LinkIndex = idynutils.left_leg.index;
    swing_foot_LinkIndex = idynutils.right_leg.index;


    ROS_INFO("Initial Pose Ref left_arm:");   cartesian_utils::printHomogeneousTransform(taskCartesianLWrist->getReference());std::cout<<std::endl;
    ROS_INFO("Initial Pose Ref right_arm:");  cartesian_utils::printHomogeneousTransform(taskCartesianRWrist->getReference());std::cout<<std::endl;
    ROS_INFO("Initial Pose Ref swing_foot:"); cartesian_utils::printHomogeneousTransform(taskCartesianRSole->getReference()); std::cout<<std::endl;
    ROS_INFO("Initial Position Ref CoM: [ %s ]", taskCoM->getReference().toString().c_str());

if(RIGHT_ARM_IMPEDANCE) {
    ROS_INFO("Setting Impedance Mode for q_right_arm:");
    for(unsigned int i = 0; i < q_right_arm.size(); ++i)
        IYarp.right_arm.controlMode->setImpedancePositionMode(i);
} else {
    ROS_INFO("Setting Position Mode for q_right_arm:");
    for(unsigned int i = 0; i < q_right_arm.size(); ++i)
        IYarp.right_arm.controlMode->setPositionMode(i);
}

if(LEFT_ARM_IMPEDANCE) {
    ROS_INFO("Setting Impedance Mode for q_left_arm:");
    for(unsigned int i = 0; i < q_left_arm.size(); ++i)
        IYarp.left_arm.controlMode->setImpedancePositionMode(i);
} else {
    ROS_INFO("Setting Position Mode for q_left_arm:");
    for(unsigned int i = 0; i < q_left_arm.size(); ++i)
        IYarp.left_arm.controlMode->setPositionMode(i);
}

if(TORSO_IMPEDANCE) {
    ROS_INFO("Setting Impedance Mode for q_torso:");
    for(unsigned int i = 0; i < q_torso.size(); ++i)
        IYarp.torso.controlMode->setImpedancePositionMode(i);
} else {
    ROS_INFO("Setting Position Mode for q_torso:");
    for(unsigned int i = 0; i < q_torso.size(); ++i)
        IYarp.torso.controlMode->setPositionMode(i);
}

    ROS_INFO("Setting Position Mode for q_right_leg:");
    for(unsigned int i = 0; i < q_right_leg.size(); ++i)
        IYarp.right_leg.controlMode->setPositionMode(i);

    ROS_INFO("Setting Position Mode for q_left_leg:");
    for(unsigned int i = 0; i < q_left_leg.size(); ++i)
        IYarp.left_leg.controlMode->setPositionMode(i);

    if(is_clik)
        ROS_WARN("SoT is running as CLIK");
    else
        ROS_WARN("SoT is NOT running as CLIK");

    ROS_INFO("sot_VelKinCon START!!!");

    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_COMPUTATION_TIME,             &t_elapsed));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_LEFT_ARM_POSITION_ERROR,      eLWrist_p.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_LEFT_ARM_ORIENTATION_ERROR,   eLWrist_o.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_RIGHT_ARM_POSITION_ERROR,     eRWrist_p.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_RIGHT_ARM_ORIENTATION_ERROR,  eRWrist_o.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_SWING_FOOT_POSITION_ERROR,    eSwingFoot_p.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_SWING_FOOT_ORIENTATION_ERROR, eSwingFoot_o.data()));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_COM_POSITION_ERROR,           eCoM.data()));

    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_USE_3_STACKS,                 &use_3_stacks));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_MAX_JOINT_VELOCITY,           &max_joint_velocity));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_MAX_COM_VELOCITY,             &max_CoM_velocity));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_ORIENTATION_ERROR_GAIN,       &orientation_error_gain));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_POSTURAL_WEIGHT_COEFFICIENT,  &postural_weight_coefficient));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_MINEFFORT_WEIGHT_COEFFICIENT, &mineffort_weight_coefficient));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_POSTURAL_WEIGHT_STRATEGY,     &postural_weight_strategy));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_VELOCITY_BOUNDS_SCALE,        &velocity_bounds_scale));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QPOASES_NWSR0,                &qpOASES_NWSR0));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QPOASES_NWSR1,                &qpOASES_NWSR1));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QPOASES_NWSR2,                &qpOASES_NWSR2));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QPOASES_ENABLEREGULARISATION0,&qpOASES_enableRegularisation0));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QPOASES_ENABLEREGULARISATION1,&qpOASES_enableRegularisation1));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QPOASES_ENABLEREGULARISATION2,&qpOASES_enableRegularisation2));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER0,&qpOASES_eps0));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER1,&qpOASES_eps1));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER2,&qpOASES_eps2));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_CLIK,                                &is_clik));
    YARP_ASSERT(paramHelper->linkParam(PARAM_ID_WORLD_UPDATE,                        &update_world));

    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_HELP,           this));
    YARP_ASSERT(paramHelper->registerCommandCallback(COMMAND_ID_SAVE_PARAMS,    this));


    if(use_3_stacks)
    {
        qpOasesSolver = wb_sot::solvers::QPOases_sot::SolverPointer(
            new wb_sot::solvers::QPOases_sot(stack_of_3_tasks));
    }
    else
    {
        qpOasesSolver = wb_sot::solvers::QPOases_sot::SolverPointer(
            new wb_sot::solvers::QPOases_sot(stack_of_2_tasks));
    }

    return true;
}

void sot_VelKinCon_ctrl::run()
{
    std::list<KDL::Vector> points;
    std::vector<KDL::Vector> ch;

#ifdef DEBUG
    paramHelper->lock();
    paramHelper->readStreamParams();
#endif



    checkInput();

    if(is_clik)
        getFeedBack();
    else
        q += dq_ref;


    idynutils.updateiDyn3Model(q,update_world);

    if(controlLaw())
        move();


    IYarp.sendWorldToBaseLinkPose(idynutils.coman_iDyn3.getWorldBasePose());


    drc_shared::convex_hull::getSupportPolygonPoints(idynutils,points);
    if(_convex_hull.getConvexHull(points,ch)) {
        IYarp.sendCH(ch);
    }



#ifdef DEBUG
    paramHelper->sendStreamParams();
    paramHelper->unlock();
#endif
}

//Also here the configurations come in deg so we need to convert to rad!
void sot_VelKinCon_ctrl::getFeedBack()
{
    IYarp.left_arm.sense(q_left_arm);
    IYarp.right_arm.sense(q_right_arm);
    IYarp.left_leg.sense(q_left_leg);
    IYarp.right_leg.sense(q_right_leg);
    IYarp.torso.sense(q_torso);

    idynutils.fromRobotToIDyn(q_left_arm,q,idynutils.left_arm);
    idynutils.fromRobotToIDyn(q_right_arm,q,idynutils.right_arm);
    idynutils.fromRobotToIDyn(q_left_leg,q,idynutils.left_leg);
    idynutils.fromRobotToIDyn(q_right_leg,q,idynutils.right_leg);
    idynutils.fromRobotToIDyn(q_torso,q,idynutils.torso);
    
}

void sot_VelKinCon_ctrl::checkInput()
{
    IYarp.getLeftArmCartesianRef(left_arm_pos_ref);
    IYarp.getRightArmCartesianRef(right_arm_pos_ref);
    IYarp.getCoMCartesianRef(com_pos_ref);
    IYarp.getSwingFootCartesianRef(swing_foot_pos_ref);
}

/** Here we convert from rad to deg!
    The implemented control is like a velocity control: q_d = q + dq
**/
void sot_VelKinCon_ctrl::move()
{
    yarp::sig::Vector torso(q_torso.size(), 0.0);
    yarp::sig::Vector left_arm(q_left_arm.size(), 0.0);
    yarp::sig::Vector right_arm(q_right_arm.size(), 0.0);
    yarp::sig::Vector left_leg(q_left_leg.size(), 0.0);
    yarp::sig::Vector right_leg(q_right_leg.size(), 0.0);

    for(unsigned int i = 0; i < torso.size(); ++i){
        torso[i] = q[idynutils.torso.joint_numbers[i]] + dq_ref[idynutils.torso.joint_numbers[i]];
    }
    //Here we assumes that left and right arm has the same number of joints!
    for(unsigned int i = 0; i < left_arm.size(); ++i){
        left_arm[i] = q[idynutils.left_arm.joint_numbers[i]] + dq_ref[idynutils.left_arm.joint_numbers[i]];
        right_arm[i] = q[idynutils.right_arm.joint_numbers[i]] + dq_ref[idynutils.right_arm.joint_numbers[i]];
    }
    //Here we assumes that left and right leg has the same number of joints!
    for(unsigned int i = 0; i < left_leg.size(); ++i){
        left_leg[i] = q[idynutils.left_leg.joint_numbers[i]] + dq_ref[idynutils.left_leg.joint_numbers[i]];
        right_leg[i] = q[idynutils.right_leg.joint_numbers[i]] + dq_ref[idynutils.right_leg.joint_numbers[i]];
    }

    IYarp.torso.move(torso);
    IYarp.left_arm.move(left_arm);
    IYarp.right_arm.move(right_arm);
    IYarp.left_leg.move(left_leg);
    IYarp.right_leg.move(right_leg);

    /// GRAVITY COMPENSATION
    if(LEFT_ARM_IMPEDANCE) {
        yarp::sig::Vector tau_gravity_left_arm = getGravityCompensationTorque(idynutils.left_arm.joint_names);
        for(unsigned int i = 0; i < idynutils.left_arm.joint_names.size(); ++i)
            IYarp.left_arm.impedancePositionControl->setImpedanceOffset(i, tau_gravity_left_arm[i]);
    }
    if(RIGHT_ARM_IMPEDANCE) {
        yarp::sig::Vector tau_gravity_right_arm = getGravityCompensationTorque(idynutils.right_arm.joint_names);
        for(unsigned int i = 0; i < idynutils.right_arm.joint_names.size(); ++i)
            IYarp.right_arm.impedancePositionControl->setImpedanceOffset(i, tau_gravity_right_arm[i]);
    }
    if(TORSO_IMPEDANCE) {
        yarp::sig::Vector tau_gravity_torso = getGravityCompensationTorque(idynutils.torso.joint_names);
        for(unsigned int i = 0; i < idynutils.torso.joint_names.size(); ++i)
            IYarp.torso.impedancePositionControl->setImpedanceOffset(i, tau_gravity_torso[i]);
    }
}

bool sot_VelKinCon_ctrl::controlLaw()
{

    taskFirstAggregated->update(q);
    taskSecondAggregated->update(q);
    bounds->update(q);

    /** Set of last tasks **/
    /**
    *  Here we stack the last set of tasks:
    *
    *  Qdq + (q_ref - q) <-- Postural task
    *  (-grad(g(q))/tau_max)dq <-- Min effort
    *
    **/

    double l = postural_weight_coefficient;
    double l2 = 1.0 - l;

    // TODO: Aggregated has a setBeta(taskIndex,weight)
    taskPostural->setWeight(l*taskPostural->getWeight());
    taskMinimumEffort->setWeight(l2*taskMinimumEffort->getWeight());

    IYarp.tic();

    bool control_computed = false;
    control_computed = qpOasesSolver->solve(dq_ref);

    t_elapsed = IYarp.toc();
    static std::ofstream FILE("time.dat");
    FILE<<t_elapsed<<std::endl;
    FILE.flush();

    if(!control_computed) {
        ROS_ERROR("Error computing control");
    }
    return control_computed;
}


yarp::sig::Vector sot_VelKinCon_ctrl::getGravityCompensationTorque(const std::vector<string> &joint_names)
{
    yarp::sig::Vector tau(joint_names.size(), 0.0);
    int j = 0;
    for(unsigned int i = 0; i < joint_names.size(); ++i)
    {
        j = gravity_compensator_idynutils.coman_iDyn3.getDOFIndex(joint_names[i]);
        tau[i] = tau_gravity[j];
    }
    return tau;
}
