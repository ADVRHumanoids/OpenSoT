#include "sot_VelKinCon.h"
#include <map>
#include "urdf_model/joint.h"
#include <boost/foreach.hpp>
#include <iCub/ctrl/math.h>

#define toRad(X) (X*M_PI/180.0)

#define DELTA_POSTURAL 1e-2
#define MAX_JOINT_VELOCITY 0.8 //[rad/sec]
#define MAX_CARTESIAN_VELOCITY 1.0 //[m/sec]
#define EPS_REGULARISATION 1e-9
#define RATE_PERIOD 25  // [ms]
#define TORSO_WEIGHT 1.0

// TO FIX:
//#define LEFT_ARM_CONTROL_PORT "/PDgravity/left_arm/set_ref:i"
//#define RIGHT_ARM_CONTROL_PORT "/PDgravity/right_arm/set_ref:i"
//#define WAIST_CONTROL_PORT ""

using namespace iCub::iDynTree;
using namespace yarp::math;

// Here it is the path to the URDF model
const std::string coman_model_folder = std::string(getenv("YARP_WORKSPACE")) + "/coman_yarp_apps/coman_urdf/coman.urdf";

sot_VelKinCon::sot_VelKinCon() :
    yarp(),
    yarp::os::RateThread(RATE_PERIOD*1000.0 /* [ms] */)
{
    /// iDyn3 Model creation
    // Giving name to references for FT sensors and IMU
    std::vector<std::string> joint_sensor_names;
    joint_sensor_names.push_back("l_ankle_joint");
    joint_sensor_names.push_back("r_ankle_joint");
    std::string waist_link_name = "Waist";

    if (!coman_model.initFile(coman_model_folder))
      std::cout<<"Failed to parse urdf robot model"<<std::endl;
    if (!kdl_parser::treeFromUrdfModel(coman_model, coman_tree))
      std::cout<<"Failed to construct kdl tree"<<std::endl;

    // Here the iDyn3 model of the robot is generated
    coman_iDyn3.constructor(coman_tree, joint_sensor_names, waist_link_name);
    int nJ = coman_iDyn3.getNrOfDOFs(); //23
    yarp::sig::Vector qMax; qMax.resize(nJ,0.0);
    yarp::sig::Vector qMin; qMin.resize(nJ,0.0);

    std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator i;
    for(i = coman_model.joints_.begin(); i != coman_model.joints_.end(); ++i) {
        int jIndex = coman_iDyn3.getDOFIndex(i->first);
        if(jIndex != -1) {
            qMax[jIndex] = i->second->limits->upper;
            qMin[jIndex] = i->second->limits->lower;
        }
    }

    coman_iDyn3.setJointBoundMax(qMax);
    coman_iDyn3.setJointBoundMin(qMin);
    std::cout<<"Loaded COMAN in iDyn3!"<<std::endl;
    printModelInformation();

    std::string right_arm_name = "r_wrist";
    std::string left_arm_name = "l_wrist";
    waist_LinkIndex = coman_iDyn3.getLinkIndex(waist_link_name);
    right_arm_LinkIndex = coman_iDyn3.getLinkIndex(right_arm_name);
    left_arm_LinkIndex = coman_iDyn3.getLinkIndex(left_arm_name);
    if(right_arm_LinkIndex == -1 || left_arm_LinkIndex == -1)
        std::cout << "Failed to get link index for arms" << std::endl;

    /// PolyDriver Creation
    yarp::os::ConstString s_r("/sot_VelKinCon/coman/right_arm");
    yarp::os::ConstString ss_r("/coman/right_arm");
    yarp::os::ConstString s_l("/sot_VelKinCon/coman/left_arm");
    yarp::os::ConstString ss_l("/coman/left_arm");
    yarp::os::ConstString s_w("/sot_VelKinCon/coman/torso");
    yarp::os::ConstString ss_w("/coman/torso");

    yarp::os::Property options_r;
    yarp::os::Property options_l;
    yarp::os::Property options_w;

    options_r.put("robot", "coman");
    options_r.put("device", "remote_controlboard");
    options_r.put("local", s_r.c_str());
    options_r.put("remote", ss_r.c_str());

    options_l.put("robot", "coman");
    options_l.put("device", "remote_controlboard");
    options_l.put("local", s_l.c_str());
    options_l.put("remote", ss_l.c_str());

    options_w.put("robot", "coman");
    options_w.put("device", "remote_controlboard");
    options_w.put("local", s_w.c_str());
    options_w.put("remote", ss_w.c_str());

    right_arm_polyDriver.open(options_r);
    if (!right_arm_polyDriver.isValid())
         std::cout<<"Device not available: right arm."<<std::endl;
#ifndef RIGHT_ARM_CONTROL_PORT
    right_arm_polyDriver.view(right_arm_controlMode);
    right_arm_polyDriver.view(right_arm_posControl);
    right_arm_polyDriver.view(right_arm_encodersMotor);
#endif

    left_arm_polyDriver.open(options_l);
    if (!left_arm_polyDriver.isValid())
         std::cout<<"Device not available: left arm."<<std::endl;
#ifndef LEFT_ARM_CONTROL_PORT
    left_arm_polyDriver.view(left_arm_controlMode);
    left_arm_polyDriver.view(left_arm_posControl);
    left_arm_polyDriver.view(left_arm_encodersMotor);
#endif

    waist_polyDriver.open(options_w);
    if (!waist_polyDriver.isValid())
         std::cout<<"Device not available: waist."<<std::endl;
#ifndef WAIST_CONTROL_PORT
    waist_polyDriver.view(waist_controlMode);
    waist_polyDriver.view(waist_posControl);
    waist_polyDriver.view(waist_encodersMotor);
#endif

    right_arm_joint_names.push_back("RShSag");
    right_arm_joint_names.push_back("RShLat");
    right_arm_joint_names.push_back("RShYaw");
    right_arm_joint_names.push_back("RElbj");

    left_arm_joint_names.push_back("LShSag");
    left_arm_joint_names.push_back("LShLat");
    left_arm_joint_names.push_back("LShYaw");
    left_arm_joint_names.push_back("LElbj");

    waist_joint_names.push_back("WaistSag");
    waist_joint_names.push_back("WaistLat");
    waist_joint_names.push_back("WaistYaw");

    std::cout<<"Right Arm joint indices: \n";
    BOOST_FOREACH(std::string joint_name, right_arm_joint_names){
        std::cout<<coman_iDyn3.getDOFIndex(joint_name)<<" ";
        right_arm_joint_numbers.push_back(coman_iDyn3.getDOFIndex(joint_name));
    }
    std::cout<<std::endl;
    std::cout<<"Left Arm joint indices: \n";
    BOOST_FOREACH(std::string joint_name, left_arm_joint_names){
        std::cout<<coman_iDyn3.getDOFIndex(joint_name)<<" ";
        left_arm_joint_numbers.push_back(coman_iDyn3.getDOFIndex(joint_name));
    }
    std::cout<<std::endl;
    std::cout<<"Waist joint indices: \n";
    BOOST_FOREACH(std::string joint_name, waist_joint_names){
        std::cout<<coman_iDyn3.getDOFIndex(joint_name)<<" ";
        waist_joint_numbers.push_back(coman_iDyn3.getDOFIndex(joint_name));
    }
    std::cout<<std::endl;

    Q_postural.resize(nJ, nJ);
    Q_postural.eye();
    setQ_postural(); //Add weights!
}

bool sot_VelKinCon::threadInit() {

    /* setting q_ref */
    int nj = coman_iDyn3.getNrOfDOFs();
    q_ref.resize(nj, 0.0+DELTA_POSTURAL);

    // Here we initialize the state before it is read from the robot/simulator
    for(unsigned int i = 0; i < coman_iDyn3.getNrOfDOFs(); ++i){
        q.push_back(0.0);
        dq_ref.push_back(0.0);
        ddq_ref.push_back(0.0);
    }

    // Here we set these values in our internal model
    coman_iDyn3.setAng(q_ref);
    coman_iDyn3.setDAng(dq_ref);
    coman_iDyn3.setD2Ang(ddq_ref); // Since we want only the gravity term we set ddq = 0!
    // This is the fake Inertial Measure
    yarp::sig::Vector g(3);
    g[0] = 0; g[1] = 0; g[2] = -9.81;
    yarp::sig::Vector o(3);
    o[0] = 0; o[1] = 0; o[2] = 0;
    coman_iDyn3.setInertialMeasure(o, o, g);


    // initializing x_refs from zero configuration
    coman_iDyn3.kinematicRNEA();
    coman_iDyn3.computePositions();
    // Set World Pose
    yarp::sig::Vector foot_pose(3);
    foot_pose = coman_iDyn3.getPosition(coman_iDyn3.getLinkIndex("r_sole")).getCol(3).subVector(0,2);
    yarp::sig::Matrix worldT(4,4);
    worldT.eye();
    worldT(2,3) = -foot_pose(2);
    std::cout<<"World Base Pose:\n";
    std::cout<<worldT.toString()<<std::endl;
    coman_iDyn3.setWorldBasePose(worldT);
    coman_iDyn3.computePositions();

    right_arm_pos_ref = coman_iDyn3.getPosition(right_arm_LinkIndex).getCol(3).subVector(0,2);
    left_arm_pos_ref = coman_iDyn3.getPosition(left_arm_LinkIndex).getCol(3).subVector(0,2);

    std::cout<<"Ref configuration for right arm is :[ ";
    for(unsigned int i = 0; i < 3; ++i){
        std::cout<<right_arm_pos_ref[i]<<" ";
    }
    std::cout<<" ]"<<std::endl;

    std::cout<<"Ref configuration for left arm is :[ ";
    for(unsigned int i = 0; i < 3; ++i){
        std::cout<<left_arm_pos_ref[i]<<" ";
    }
    std::cout<<" ]"<<std::endl;

    // Connecting Port to set end effector position reference
    right_arm_pos_ref_port.open("/sot_VelKinCon/coman/right_arm/position/ref:i");
    left_arm_pos_ref_port.open("/sot_VelKinCon/coman/left_arm/position/ref:i");

    right_arm_configuration_ref_port.open("/sot_VelKinCon/coman/right_arm/configuration/ref:o");
    left_arm_configuration_ref_port.open("/sot_VelKinCon/coman/left_arm/configuration/ref:o");
    waist_configuration_ref_port.open("/sot_VelKinCon/coman/torso/configuration/ref:o");

#ifndef LEFT_ARM_CONTROL_PORT
    for(unsigned int i = 0; i < left_arm_joint_numbers.size(); ++i) {
        left_arm_controlMode->setPositionMode(i); // Here we set the joints of the right/left arm to be velocity controlled
    }
#else
    yarp.connect("/sot_VelKinCon/coman/left_arm/configuration/ref:o", LEFT_ARM_CONTROL_PORT);
#endif
#ifndef RIGHT_ARM_CONTROL_PORT
    for(unsigned int i = 0; i < right_arm_joint_numbers.size(); ++i) {
        right_arm_controlMode->setPositionMode(i); // Here we set the joints of the right/left arm to be velocity controlled
    }
#else
    yarp.connect("/sot_VelKinCon/coman/right_arm/configuration/ref:o", RIGHT_ARM_CONTROL_PORT);
#endif
#ifndef WAIST_CONTROL_PORT
    for(unsigned int i =0; i < waist_joint_numbers.size(); ++i) {
        waist_controlMode->setPositionMode(i);
    }
#else
    yarp.connect("/sot_VelKinCon/coman/torso/configuration/ref:o", WAIST_CONTROL_PORT);
#endif
    return true;
}

void sot_VelKinCon::threadRelease() {
    // Closing ports
    right_arm_pos_ref_port.close();
    left_arm_pos_ref_port.close();
    right_arm_configuration_ref_port.close();
    left_arm_configuration_ref_port.close();
    waist_configuration_ref_port.close();
}

void sot_VelKinCon::run() {
    double t_old = yarp::os::Time::now();

    yarp::sig::Vector q_arm_state(4);
#ifndef RIGHT_ARM_CONTROL_PORT
    right_arm_encodersMotor->getEncoders(q_arm_state.data());
    for(unsigned int i = 0; i < q_arm_state.size(); ++i)
        q[right_arm_joint_numbers[i]] = toRad(q_arm_state[i]);
#endif
    q_arm_state.zero();
#ifndef LEFT_ARM_CONTROL_PORT
    left_arm_encodersMotor->getEncoders(q_arm_state.data());
    for(unsigned int i = 0; i < q_arm_state.size(); ++i)
        q[left_arm_joint_numbers[i]] = toRad(q_arm_state[i]);
#endif
#ifndef WAIST_CONTROL_PORT
    yarp::sig::Vector q_waist_state(3);
    waist_encodersMotor->getEncoders(q_waist_state.data());
    for(unsigned int i = 0; i < q_waist_state.size(); ++i)
        q[waist_joint_numbers[i]] = toRad(q_waist_state[i]);
#endif


    // Here we read input ports in meters
    yarp::os::Bottle* bot_ref_r = right_arm_pos_ref_port.read(false);
    yarp::os::Bottle* bot_ref_l = left_arm_pos_ref_port.read(false);
    if(bot_ref_r != NULL){
        std::cout<<"New ref configuration for right arm is :[ ";
        for(unsigned int i = 0; i < 3; ++i){
            right_arm_pos_ref[i] = bot_ref_r->get(i).asDouble();
            std::cout<<right_arm_pos_ref[i]<<" ";
        }
        std::cout<<" ] [m]"<<std::endl;
    }
    if(bot_ref_l != NULL){
        std::cout<<"New ref configuration for left arm is :[ ";
        for(unsigned int i = 0; i < 3; ++i){
            left_arm_pos_ref[i] = bot_ref_l->get(i).asDouble();
            std::cout<<left_arm_pos_ref[i]<<" ";
        }
        std::cout<<" ] [m]"<<std::endl;
    }



    // This command update the kinematic internal model with new joints position, velocity and accelerations
    /* We consider the robot stiff so we do not do any difference between link and motor side */

    coman_iDyn3.setAng(q); // Here we set in the internal model the joint position...
    if(coman_iDyn3.kinematicRNEA()) // This compute kinematic informations
    {
        coman_iDyn3.computePositions();
        yarp::sig::Vector pos_R = coman_iDyn3.getPosition(right_arm_LinkIndex).getCol(3).subVector(0,2);
        yarp::sig::Vector pos_L = coman_iDyn3.getPosition(left_arm_LinkIndex).getCol(3).subVector(0,2);

        yarp::sig::Matrix JRWrist;
        if(!coman_iDyn3.getJacobian(right_arm_LinkIndex,JRWrist))
            std::cout << "Error computing Jacobian for Right Wrist" << std::endl;
        JRWrist = JRWrist.removeRows(3,3);    // getting only position part of Jacobian
        JRWrist = JRWrist.removeCols(0,6);    // removing unactuated joints (floating base)

        yarp::sig::Matrix JLWrist;
        if(!coman_iDyn3.getJacobian(left_arm_LinkIndex,JLWrist))
            std::cout << "Error computing Jacobian for Left Wrist" << std::endl;
        JLWrist = JLWrist.removeRows(3,3);   // getting only position part of Jacobian
        JLWrist = JLWrist.removeCols(0,6);    // removing unactuated joints (floating base)

        /// Each Jacobian contains waist + arm
        for(unsigned int i = 0; i < JLWrist.cols(); ++i)
        {
            bool set_zero = true;
            for(unsigned int j = 0; j < right_arm_joint_names.size(); ++j){
                if(i == right_arm_joint_numbers[j]){
                    set_zero = false;
                    break;}
            }
            for(unsigned int j = 0; j < waist_joint_names.size(); ++j){
                if(i == waist_joint_numbers[j]){
                    set_zero = false;
                    break;}
            }
            if(set_zero){
                for(unsigned int k = 0; k < 3; ++k)
                    JRWrist(k,i) = 0.0;
            }

            set_zero = true;
            for(unsigned int j = 0; j < left_arm_joint_names.size(); ++j){
                if(i == left_arm_joint_numbers[j]){
                    set_zero = false;
                    break;}
            }
            for(unsigned int j = 0; j < waist_joint_names.size(); ++j){
                if(i == waist_joint_numbers[j]){
                    set_zero = false;
                    break;}
            }
            if(set_zero){
                for(unsigned int k = 0; k < 3; ++k)
                    JLWrist(k,i) = 0.0;
            }
        }
//        std::cout<<"JRWRIST: \n"<<JRWrist.submatrix(0,2,0,14).toString(2)<<std::endl;
//        std::cout<<JRWrist.submatrix(0,2,15,22).toString(2)<<std::endl;
//        std::cout<<std::endl;
//        std::cout<<"JLWRIST: \n"<<JLWrist.submatrix(0,2,0,14).toString(2)<<std::endl;
//        std::cout<<JLWrist.submatrix(0,2,15,22).toString(2)<<std::endl;
//        std::cout<<std::endl;


        yarp::sig::Vector eRWrist = right_arm_pos_ref - pos_R ;
        yarp::sig::Vector eLWrist = left_arm_pos_ref - pos_L ;
        yarp::sig::Matrix J1 = yarp::math::pile(JRWrist, JLWrist);
        yarp::sig::Vector e1 = MAX_JOINT_VELOCITY*yarp::math::cat(eRWrist, eLWrist);
        yarp::sig::Vector eq = MAX_CARTESIAN_VELOCITY*(q_ref - q); // postural error

//        std::cout << "RWrist error: ";
//        std::cout<<eRWrist.toString()<<std::endl;

//        std::cout << "LWrist error: ";
//        std::cout<<eLWrist.toString()<<std::endl;

//        std::cout<<"Posture error: [ "<<eq.toString()<<" ]"<<std::endl;

        // getting dq in rads, converting it in degs
        bool control_computed = false;
        control_computed = computeControlHQP(J1, e1, Q_postural, eq, dq_ref);
        if(!control_computed) {
            std::cout << "Error computing control" << std::endl;
        }

        yarp::os::Bottle left_arm_bottle;
        yarp::os::Bottle right_arm_bottle;
        yarp::os::Bottle waist_bottle;
        for(unsigned int i = 0; i < right_arm_joint_numbers.size(); ++i){
            double qi_right_arm = (q[right_arm_joint_numbers[i]] + dq_ref[right_arm_joint_numbers[i]]) * CTRL_RAD2DEG;
#ifndef RIGHT_ARM_CONTROL_PORT
            right_arm_posControl->setRefSpeed(i, MAX_JOINT_VELOCITY*CTRL_RAD2DEG);
            right_arm_posControl->positionMove(i, qi_right_arm);
#else
            q[right_arm_joint_numbers[i]] += dq_ref[right_arm_joint_numbers[i]];
#endif
            right_arm_bottle.addDouble(qi_right_arm);

            double qi_left_arm = (q[left_arm_joint_numbers[i]] + dq_ref[left_arm_joint_numbers[i]]) * CTRL_RAD2DEG;
#ifndef LEFT_ARM_CONTROL_PORT
            left_arm_posControl->setRefSpeed(i, MAX_JOINT_VELOCITY*CTRL_RAD2DEG);
            left_arm_posControl->positionMove(i, qi_left_arm);
#else
            q[left_arm_joint_numbers[i]] += dq_ref[left_arm_joint_numbers[i]];
#endif
            left_arm_bottle.addDouble(qi_left_arm);
        }
        for(unsigned int i = 0; i < waist_joint_numbers.size(); ++i){
            double qi_waist = (q[waist_joint_numbers[i]] + dq_ref[waist_joint_numbers[i]]) * CTRL_RAD2DEG;
#ifndef WAIST_CONTROL_PORT
            waist_posControl->setRefSpeed(i, MAX_JOINT_VELOCITY*CTRL_RAD2DEG);
            waist_posControl->positionMove(i, qi_waist);
#else
            q[waist_joint_numbers[i]] += dq_ref[waist_joint_numbers[i]]
#endif
            waist_bottle.addDouble(qi_waist);
        }
        right_arm_configuration_ref_port.write(right_arm_bottle);
        left_arm_configuration_ref_port.write(left_arm_bottle);
        waist_configuration_ref_port.write(waist_bottle);
    }
    else
        std::cout<<"ERROR computing kinematicRNEA!"<<std::endl;

    double t_now = yarp::os::Time::now();
    //std::cout<<"dt_comp: "<<t_now - t_old<<std::endl;
}

void sot_VelKinCon::printModelInformation()
{
    std::cout<<"#DOFS: "<<coman_iDyn3.getNrOfDOFs()<<std::endl;
    std::cout<<"#Links: "<<coman_iDyn3.getNrOfLinks()<<std::endl;
}

/**
  * \brief computeControlHQP
  *
  */
bool sot_VelKinCon::computeControlHQP( const yarp::sig::Matrix& J1,
                                       const yarp::sig::Vector& e1,
                                       const yarp::sig::Matrix& J2,
                                       const yarp::sig::Vector& eq,
                                       yarp::sig::Vector& dq_ref)
{

    int nj = coman_iDyn3.getNrOfDOFs();
    static yarp::sig::Vector dq1(nj,0.0);
    static yarp::sig::Vector y1(nj,0.0);
    static yarp::sig::Vector dq2(nj, 0.0);  // dq: desired joint vel; eq: joint space position error
    static yarp::sig::Vector y2(nj, 0.0);  // dq: desired joint vel; eq: joint space position error

    static qpOASES::Bounds bounds1;
    static qpOASES::Bounds bounds2;
    static qpOASES::Constraints constraints2;
    bool initial_guess = false;



    /*
      We solve a single QP where the priority between
      different tasks is set by using a weight matrix Q

      min         (Ax - b)'Q(Ax - b)
      subj to     l <=   x <=  u
    */

    /*
      QPOASES::Quadratic_program solves by default a quadratic problem in the form
      min        .5 x'Hx + x'g
      subj to  Alb <= Ax <= Aub
                 l <=  x <= u
     */

    int njTask1 = J1.rows();              // size for task 1
    double dT = getRate()/1000.0;        // thread rate in ms
    yarp::sig::Matrix H1 = J1.transposed()*J1;         // size of problem is bigger than the size of task because we need the extra slack variables
    yarp::sig::Vector g1 = -1.0*J1.transposed()*e1;

    yarp::sig::Matrix H2 = J2.transposed();
    yarp::sig::Vector g2 = -1.0*J2.transposed()*eq;

    yarp::sig::Vector qMax = coman_iDyn3.getJointBoundMax();
    //std::cout << "qMax: "<<qMax.toString()<<std::endl;

    yarp::sig::Vector qMin = coman_iDyn3.getJointBoundMin();
    //std::cout << "qMin: "<<qMin.toString()<<std::endl;

    yarp::sig::Vector u1(nj); yarp::sig::Vector l1(nj); //Joint Limits constraints
    u1 = (qMax - q)/(10.0*dT);
    l1 = (qMin - q)/(10.0*dT);

    yarp::sig::Vector u2(nj, MAX_JOINT_VELOCITY*dT); //Max velocity
    yarp::sig::Vector u(nj); yarp::sig::Vector l(nj);
    for(unsigned int i = 0; i < nj; ++i){
        u[i] = std::min(u1[i], u2[i]);
        l[i] = std::max(l1[i], -u2[i]);
    }

    //    std::cout << "u: "<<u.toString()<<std::endl;
    //    std::cout << "l: "<<l.toString()<<std::endl;


    USING_NAMESPACE_QPOASES
    /* Setting up QProblem object. */
    Options qpOasesOptionsqp1;
    qpOasesOptionsqp1.printLevel = PL_HIGH;
    qpOasesOptionsqp1.setToReliable();
//    qpOasesOptionsqp1.enableRegularisation = BT_TRUE;
//    qpOasesOptionsqp1.numRegularisationSteps = 1;
//    qpOasesOptionsqp1.numRefinementSteps = 1;
//    qpOasesOptionsqp1.epsRegularisation = EPS_REGULARISATION;
    QProblemB qp1( nj, HST_SEMIDEF);
    qp1.setOptions( qpOasesOptionsqp1 );

    Options qpOasesOptionsqp2;
    qpOasesOptionsqp2.printLevel = PL_HIGH;
    qpOasesOptionsqp2.setToReliable();
    QProblem qp2( nj, njTask1, HST_POSDEF);
    qp2.setOptions( qpOasesOptionsqp2 );

    /* Solve first QP. */
    int nWSR = 2^32;
    //real_t cpu_time1 = 0.004;
    if(initial_guess==true)
        qp1.init( H1.data(),g1.data(), l.data(), u.data(),nWSR,0, dq1.data(), y1.data(), &bounds1);
    else
        qp1.init( H1.data(),g1.data(), l.data(), u.data(), nWSR,0);

    if(dq1.size() != qp1.getNV()) {
        dq1.resize(qp1.getNV());
        initial_guess = false;
    }
    if(y1.size() != qp1.getNV()) {
        y1.resize(qp1.getNV());
        initial_guess = false;
    }

    int success1 = -1;
    int success2 = -1;

    success1 = qp1.getPrimalSolution( dq1.data() );
    qp1.getDualSolution(y1.data());
    qp1.getBounds(bounds1);

    if(success1== RET_QP_NOT_SOLVED ||
            (success1 != RET_QP_SOLVED && success1 != SUCCESSFUL_RETURN)) {
        std::cout<<"ERROR OPTIMIZING FIRST TASK! ERROR #"<<success1<<std::endl;
        initial_guess = false;
    } else {
        //yarp::sig::Vector residual(nj);
        //residual = H1*dq1 + g1;
        //std::cout<<"RESIDUAL FIRST TASK: [ "<<residual.toString()<<" ]"<<std::endl;

        /* Solve second QP. */
        yarp::sig::Vector b2 = J1*dq1;
        yarp::sig::Vector b2u = b2;
        yarp::sig::Vector b2l = b2;
        nWSR = 2^32;
        //real_t cpu_time2 = 0.002;
        if(initial_guess == true)
            qp2.init( H2.data(),g2.data(), J1.data(), l.data(), u.data(),
                      b2l.data(), b2u.data(), nWSR, 0, dq2.data(), y2.data(),
                      &bounds2, & constraints2);
        else
            qp2.init( H2.data(),g2.data(), J1.data(), l.data(), u.data(),
                      b2l.data(), b2u.data(), nWSR, 0);
        if(dq2.size() != qp2.getNV()) {
            dq2.resize(qp2.getNV());
            initial_guess = false;
        }
        if(y2.size() != qp2.getNV() + qp2.getNC()) {
            y2.resize(qp2.getNV() + qp2.getNC());
            initial_guess = false;
        }
        success2 = qp2.getPrimalSolution( dq2.data() );
        qp2.getDualSolution(y2.data());
        qp2.getBounds(bounds2);
        qp2.getConstraints(constraints2);
        if(success2 == RET_QP_NOT_SOLVED ||
                (success2 != RET_QP_SOLVED && success2 != SUCCESSFUL_RETURN)) {
            std::cout<<"ERROR OPTIMIZING POSTURE TASK! ERROR #"<<success2<<std::endl;
            initial_guess = false;
        } else {
            dq_ref = dq2;
//            std::cout<<"dq2 = [ "<<dq2.subVector(waist_joint_numbers[0],
//                                                 waist_joint_numbers[2]).toString()<<" ]\n";
            return true;
        }
    }

//    yarp::sig::Vector residual2(nj);
//    residual2 = H2*dq + g2;
//    std::cout<<"RESIDUAL POSTURE TASK: [ "<<residual2.toString()<<" ]"<<std::endl;


//    yarp::sig::Vector dx = J1*dq - e1;
//    std::cout<<"dx: "<<dx.toString()<<std::endl;

    return false;
}

void sot_VelKinCon::setQ_postural()
{
    yarp::sig::Vector qMax = coman_iDyn3.getJointBoundMax();
    yarp::sig::Vector qMin = coman_iDyn3.getJointBoundMin();

    yarp::sig::Vector i(qMax.size(),1.0);
    yarp::sig::Vector w = i/(qMax-qMin);
    w*=w;

    std::vector<unsigned int> waist_left_arm_joint_numbers = waist_joint_numbers;
    std::vector<unsigned int> waist_right_arm_joint_numbers = waist_joint_numbers;
    waist_left_arm_joint_numbers.insert(waist_left_arm_joint_numbers.end(), left_arm_joint_numbers.begin(), left_arm_joint_numbers.end());
    waist_right_arm_joint_numbers.insert(waist_right_arm_joint_numbers.end(), right_arm_joint_numbers.begin(), right_arm_joint_numbers.end());

    for(unsigned int i = 0; i < waist_left_arm_joint_numbers.size(); ++i)
    {
        w[waist_left_arm_joint_numbers[i]]  *= (double)(waist_left_arm_joint_numbers.size() - i);
    }

    for(unsigned int i = waist_joint_numbers.size(); i < waist_right_arm_joint_numbers.size(); ++i)
    {
        w[waist_right_arm_joint_numbers[i]] *= (double)(waist_right_arm_joint_numbers.size() - i);
    }
    for(unsigned int i = 0; i < waist_joint_numbers.size(); ++i) {
        w[waist_joint_numbers[i]] *= TORSO_WEIGHT;
    }

    std::cout << "Q_w: " << w.toString() << std::endl;
    Q_postural.diagonal(w);
}
