#include "sot_VelKinCon_ctrl.h"
#include <boost/foreach.hpp>


#define toRad(X) (X*M_PI/180.0)

/** TODO: PUT ALL THIS DEFINES IN A CONFIG FILE **/

#define TORSO_WEIGHT 1.0

/** ******************************************* **/

using namespace iCub::iDynTree;
using namespace yarp::math;

// Here it is the path to the URDF model
const std::string coman_model_folder = std::string(getenv("YARP_WORKSPACE")) + "/coman_yarp_apps/coman_urdf/coman.urdf";

sot_VelKinCon_ctrl::sot_VelKinCon_ctrl(const double period, int argc, char *argv[]):
    RateThread(int(period*1000.0)),
    IYarp(),
    q_ref(1),
    dq_ref(1),
    ddq_ref(1),
    q(1),
    q_left_arm(1),
    q_right_arm(1),
    q_left_leg(1),
    q_right_leg(1),
    q_torso(1)
{
    iDyn3Model();
    setJointNames();
    setControlledKinematicChainsLinkIndex();
    setControlledKinematicChainsJointNumbers();
    setQPostural();

    int nJ = coman_iDyn3.getNrOfDOFs();
    q.resize(nJ, 0.0);
    q_ref.resize(nJ, 0.0);
    dq_ref.resize(nJ,0.0);
    ddq_ref.resize(nJ, 0.0);

    IYarp.encodersMotor_left_arm->getAxes(&nJ);
    q_left_arm.resize(nJ, 0.0);
    IYarp.encodersMotor_right_arm->getAxes(&nJ);
    q_right_arm.resize(nJ, 0.0);
    IYarp.encodersMotor_left_leg->getAxes(&nJ);
    q_left_leg.resize(nJ, 0.0);
    IYarp.encodersMotor_right_leg->getAxes(&nJ);
    q_right_leg.resize(nJ, 0.0);
    IYarp.encodersMotor_torso->getAxes(&nJ);
    q_torso.resize(nJ, 0.0);
}

//Qui devo prendere la configurazione iniziale del robot!
bool sot_VelKinCon_ctrl::threadInit()
{
    getFeedBack();

    //Here we set as initial reference the measured value
    q_ref = q;

    updateiDyn3Model(true);

    std::cout<<"sot_VelKinCon START!!!"<<std::endl;
    return true;
}

void sot_VelKinCon_ctrl::run()
{

}

void sot_VelKinCon_ctrl::iDyn3Model()
{
    /// iDyn3 Model creation
    // Giving name to references for FT sensors and IMU
    std::vector<std::string> joint_sensor_names;
    joint_sensor_names.push_back("l_ankle_joint");
    joint_sensor_names.push_back("r_ankle_joint");
    waist_link_name = "Waist";

    if (!coman_model.initFile(coman_model_folder))
      std::cout<<"Failed to parse urdf robot model"<<std::endl;

    if (!kdl_parser::treeFromUrdfModel(coman_model, coman_tree))
      std::cout<<"Failed to construct kdl tree"<<std::endl;

    // Here the iDyn3 model of the robot is generated
    coman_iDyn3.constructor(coman_tree, joint_sensor_names, waist_link_name);
    std::cout<<"Loaded COMAN in iDyn3!"<<std::endl;

    int nJ = coman_iDyn3.getNrOfDOFs(); //29
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

    std::cout<<"#DOFS: "<<coman_iDyn3.getNrOfDOFs()<<std::endl;
    std::cout<<"#Links: "<<coman_iDyn3.getNrOfLinks()<<std::endl;
}

void sot_VelKinCon_ctrl::setControlledKinematicChainsLinkIndex()
{
    right_arm_name = "r_wrist";
    left_arm_name = "l_wrist";
    waist_LinkIndex = coman_iDyn3.getLinkIndex(waist_link_name);
    right_arm_LinkIndex = coman_iDyn3.getLinkIndex(right_arm_name);
    left_arm_LinkIndex = coman_iDyn3.getLinkIndex(left_arm_name);
    if(right_arm_LinkIndex == -1)
        std::cout << "Failed to get link index for right arm" << std::endl;
    if(left_arm_LinkIndex == -1)
        std::cout << "Failed to get link index for left arm" << std::endl;
    if(waist_LinkIndex == -1)
        std::cout << "Failed to get link index for Waist" << std::endl;
}

void sot_VelKinCon_ctrl::setControlledKinematicChainsJointNumbers()
{
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
    BOOST_FOREACH(std::string joint_name, torso_joint_names){
        std::cout<<coman_iDyn3.getDOFIndex(joint_name)<<" ";
        waist_joint_numbers.push_back(coman_iDyn3.getDOFIndex(joint_name));
    }
    std::cout<<std::endl;
}

void sot_VelKinCon_ctrl::setQPostural()
{
    int nJ = coman_iDyn3.getNrOfDOFs();

    Q_postural.resize(nJ, nJ);
    Q_postural.eye();

    yarp::sig::Vector qMax = coman_iDyn3.getJointBoundMax();
    yarp::sig::Vector qMin = coman_iDyn3.getJointBoundMin();

    Q_postural.diagonal(computeW(qMin, qMax, right_arm_joint_numbers,
                                 left_arm_joint_numbers, waist_joint_numbers));
}

yarp::sig::Vector sot_VelKinCon_ctrl::computeW(const yarp::sig::Vector &qMin,
                                               const yarp::sig::Vector &qMax,
                                               const std::vector<unsigned int>& right_arm_joint_numbers,
                                               const std::vector<unsigned int>& left_arm_joint_numbers,
                                               const std::vector<unsigned int>& waist_joint_numbers)
{
    yarp::sig::Vector i(qMax.size(), 1.0);
    yarp::sig::Vector w = i/(qMax-qMin);
    w*=w;
    std::cout<<"i/(qMax-qMin)^2: "<<w.toString()<<std::endl;

    std::vector<unsigned int> waist_left_arm_joint_numbers = waist_joint_numbers;
    std::vector<unsigned int> waist_right_arm_joint_numbers = waist_joint_numbers;
    waist_left_arm_joint_numbers.insert(waist_left_arm_joint_numbers.end(), left_arm_joint_numbers.begin(), left_arm_joint_numbers.end());
    waist_right_arm_joint_numbers.insert(waist_right_arm_joint_numbers.end(), right_arm_joint_numbers.begin(), right_arm_joint_numbers.end());

    std::cout<<"index weight waist_left_arm: ";
    for(unsigned int i = 0; i < waist_left_arm_joint_numbers.size(); ++i)
    {
        w[waist_left_arm_joint_numbers[i]]  *= (double)(waist_left_arm_joint_numbers.size() - i);
        std::cout<<(double)(waist_left_arm_joint_numbers.size() - i)<<" ";
    }
    std::cout<<std::endl;

    std::cout<<"index weight waist_right_arm: ";
    for(unsigned int i = 0; i < waist_right_arm_joint_numbers.size(); ++i)
    {
        w[waist_right_arm_joint_numbers[i]] *= (double)(waist_right_arm_joint_numbers.size() - i);
        std::cout<<(double)(waist_right_arm_joint_numbers.size() - i)<<" ";
    }
    std::cout<<std::endl;

    std::cout<<"index weight waist: ";
    for(unsigned int i = 0; i < waist_joint_numbers.size(); ++i) {
        w[waist_joint_numbers[i]] *= TORSO_WEIGHT;
        std::cout<<TORSO_WEIGHT<<" ";
    }
    std::cout<<std::endl;

    std::cout<<"w: "<<w.toString()<<std::endl;
    return w;
}

void sot_VelKinCon_ctrl::updateiDyn3Model(const bool set_world_pose = false)
{
    // Here we set these values in our internal model
    coman_iDyn3.setAng(q_ref);
    coman_iDyn3.setDAng(dq_ref);
    coman_iDyn3.setD2Ang(ddq_ref); // Since we want only the gravity term we set ddq = 0!
    // This is the fake Inertial Measure
    yarp::sig::Vector g(3);
    g[0] = 0; g[1] = 0; g[2] = 9.81;
    yarp::sig::Vector o(3);
    o[0] = 0; o[1] = 0; o[2] = 0;
    coman_iDyn3.setInertialMeasure(o, o, g);

    coman_iDyn3.kinematicRNEA();
    coman_iDyn3.computePositions();

    // Set World Pose: to do only once at the beginning
    if(set_world_pose)
    {
        yarp::sig::Vector foot_pose(3);
        foot_pose = coman_iDyn3.getPosition(coman_iDyn3.getLinkIndex("r_sole")).getCol(3).subVector(0,2);
        yarp::sig::Matrix worldT(4,4);
        worldT.eye();
        worldT(2,3) = -foot_pose(2);
        std::cout<<"World Base Pose:\n";
        std::cout<<worldT.toString()<<std::endl;
        coman_iDyn3.setWorldBasePose(worldT);
        coman_iDyn3.computePositions();
    }
}

//Also here the configurations come in deg so we need to convert to rad!
void sot_VelKinCon_ctrl::getFeedBack()
{
    IYarp.encodersMotor_left_arm->getEncoders(q_left_arm.data());
    IYarp.encodersMotor_right_arm->getEncoders(q_right_arm.data());
    IYarp.encodersMotor_left_leg->getEncoders(q_left_leg.data());
    IYarp.encodersMotor_right_leg->getEncoders(q_right_leg.data());
    IYarp.encodersMotor_torso->getEncoders(q_torso.data());

    //To make things faster: we suppose that arms has same number of dofs
    for(unsigned int i = 0; i < q_left_arm.size(); ++i)
    {
        q_left_arm[i] = toRad(q_left_arm[i]); //from deg to rad!
        q[coman_iDyn3.getDOFIndex(left_arm_joint_names[i])] = q_left_arm[i];
        q_right_arm[i] = toRad(q_right_arm[i]); //from deg to rad!
        q[coman_iDyn3.getDOFIndex(right_arm_joint_names[i])] = q_right_arm[i];
    }
    //To make things faster: we suppose that legs has same number of dofs
    for(unsigned int i = 0; i < q_left_leg.size(); ++i)
    {
        q_left_leg[i] = toRad(q_left_leg[i]); //from deg to rad!
        q[coman_iDyn3.getDOFIndex(left_leg_joint_names[i])] = q_left_leg[i];
        q_right_leg[i] = toRad(q_right_leg[i]); //from deg to rad!
        q[coman_iDyn3.getDOFIndex(right_leg_joint_names[i])] = q_right_leg[i];
    }
    for(unsigned int i = 0; i < q_torso.size(); ++i)
    {
        q_torso[i] = toRad(q_torso[i]); //from deg to rad!
        q[coman_iDyn3.getDOFIndex(torso_joint_names[i])] = q_torso[i];
    }
}
