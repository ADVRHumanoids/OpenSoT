#include "convex_hull.h"

using namespace wb_sot;

std::string foot_reference_frames::_UPPER_LEFT = "_upper_left_joint";
std::string foot_reference_frames::_LOWER_LEFT = "_lower_left_joint";
std::string foot_reference_frames::_UPPER_RIGHT = "_upper_right_joint";
std::string foot_reference_frames::_LOWER_RIGHT = "_lower_right_joint";



void convex_hull::init(const boost::shared_ptr<const urdf::ModelInterface> &robot_urdf,
                         const KDL::Frame &l_ankle_CoM, const KDL::Frame &r_ankle_CoM)
{
    _robot_urdf = robot_urdf;
    _l_ankle_CoM = l_ankle_CoM;
    _r_ankle_CoM = r_ankle_CoM;

    getRefFrames("l_foot", "l_sole_joint", _l_foot);
    getRefFrames("r_foot", "r_sole_joint", _r_foot);
    std::cout<<"INITIALIZED CONVEX HULL"<<std::endl;
}

void convex_hull::getRefFrames(const std::string &foot_name, const std::string &sole_name, foot_reference_frames &foot)
{   
    boost::shared_ptr<const urdf::Joint> joint_sole = _robot_urdf->getJoint(sole_name);
    urdf::Pose pose = joint_sole->parent_to_joint_origin_transform;
    KDL::Frame sole_frame;
    URDFPose2KDLFrame(pose, sole_frame);

    std::cout<<foot_name + foot_reference_frames::_LOWER_LEFT<<std::endl;
    boost::shared_ptr<const urdf::Joint> joint = _robot_urdf->getJoint(foot_name + foot_reference_frames::_LOWER_LEFT);
    pose = joint->parent_to_joint_origin_transform;
    URDFPose2KDLFrame(pose, foot.lower_left);

    joint = _robot_urdf->getJoint(foot_name + foot_reference_frames::_LOWER_RIGHT);
    pose = joint->parent_to_joint_origin_transform;
    URDFPose2KDLFrame(pose, foot.lower_right);

    joint = _robot_urdf->getJoint(foot_name + foot_reference_frames::_UPPER_LEFT);
    pose = joint->parent_to_joint_origin_transform;
    URDFPose2KDLFrame(pose, foot.upper_left);

    joint = _robot_urdf->getJoint(foot_name + foot_reference_frames::_UPPER_RIGHT);
    pose = joint->parent_to_joint_origin_transform;
    URDFPose2KDLFrame(pose, foot.upper_right);

    //Since we need from ankle...
    foot.lower_left = sole_frame * foot.lower_left;
    foot.lower_right = sole_frame * foot.lower_right;
    foot.upper_left = sole_frame * foot.upper_left;
    foot.upper_right = sole_frame * foot.upper_right;
}

convex_hull::~convex_hull()
{

}

void convex_hull::URDFPose2KDLFrame(const urdf::Pose &pose, KDL::Frame &T)
{
    double qx, qy, qz, qw;
    pose.rotation.getQuaternion(qx, qy, qz, qw);
    T.M = T.M.Quaternion(qx, qy, qz, qw);

    T.p = KDL::Vector(pose.position.x, pose.position.y, pose.position.z);
}

void convex_hull::computeConvexHull()
{
    //We compute the transformation from CoM to reference frames in foot
    foot_reference_frames l_foot_CoM;
    l_foot_CoM.upper_right = _l_ankle_CoM.Inverse() * _l_foot.upper_right;
    l_foot_CoM.upper_left = _l_ankle_CoM.Inverse() * _l_foot.upper_left;
    l_foot_CoM.lower_right = _l_ankle_CoM.Inverse() * _l_foot.lower_right;
    l_foot_CoM.lower_left = _l_ankle_CoM.Inverse() * _l_foot.lower_left;

    std::cout<<"l_foot upper_right:"<<std::endl;cartesian_utils::printKDLFrame(l_foot_CoM.upper_right);
    std::cout<<"l_foot upper_left:"<<std::endl;cartesian_utils::printKDLFrame(l_foot_CoM.upper_left);
    std::cout<<"l_foot lower_right:"<<std::endl;cartesian_utils::printKDLFrame(l_foot_CoM.lower_right);
    std::cout<<"l_foot lower_left:"<<std::endl;cartesian_utils::printKDLFrame(l_foot_CoM.lower_left);

    foot_reference_frames r_foot_CoM;
    r_foot_CoM.upper_right = _r_ankle_CoM.Inverse() * _r_foot.upper_right;
    r_foot_CoM.upper_left = _r_ankle_CoM.Inverse() * _r_foot.upper_left;
    r_foot_CoM.lower_right = _r_ankle_CoM.Inverse() * _r_foot.lower_right;
    r_foot_CoM.lower_left = _r_ankle_CoM.Inverse() * _r_foot.lower_left;

    std::cout<<"r_foot upper_right:"<<std::endl;cartesian_utils::printKDLFrame(r_foot_CoM.upper_right);
    std::cout<<"r_foot upper_left:"<<std::endl;cartesian_utils::printKDLFrame(r_foot_CoM.upper_left);
    std::cout<<"r_foot lower_right:"<<std::endl;cartesian_utils::printKDLFrame(r_foot_CoM.lower_right);
    std::cout<<"r_foot lower_left:"<<std::endl;cartesian_utils::printKDLFrame(r_foot_CoM.lower_left);

}


