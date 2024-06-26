#include "DefaultHumanoidStack.h"


using namespace OpenSoT;

DefaultHumanoidStack::DefaultHumanoidStack(XBot::ModelInterface& model,
                                           const double dT,
                                           std::string base_link,
                                           std::string l_hand, std::string r_hand,
                                           std::string l_foot, std::string r_foot,
                                           double joint_velocity_limits) :
     leftArm( new tasks::velocity::Cartesian("cartesian::l_wrist",
                                             model,
                                             l_hand,
                                             "world") ),
     leftArm_Position( new SubTask(leftArm, Indices::range(0,2)) ),
     leftArm_Orientation( new SubTask(leftArm, Indices::range(3,5)) ),
     rightArm( new tasks::velocity::Cartesian("cartesian::r_wrist",
                                              model,
                                              r_hand,
                                             "world") ),
     rightArm_Position( new SubTask(rightArm, Indices::range(0,2)) ),
     rightArm_Orientation( new SubTask(rightArm, Indices::range(3,5)) ),
     waist2LeftArm( new tasks::velocity::Cartesian("cartesian::l_wrist",
                                                   model,
                                                   l_hand,
                                                   base_link) ),
     waist2LeftArm_Position( new SubTask(waist2LeftArm, Indices::range(0,2)) ),
     waist2LeftArm_Orientation( new SubTask(waist2LeftArm, Indices::range(3,5)) ),
     waist2RightArm( new tasks::velocity::Cartesian("cartesian::r_wrist",
                                                    model,
                                                    r_hand,
                                                    base_link) ),
     waist2RightArm_Position( new SubTask(waist2RightArm, Indices::range(0,2)) ),
     waist2RightArm_Orientation( new SubTask(waist2RightArm, Indices::range(3,5)) ),
     leftLeg( new tasks::velocity::Cartesian("cartesian::l_sole",
                                             model,
                                             l_foot,
                                             "world") ),
     leftLeg_Position( new SubTask(leftLeg,    Indices::range(0,2)) ),
     leftLeg_Orientation( new SubTask(leftLeg, Indices::range(3,5)) ),
     rightLeg( new tasks::velocity::Cartesian("cartesian::r_sole",
                                              model,
                                              r_foot,
                                              "world") ),
     rightLeg_Position( new SubTask(rightLeg,    Indices::range(0,2)) ),
     rightLeg_Orientation( new SubTask(rightLeg, Indices::range(3,5)) ),
     waist( new tasks::velocity::Cartesian("cartesian::waist",
                                            model,
                                            base_link,
                                            "world") ),
     right2LeftLeg( new tasks::velocity::Cartesian("cartesian:r2l_sole",
                                                  model,
                                                  r_foot,
                                                  l_foot) ),
     waist_Position( new SubTask(waist,    Indices::range(0,2)) ),
     waist_Position_XY( new SubTask(waist, Indices::range(0,1)) ),
     waist_Position_Z( new SubTask(waist,  Indices::range(2,2)) ),
     waist_Orientation( new SubTask(waist, Indices::range(3,5)) ),
     com( new tasks::velocity::CoM(model) ),
     com_XY( new SubTask(com, Indices::range(0,1)) ),
     com_Z( new SubTask(com,  Indices::range(2,2)) ),
     gaze( new tasks::velocity::Gaze("cartesian::gaze",  model, "world") ),
     waist2gaze( new tasks::velocity::Gaze("cartesian::w2gaze",  model, base_link) ),
     postural( new tasks::velocity::Postural(model) ),
     velocityLimits( new constraints::velocity::VelocityLimits(model, joint_velocity_limits,
                                                               dT) ),
     comVelocity(new constraints::velocity::CartesianVelocity(Eigen::Vector3d::Constant(3,0.3)
                                                        ,dT,com))


 {
    Eigen::VectorXd qmin, qmax;
    model.getJointLimits(qmin, qmax);
    jointLimits.reset( new constraints::velocity::JointLimits(model,qmax,qmin));
 }


