#include <OpenSoT/utils/DefaultHumanoidStack.h>


using namespace OpenSoT;

DefaultHumanoidStack::DefaultHumanoidStack(iDynUtils& model,
                                           const double dT,
                                           const yarp::sig::Vector& state) :
     leftArm( new tasks::velocity::Cartesian("cartesian::l_wrist",
                                             state,
                                             model,
                                             model.left_arm.end_effector_name,
                                             "world") ),
     rightArm( new tasks::velocity::Cartesian("cartesian::r_wrist",
                                              state,
                                              model,
                                              model.right_arm.end_effector_name,
                                             "world") ),
     waist2LeftArm( new tasks::velocity::Cartesian("cartesian::l_wrist",
                                                   state,
                                                   model,\
                                                   model.left_arm.end_effector_name,
                                                   "Waist") ),
     waist2RightArm( new tasks::velocity::Cartesian("cartesian::r_wrist",
                                                    state,
                                                    model,
                                                    model.right_arm.end_effector_name,
                                                    "Waist") ),
     leftLeg( new tasks::velocity::Cartesian("cartesian::l_sole",
                                             state,
                                             model,
                                             model.left_leg.end_effector_name,
                                             "world") ),
     rightLeg( new tasks::velocity::Cartesian("cartesian::r_sole",
                                              state,
                                              model,
                                              model.right_leg.end_effector_name,
                                              "world") ),
     waist( new tasks::velocity::Cartesian("cartesian::waist",
                                            state,
                                            model,
                                            "Waist",
                                            "world") ),
     com( new tasks::velocity::CoM(state,
                                   model) ),
     postural( new tasks::velocity::Postural(state) ),
     comVelocity( new constraints::velocity::CoMVelocity(yarp::sig::Vector(3,.3),
                                                         dT,
                                                         state,
                                                         model) ),
     convexHull( new constraints::velocity::ConvexHull(state,
                                                       model) ),
     jointLimits( new constraints::velocity::JointLimits(state,
                                                         model.iDyn3_model.getJointBoundMax(),
                                                         model.iDyn3_model.getJointBoundMin()) ),
     velocityLimits( new constraints::velocity::VelocityLimits(0.3,
                                                               dT,
                                                               state.size()) )


 {

 }


