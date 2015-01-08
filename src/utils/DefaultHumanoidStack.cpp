#include <OpenSoT/utils/DefaultHumanoidStack.h>

// using namespace OpenSoT;

// Factory::DefaultHumanoidStack::Ptr Factory::getDefaultHumanoidStack(std::string srdfModelFileName,
//                                                                     const double dT) {
//     if(!defaultHumanoidStack)
//         defaultHumanoidStack = Factory::DefaultHumanoidStack::Ptr(new Factory::DefaultHumanoidStack(srdfModelFileName,
//                                                                                                     dT));

//     return defaultHumanoidStack;
// }




// Factory::DefaultHumanoidStack::DefaultHumanoidStack(std::string srdfModelFileName,
//                                                              const double dT) :
//     leftArm( new tasks::velocity::Cartesian("cartesian::l_wrist",
//                                             model.zeros,
//                                             model,
//                                             model.left_arm.end_effector_name,
//                                             "world") ),
//     rightArm( new tasks::velocity::Cartesian("cartesian::r_wrist",
//                                              model.zeros,
//                                              model,
//                                              model.right_arm.end_effector_name,
//                                             "world") ),
//     waist2LeftArm( new tasks::velocity::Cartesian("cartesian::l_wrist",
//                                                   model.zeros,
//                                                   model,\
//                                                   model.left_arm.end_effector_name,
//                                                   "Waist") ),
//     waist2RightArm( new tasks::velocity::Cartesian("cartesian::r_wrist",
//                                                    model.zeros,
//                                                    model,
//                                                    model.right_arm.end_effector_name,
//                                                    "Waist") ),
//     leftLeg( new tasks::velocity::Cartesian("cartesian::l_sole",
//                                             model.zeros,
//                                             model,
//                                             model.left_leg.end_effector_name,
//                                             "world") ),
//     rightLeg( new tasks::velocity::Cartesian("cartesian::r_sole",
//                                              model.zeros,
//                                              model,
//                                              model.right_leg.end_effector_name,
//                                              "world") ),
//     rightLeg2LeftLeg( new tasks::velocity::Cartesian("cartesian::l_sole::r_sole",
//                                                      model.zeros,
//                                                      model,
//                                                      model.left_leg.end_effector_name,
//                                                      model.right_leg.end_effector_name) ),
//     leftLeg2RightLeg( new tasks::velocity::Cartesian("cartesian::r_sole::l_sole",
//                                                      model.zeros,
//                                                      model,
//                                                      model.right_leg.end_effector_name,
//                                                      model.left_leg.end_effector_name) ),
//     com( new tasks::velocity::CoM(model_com.zeros,
//                                   model_com) ),
//     postural( new tasks::velocity::Postural(model.zeros) ),
//     comVelocity( new constraints::velocity::CoMVelocity(yarp::sig::Vector(0.3,3),
//                                                         dT,
//                                                         model.zeros,
//                                                         model_com) ),
//     convexHull( new constraints::velocity::ConvexHull(model.zeros,
//                                                       model_com) ),
//     jointLimits( new constraints::velocity::JointLimits(model.zeros,
//                                                         model.iDyn3_model.getJointBoundMax(),
//                                                         model.iDyn3_model.getJointBoundMin()) ),
//     velocityLimits( new constraints::velocity::VelocityLimits(0.3,
//                                                               dT,
//                                                               model.zeros.size()) )


// {
//     model_com.iDyn3_model.setFloatingBaseLink(model_com.right_leg.index);
// }


// void OpenSoT::Factory::DefaultHumanoidStack::update(const yarp::sig::Vector& q)
// {
//     model.updateiDyn3Model(q, true);
//     model_com.updateiDyn3Model(q, true);
//     Stack<yarp::sig::Matrix, yarp::sig::Vector>::update(q);
// }

// template <class Matrix_type, class Vector_type>
// void OpenSoT::Factory::Stack<Matrix_type, Vector_type>::update(const Vector_type &q)
// {
//     for(auto task : stack) {
//         task->update(q);
//     }
// }

// template <class Matrix_type, class Vector_type>
// OpenSoT::Factory::Stack<Matrix_type, Vector_type>::operator typename OpenSoT::Solver<Matrix_type, Vector_type>::Stack()
// {

// }

// template <class Matrix_type, class Vector_type>
// OpenSoT::Factory::Stack<Matrix_type, Vector_type>::Stack(typename OpenSoT::Solver<Matrix_type, Vector_type>::Stack stack)
// {

// }
