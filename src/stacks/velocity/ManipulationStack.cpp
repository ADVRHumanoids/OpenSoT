#include <OpenSoT/stacks/velocity/ManipulationStack.h>


using namespace OpenSoT;

ManipulationStack::ManipulationStack(iDynUtils& model,
                                     const double dT,
                                     const yarp::sig::Vector& state) :
  OpenSoT::AutoStack(state.size()),
  eps(2e12)
 {
    const yarp::sig::Vector &q = state;
    const yarp::sig::Vector dq = q*0.0;
    
    model.setFloatingBaseLink(model.left_leg.end_effector_name);
    KDL::Frame from_anchor_to_world = from_anchor_to_world.Identity(); // zero because we want l_sole both as anchor and world
    model.setAnchor_T_World(from_anchor_to_world);
    model.disableDynamicsUpdate();
    
    model.updateiDyn3Model(q, true);

    DHS.reset( new DefaultHumanoidStack(model, dT, state) );
    
    /*                            */
    /*      CONFIGURING DHS       */
    /*                            */
    /*DHS->rightLeg->setLambda(0.6);*/   DHS->rightLeg->setOrientationErrorGain(1.0);
    /*DHS->leftLeg->setLambda(0.6);*/    DHS->leftLeg->setOrientationErrorGain(1.0);
    DHS->rightArm->setLambda(1.2);       DHS->rightArm->setOrientationErrorGain(1.0);
    DHS->leftArm->setLambda(1.2);        DHS->leftArm->setOrientationErrorGain(1.0);
    DHS->com_XY->setLambda(0.05);        DHS->postural->setLambda(0.05);
    DHS->posturalForTorso->setLambda(0.1);
    DHS->posturalForLimbsAndHead->setLambda(0.05);
    // velocity limits
    DHS->velocityLimits->setVelocityLimits(M_PI_4);
    DHS->waist->setLambda(0.01);
    DHS->waist->setOrientationErrorGain(10.0);


    std::vector<bool> gaze_active_joint_mask = DHS->gaze->getActiveJointsMask();
    std::vector<bool> com_active_joint_mask = DHS->com->getActiveJointsMask();

    for(unsigned int i = 0; i < gaze_active_joint_mask.size(); ++i)
        gaze_active_joint_mask[i] = false;
    for(unsigned int i = 0; i < model.head.joint_numbers.size(); ++i)
    {
        gaze_active_joint_mask[model.head.joint_numbers[i]] = true;
        com_active_joint_mask[model.head.joint_numbers[i]] = false;
    }
    DHS->com->setActiveJointsMask(com_active_joint_mask);
    DHS->gaze->setActiveJointsMask(gaze_active_joint_mask);
    DHS->waist2gaze->setActiveJointsMask(gaze_active_joint_mask);
    DHS->gaze->setOrientationErrorGain(0.05);
    DHS->waist2gaze->setOrientationErrorGain(0.05);

    
    /*                            */
    /*       CREATING STACK       */
    /*                            */

    // defining the stack 
    this->getStack() = 
         (( DHS->postural_Torso + DHS->rightLeg ) /
         ( (DHS->com_XY + DHS->gaze) <<  DHS->convexHull ) /
         ( (DHS->leftArm + DHS->rightArm + DHS->waist) ) /
         ( (DHS->postural_LimbsAndHead) ))->getStack();
    // imposing joint and velocity limits TBD if you want to lock a joint modify the joint limits setting the actual posion and putting the velocity limits to 0
    this->getBoundsList().push_back(DHS->jointLimits);
    this->getBoundsList().push_back(DHS->velocityLimits);
    // creating the aggregated bounds given the bounds list
    this->getBounds()->update(q);
    // updating the stack based on the actual state
    this->update(q);
 }


