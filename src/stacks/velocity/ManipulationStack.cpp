#include <OpenSoT/stacks/velocity/ManipulationStack.h>


using namespace OpenSoT;

ManipulationStack::ManipulationStack(iDynUtils& model,
                                     const double dT,
                                     const yarp::sig::Vector& state) :
  OpenSoT::AutoStack(state.size()),
  DefaultHumanoidStack(model, dT, state)
 {
    const yarp::sig::Vector &q = state;
    const yarp::sig::Vector dq = q*0.0;
    
    model.setFloatingBaseLink(model.left_leg.end_effector_name);
    KDL::Frame from_anchor_to_world = from_anchor_to_world.Identity(); // zero because we want l_sole both as anchor and world
    model.setAnchor_T_World(from_anchor_to_world);
    
    model.updateiDyn3Model(q, true);
    
    /*                            */
    /*      CONFIGURING DHS       */
    /*                            */
    /*this->rightLeg->setLambda(0.6);*/   this->rightLeg->setOrientationErrorGain(1.0);
    /*this->leftLeg->setLambda(0.6);*/    this->leftLeg->setOrientationErrorGain(1.0);
    this->rightArm->setLambda(0.6);       this->rightArm->setOrientationErrorGain(1.0);
    this->leftArm->setLambda(0.6);        this->leftArm->setOrientationErrorGain(1.0);
    this->com_XY->setLambda(0.05);      /*this->postural->setLambda(0.05);
    this->posturalForTorso->setLambda(0.05);     
    this->posturalForLimbdsAndHead->setLambda(0.05);*/
    // velocity limits
    this->velocityLimits->setVelocityLimits(0.1);
    
    /*                            */
    /*       CREATING STACK       */
    /*                            */

    // defining the stack 
    this->getStack() = 
        (( this->rightLeg ) /
         ( (this->com_XY + this->waist) << this->convexHull ) /
         ( (this->leftArm + this->rightArm + this->postural_Torso) ) /
         ( (this->postural_LimbsAndHead) ))->getStack();
    // imposing joint and velocity limits TBD if you want to lock a joint modify the joint limits setting the actual posion and putting the velocity limits to 0
    this->getBoundsList().push_back(this->jointLimits);
    this->getBoundsList().push_back(this->velocityLimits);
    // creating the aggregated bounds given the bounds list
    this->getBounds()->update(q);
    // updating the stack based on the actual state
    this->update(q);
 }


