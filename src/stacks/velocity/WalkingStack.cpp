#include <OpenSoT/stacks/velocity/WalkingStack.h>

using namespace OpenSoT;

WalkingStack::WalkingStack(iDynUtils& model,
                                     const double dT,
                                     const Eigen::VectorXd& state) :
  OpenSoT::AutoStack(state.size()),
  eps(2e12),
  _model(model)
 {
    const Eigen::VectorXd &q = state;

    model.updateiDyn3Model(cartesian_utils::fromEigentoYarp(q), true);

    model.setFloatingBaseLink(model.left_leg.end_effector_name);
    KDL::Frame from_anchor_to_world = from_anchor_to_world.Identity(); // zero because we want l_sole both as anchor and world

    model.updateiDyn3Model(cartesian_utils::fromEigentoYarp(q), true);

    stance_foot = LEFT_FOOT;

    DHS.reset( new DefaultHumanoidStack(model, dT, state) );

    /*                            */
    /*      CONFIGURING DHS       */
    /*                            */
    DHS->rightLeg->setLambda(1.0);   DHS->rightLeg->setOrientationErrorGain(1.0);
    DHS->leftLeg->setLambda(1.0);    DHS->leftLeg->setOrientationErrorGain(1.0);
    DHS->waist2RightArm->setLambda(1.0); DHS->waist2RightArm->setOrientationErrorGain(1.0);
    DHS->waist2LeftArm->setLambda(1.0);  DHS->waist2LeftArm->setOrientationErrorGain(1.0);
    DHS->com->setLambda(1.0);
    DHS->com_XY->setLambda(1.0);
    DHS->waist->setLambda(1.0);     DHS->waist->setOrientationErrorGain(1.0);
    DHS->postural->setLambda(0.05);
    // velocity limits
    DHS->velocityLimits->setVelocityLimits(M_PI_4);

    /*                            */
    /*       CREATING STACK       */
    /*                            */

    // defining the stack
    this->getStack() =
         (( DHS->rightLeg + DHS->leftLeg + DHS->com_XY) /
         /*(( DHS->rightLeg + DHS->leftLeg + DHS->com) /*/
         /*(( DHS->rightLeg + DHS->leftLeg + DHS->waist) /*/
         ( (DHS->waist2LeftArm + DHS->waist2RightArm) ) /
         ( (DHS->postural) ))->getStack();
    // imposing joint and velocity limits TBD if you want to lock a joint modify the joint limits setting the actual posion and putting the velocity limits to 0
    this->getBoundsList().push_back(DHS->jointLimits);
    this->getBoundsList().push_back(DHS->velocityLimits);
    // creating the aggregated bounds given the bounds list
    this->getBounds()->update(q);
    // updating the stack based on the actual state
    this->update(q);
 }

bool WalkingStack::switchSupportFoot(const int trj_stance_foot)
{
    if(trj_stance_foot != stance_foot){
        if(trj_stance_foot == LEFT_FOOT){
            stance_foot = LEFT_FOOT;
            return _model.switchAnchorAndFloatingBase("l_ankle");}
        else if(trj_stance_foot == RIGHT_FOOT){
            stance_foot = RIGHT_FOOT;
            return _model.switchAnchorAndFloatingBase("r_ankle");}
    }
    return false;
}

