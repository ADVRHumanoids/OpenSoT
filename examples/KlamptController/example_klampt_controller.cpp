#include <yarp/os/Network.h>    // to change settings and send references to tasks via YARP

#include <example_klampt_controller.h>
#include <utils.h>


void ExampleKlamptController::init()
{
    yarp::sig::Vector q = model.iDyn3_model.getAng();   // [rad]

    // we assume the floating base is on the left leg end effector
    model.setFloatingBaseLink(model.left_leg.end_effector_name);

    // we assume we start with both feet in contact
    std::list<std::string> linksInContact;
    linksInContact.push_back(model.left_leg.end_effector_name);
    linksInContact.push_back(model.right_leg.end_effector_name);
    model.setLinksInContact(linksInContact);

    // we assume a constant discrete time controller with timestep dT
    DHS.reset(new OpenSoT::DefaultHumanoidStack(model, dT, q));


    /*                            */
    /*       CREATING STACK       */
    /*                            */

    // defining a stack composed of size two,
    // where the task of first priority is an aggregated of leftLeg and rightLeg,
    // task of priority two is leftArm and rightArm,
    // and the stack is subject to bounds jointLimits and velocityLimits
    stack =
        ( DHS->rightLeg + DHS->leftLeg ) /
        ( DHS->com_XY ) /
        ( DHS->waist_Position_Z ) /
        ( DHS->leftArm + DHS->rightArm ) /
        ( DHS->postural );
    stack << DHS->jointLimits; // << DHS->velocityLimits; commented since we are using VelocityAllocation


    /*                            */
    /*      CONFIGURING STACK     */
    /*                            */

    DHS->com->setLambda(0.6);        // com is a minimum velocity task
    DHS->rightLeg->setLambda(0.6);   DHS->rightLeg->setOrientationErrorGain(1.0);
    DHS->leftLeg->setLambda(0.6);    DHS->leftLeg->setOrientationErrorGain(1.0);
    DHS->rightArm->setLambda(0.3);   DHS->rightArm->setOrientationErrorGain(0.3);
    DHS->leftArm->setLambda(0.3);    DHS->leftArm->setOrientationErrorGain(0.3);
    DHS->waist->setLambda(0.6);
    DHS->jointLimits->setBoundScaling(0.3);
    DHS->velocityLimits->setVelocityLimits(0.3);

    // configuring joint mask for CoM task
    std::vector<bool> jointMask(model.getJointNames().size(),false);
    bodyJoints.insert(bodyJoints.end(),
                        model.left_arm.joint_numbers.begin(),
                        model.left_arm.joint_numbers.end());
    bodyJoints.insert(bodyJoints.end(),
                        model.right_arm.joint_numbers.begin(),
                        model.right_arm.joint_numbers.end());
    bodyJoints.insert(bodyJoints.end(),
                        model.left_leg.joint_numbers.begin(),
                        model.left_leg.joint_numbers.end());
    bodyJoints.insert(bodyJoints.end(),
                        model.right_leg.joint_numbers.begin(),
                        model.right_leg.joint_numbers.end());
    bodyJoints.insert(bodyJoints.end(),
                        model.torso.joint_numbers.begin(),
                        model.torso.joint_numbers.end());
    for(    std::list<int>::iterator j_it = bodyJoints.begin();
            j_it != bodyJoints.end();
            ++j_it)
    {
        jointMask[*j_it]=true;
        std::cout << "Enabling joint " << model.getJointNames()[*j_it]<< " in CoM activeJointMask" << std::endl;
    }

    DHS->com->setActiveJointsMask(jointMask);

    yarp::sig::Matrix pW = DHS->postural->getWeight();
    for(unsigned int i_t = 0; i_t < 3; ++i_t)
        pW(model.torso.joint_numbers[i_t],
            model.torso.joint_numbers[i_t]) *= 1e3;
    for(unsigned int i_t = 0; i_t < 6; ++i_t)
    {
        double amt = 7.5e1;
        if(i_t == 3 || i_t == 4)
            amt = 3;
        pW(model.left_leg.joint_numbers[i_t],
           model.left_leg.joint_numbers[i_t]) *= amt;
        pW(model.right_leg.joint_numbers[i_t],
           model.right_leg.joint_numbers[i_t]) *= amt;
    }
    DHS->postural->setWeight(pW);

    OpenSoT::VelocityAllocation(stack,
                                dT,
                                0.3,
                                0.6);

    // setting higher velocity limit to last stack --
    // TODO next feature of VelocityAllocation is a last_stack_speed ;)
    typedef std::list<OpenSoT::Constraint<yarp::sig::Matrix,yarp::sig::Vector>::ConstraintPtr>::iterator it_constraint;
    OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr lastTask = *(stack->getStack().rbegin());
    for(it_constraint i_c = lastTask->getConstraints().begin() ;
        i_c != lastTask->getConstraints().end() ; ++i_c) {
        if( boost::dynamic_pointer_cast<
                OpenSoT::constraints::velocity::VelocityLimits>(
                    *i_c))
            boost::dynamic_pointer_cast<
                            OpenSoT::constraints::velocity::VelocityLimits>(
                                *i_c)->setVelocityLimits(1.0);
    }

    /*                            */
    /*  CREATING TASK INTERFACES  */
    /*                            */


    {
        yarp::os::Network::init();
        using namespace OpenSoT::interfaces::yarp::tasks;

        leftArm.reset(new YCartesian(model.getRobotName(),
                                     MODULE_NAME, DHS->leftArm));
        std::cout << "left arm base - distal link:\n"
                  <<  DHS->leftArm->getBaseLink() << "-"
                  <<  DHS->leftArm->getDistalLink() << std::endl;

        rightArm.reset(new YCartesian(model.getRobotName(),
                                     MODULE_NAME, DHS->rightArm));
        std::cout << "right arm base - distal link:\n"
                  <<  DHS->rightArm->getBaseLink() << "-"
                  <<  DHS->rightArm->getDistalLink() << std::endl;

        waist.reset(new YCartesian(model.getRobotName(),
                                   MODULE_NAME, DHS->waist));
        std::cout << "waist base - distal link:\n"
                  <<  DHS->waist->getBaseLink() << "-"
                  <<  DHS->waist->getDistalLink() << std::endl;

        leftLeg.reset(new YCartesian(model.getRobotName(),
                                     MODULE_NAME, DHS->leftLeg));
        std::cout << "left leg base - distal link:\n"
                  <<  DHS->leftLeg->getBaseLink() << "-"
                  <<  DHS->leftLeg->getDistalLink() << std::endl;

        rightLeg.reset(new YCartesian(model.getRobotName(),
                                      MODULE_NAME, DHS->rightLeg));
        std::cout << "right leg base - distal link:\n"
                  <<  DHS->leftLeg->getBaseLink() << "-"
                  <<  DHS->leftLeg->getDistalLink() << std::endl;

        com.reset(new YCoM(model.getRobotName(),
                           MODULE_NAME, DHS->com));
        std::cout << "CoM base - distal link:\n"
                  <<  DHS->com->getBaseLink() << "-"
                  <<  DHS->com->getDistalLink() << std::endl;

        postural.reset(new YPostural(model.getRobotName(),
                                     MODULE_NAME, model, DHS->postural));
    }

    solver.reset(new OpenSoT::solvers::QPOases_sot(
                     stack->getStack(),
                     stack->getBounds(), 1e10));
}

bool ExampleKlamptController::debug_checkSolutionDoesntMoveFingers(const yarp::sig::Vector &dq)
{
    bool movesFingers = false;
    // checking solution for finger movement
    for(int j_n = 0; j_n < model.iDyn3_model.getNrOfDOFs(); ++j_n)
        if(std::find(bodyJoints.begin(),
                     bodyJoints.end(), j_n) == bodyJoints.end())
        {
            if(dq[j_n] > 0.0)
            {
                std::cout << "WARNING dq has element "
                          << model.getJointNames()[j_n]
                          << " = " << dq[j_n] << std::endl;
                movesFingers = true;
            }
        }
    return movesFingers;
}

bool ExampleKlamptController::debug_checkJacobiansDontContainFingerCols()
{
    bool jacobianContainFingers = true;

    // checking Jacobians for fingers
    std::vector<yarp::sig::Matrix> jacobians;
    jacobians.push_back(DHS->rightLeg->getA());
    jacobians.push_back(DHS->leftLeg->getA());
    jacobians.push_back(DHS->com_XY->getA());
    jacobians.push_back(DHS->waist_Position_Z->getA());
    jacobians.push_back(DHS->leftArm->getA());
    jacobians.push_back(DHS->rightArm->getA());

    for(std::vector<yarp::sig::Matrix>::iterator j_it = jacobians.begin();
        j_it != jacobians.end(); ++j_it)
        for(int j_n = 0; j_n < model.iDyn3_model.getNrOfDOFs(); ++j_n)
            if(std::find(bodyJoints.begin(),
                         bodyJoints.end(), j_n) == bodyJoints.end())
            {
                yarp::sig::Vector c = j_it->getCol(j_n);
                if(yarp::math::norm(c) > 0.0)
                {
                    std::cout << "WARNING J has column "
                              << c.toString()
                              << " != 0" << std::endl;
                    jacobianContainFingers = true;
                }
            }
    return jacobianContainFingers;
}

ExampleKlamptController::ExampleKlamptController(const KlamptController::JntPosition& posture)
    : KlamptController(std::string(OPENSOT_TESTS_ROBOTS_DIR)+"huboplus/huboplus.urdf"),
      time_accumulator(boost::accumulators::tag::rolling_mean::window_size = int(1.0/dT))
{
    print_mean = 0;

    yarp::sig::Vector q = fromJntToiDyn(model, posture);   // [rad]
    model.updateiDyn3Model(q,true);

    this->init();
}

ExampleKlamptController::ExampleKlamptController()
    : KlamptController(std::string(OPENSOT_TESTS_ROBOTS_DIR)+"huboplus/huboplus.urdf"),
      time_accumulator(boost::accumulators::tag::rolling_mean::window_size = int(1.0/dT))
{
    print_mean = 0;

    yarp::sig::Vector q = model.iDyn3_model.getAng();   // [rad]
    model.updateiDyn3Model(q,true);

    this->init();
}

ExampleKlamptController::~ExampleKlamptController()
{

}

KlamptController::JntCommand ExampleKlamptController::computeControl(KlamptController::JntPosition posture)
{
    yarp::sig::Vector dq;
    JntCommand command;
    double tic, toc;

    tic = yarp::os::Time::now();
    yarp::sig::Vector q = fromJntToiDyn(model, posture);
    model.updateiDyn3Model(q, true);

    /*yarp::sig::Matrix M(6+model.iDyn3_model.getNrOfDOFs(), 6+model.iDyn3_model.getNrOfDOFs());
    model.iDyn3_model.getFloatingBaseMassMatrix(M);
    M.removeCols(0,6); M.removeRows(0,6);
    DHS->postural->setWeight(M);*/ // this causes CROSS-COUPLING

    stack->update(q);

    debug_checkJacobiansDontContainFingerCols();

    if(solver->solve(dq))
    {
        command = fromiDynToJnt(model, dq);
        debug_checkSolutionDoesntMoveFingers(dq);
    }
    else
    {
        dq.zero();
        std::cout << "Error computing solve()" << std::endl;
    }
    toc = yarp::os::Time::now();
    time_accumulator(toc-tic);

    // print mean every s
    if(((++print_mean)%int(1.0/dT))==0) {
        print_mean = 0;
        std::cout << "dt = "
                  << boost::accumulators::extract::rolling_mean(time_accumulator) << std::endl;

        std::cout << "l_wrist reference:\n" << DHS->leftArm->getReference().toString() << std::endl;
        std::cout << "r_wrist reference:\n" << DHS->rightArm->getReference().toString() << std::endl;
        std::cout << "waist reference:\n"   << DHS->waist->getReference().toString() << std::endl;
        std::cout << "com reference:\n"     << DHS->com->getReference().toString() << std::endl;

    }
    return command;
}
