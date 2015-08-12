#include <example_klampt_controller.h>
#include <utils.h>


ExampleKlamptController::ExampleKlamptController()
    : KlamptController(std::string(OPENSOT_TESTS_ROBOTS_DIR)+"huboplus/huboplus.urdf"),
      time_accumulator(boost::accumulators::tag::rolling_mean::window_size = int(1.0/dT))
{
    yarp::sig::Vector q = model.iDyn3_model.getAng();   // [rad]
    model.updateiDyn3Model(q,true);

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
        ( DHS->leftLeg + DHS->rightLeg ) /
        ( DHS->leftArm + DHS->rightArm + DHS->waist_Orientation + DHS->com_XY ) /
        ( DHS->postural );
    stack << DHS->jointLimits; // << DHS->velocityLimits; commented since we are using VelocityAllocation


    /*                            */
    /*      CONFIGURING STACK     */
    /*                            */

    DHS->com->setLambda(0.0);        // com is a minimum velocity task
    DHS->rightLeg->setLambda(0.6);   DHS->rightLeg->setOrientationErrorGain(1.0);
    DHS->leftLeg->setLambda(0.6);    DHS->leftLeg->setOrientationErrorGain(1.0);
    DHS->rightArm->setLambda(0.1);   DHS->rightArm->setOrientationErrorGain(0.3);
    DHS->leftArm->setLambda(0.1);    DHS->leftArm->setOrientationErrorGain(0.3);
    DHS->jointLimits->setBoundScaling(0.3);
    DHS->velocityLimits->setVelocityLimits(0.3);

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

    pW = DHS->waist_Orientation->getWeight();
    pW(1, 1) *= 1e-2;
    DHS->waist_Orientation->setWeight(pW);

    OpenSoT::VelocityAllocation(stack,
                                dT,
                                0.3,
                                0.3);

    // setting higher velocity limit to last stack --
    // TODO next feature of VelocityAllocation is a last_stack_speed ;)
    typedef std::list<OpenSoT::Constraint<yarp::sig::Matrix,yarp::sig::Vector>::ConstraintPtr>::iterator it_constraint;
    OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr lastTask = stack->getStack()[2];
    for(it_constraint i_c = lastTask->getConstraints().begin() ;
        i_c != lastTask->getConstraints().end() ; ++i_c) {
        if( boost::dynamic_pointer_cast<
                OpenSoT::constraints::velocity::VelocityLimits>(
                    *i_c))
            boost::dynamic_pointer_cast<
                            OpenSoT::constraints::velocity::VelocityLimits>(
                                *i_c)->setVelocityLimits(.6);
    }

    /*                            */
    /*  CREATING TASK INTERFACES  */
    /*                            */


    {
        using namespace OpenSoT::interfaces::yarp::tasks;

        leftArm.reset(new YCartesian(model.getRobotName(),
                                     MODULE_NAME, DHS->leftArm));
        rightArm.reset(new YCartesian(model.getRobotName(),
                                     MODULE_NAME, DHS->rightArm));
        waist.reset(new YCartesian(model.getRobotName(),
                                   MODULE_NAME, DHS->waist));
        leftLeg.reset(new YCartesian(model.getRobotName(),
                                     MODULE_NAME, DHS->leftLeg));
        rightLeg.reset(new YCartesian(model.getRobotName(),
                                      MODULE_NAME, DHS->rightLeg));
        com.reset(new YCoM(model.getRobotName(),
                           MODULE_NAME, DHS->com));

        postural.reset(new YPostural(model.getRobotName(),
                                     MODULE_NAME, model, DHS->postural));
    }

    solver.reset(new OpenSoT::solvers::QPOases_sot(
                     stack->getStack(),
                     stack->getBounds(), 1e10));
}

ExampleKlamptController::~ExampleKlamptController()
{

}

KlamptController::JntCommand ExampleKlamptController::computeControl(KlamptController::JntPosition posture)
{
    yarp::sig::Vector dq;
    JntCommand command;
    double tic, toc;
    int print_mean = 0;

    tic = yarp::os::Time::now();
    yarp::sig::Vector q = fromJntToiDyn(model, posture);
    model.updateiDyn3Model(q, true);

    yarp::sig::Matrix M(6+model.iDyn3_model.getNrOfDOFs(), 6+model.iDyn3_model.getNrOfDOFs());
    model.iDyn3_model.getFloatingBaseMassMatrix(M);
    M.removeCols(0,6); M.removeRows(0,6);
    DHS->postural->setWeight(M);

    stack->update(q);
    if(solver->solve(dq))
            command = fromiDynToJnt(model, dq);
        else
            std::cout << "Error computing solve()" << std::endl;
        toc = yarp::os::Time::now();
        time_accumulator(toc-tic);

        // print mean every 5s
        if(((++print_mean)%int(5.0/dT))==0) {
            print_mean = 0;
            std::cout << "dt = "
                      << boost::accumulators::extract::rolling_mean(time_accumulator) << std::endl;

            std::cout << "l_wrist reference:" << DHS->leftArm->getReference().toString() << std::endl;
            std::cout << "r_wrist reference:" << DHS->rightArm->getReference().toString() << std::endl;
            std::cout << "waist reference:"   << DHS->waist->getReference().toString() << std::endl;

        }
        return command;
}
