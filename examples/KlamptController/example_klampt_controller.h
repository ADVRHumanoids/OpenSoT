/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __OPENSOT_EXAMPLE_KLAMPT_CONTROLLER_H__
#define __OPENSOT_EXAMPLE_KLAMPT_CONTROLLER_H__

#include <KlamptController.h>
#include <OpenSoT/utils/DefaultHumanoidStack.h>
#include <OpenSoT/interfaces/yarp/tasks/YCartesian.h>
#include <OpenSoT/interfaces/yarp/tasks/YCoM.h>
#include <OpenSoT/interfaces/yarp/tasks/YPostural.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>


#define MODULE_NAME "huboplus_klampt_controller"
#define dT  1e-2    // [s]

typedef boost::accumulators::accumulator_set<double,
                                            boost::accumulators::stats<boost::accumulators::tag::rolling_mean>
                                            > Accumulator;

class ExampleKlamptController : public KlamptController
{
    /* LIST OF TASKS ACCESSIBLE VIA PYTHON YARP INTERFACES */
    OpenSoT::interfaces::yarp::tasks::YCartesian::Ptr leftArm;
    OpenSoT::interfaces::yarp::tasks::YCartesian::Ptr rightArm;
    OpenSoT::interfaces::yarp::tasks::YCartesian::Ptr waist;
    OpenSoT::interfaces::yarp::tasks::YCartesian::Ptr leftLeg;
    OpenSoT::interfaces::yarp::tasks::YCartesian::Ptr rightLeg;
    OpenSoT::interfaces::yarp::tasks::YCoM::Ptr com;
    OpenSoT::interfaces::yarp::tasks::YPostural::Ptr postural;

    /* keeping a pointer to the DHS */
    boost::shared_ptr<OpenSoT::DefaultHumanoidStack> DHS;

    /* to compute mean computing time over 1sec */
    Accumulator time_accumulator;

    /* counter for periodic print statements */
    int print_mean;

    std::list<int> bodyJoints;

    void init();

    bool debug_checkSolutionDoesntMoveFingers(const yarp::sig::Vector &dq);
    bool debug_checkJacobiansDontContainFingerCols();

public:
    ExampleKlamptController(const KlamptController::JntPosition& posture);

    ExampleKlamptController();

    ~ExampleKlamptController();

    /**
     * @brief computeControl updates the internal model with the
     * specified configuration vector, updates the stack accordingly, and solves
     * the IK problem to obtain a new joint command to be used.
     * @param posture the actual joint posture
     * @return the joint velocity command (dq) output by solving the IK problem
     */
    KlamptController::JntCommand computeControl(KlamptController::JntPosition posture);
};

#endif
