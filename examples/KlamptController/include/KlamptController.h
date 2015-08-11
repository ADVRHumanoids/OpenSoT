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

#ifndef __OPENSOT_KLAMPT_CONTROLLER_H__
#define __OPENSOT_KLAMPT_CONTROLLER_H__


#include <idynutils/idynutils.h>
#include <OpenSoT/utils/AutoStack.h>
#include <OpenSoT/Solver.h>
#include <map>
#include <string>

class KlamptController
{
protected:
    iDynUtils model;
    OpenSoT::AutoStack::Ptr stack;
    OpenSoT::Solver<yarp::sig::Matrix, yarp::sig::Vector>::SolverPtr solver;

public:
    typedef std::map<std::string, double> JntPosition;
    typedef JntPosition JntCommand;

    /**
     * @brief KlamptController loads the idynutils model
     * @param urdf_path the path where the robot urdf resides.
     * It is assumed the URDF file is in the same directory
     * where the robot SRDF file resides, and that the robot
     * name can be safely deducted from the URDF path.
     *
     * e.g.
     * ls /path/to/robot_urdf/
     *
     * /path/to/robot_urdf/robot_name.urdf
     * /path/to/robot_urdf/robot_name.srdf
     */
    KlamptController(std::string urdf_path);
    ~KlamptController();

    /**
     * @brief getPosture returns the robot posture
     * @return robot posture
     */
    JntPosition getPosture();

    /**
     * @brief setPosture updates the internal model with the
     * specified configuration vector and updates the kinematic
     * information of the robot
     * @param posture the desired joint posture
     */
    void setPosture(const JntPosition& posture);

    /**
     * @brief computeControl
     * @param q
     * @return joint commands for the robot
     */
    virtual JntCommand computeControl(JntPosition q) = 0;
};

#endif
