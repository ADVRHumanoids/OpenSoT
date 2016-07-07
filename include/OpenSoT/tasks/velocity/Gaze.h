/*
 * Copyright (C) 2016 Walkman
 * Authors:Enrico Mingo Hoffman, Alessio Rocchi
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

#ifndef __TASKS_VELOCITY_GAZE_H__
#define __TASKS_VELOCITY_GAZE_H__

#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/SubTask.h>
#include <idynutils/cartesian_utils.h>

namespace OpenSoT {
namespace tasks {
namespace velocity {

class Gaze: public SubTask
{
public:
    typedef boost::shared_ptr<Gaze> Ptr;

    Gaze(std::string task_id, const yarp::sig::Vector &x, iDynUtils &robot, std::string base_link);
    ~Gaze();

    /**
     * @brief setGaze
     * @param desiredGaze pose of the object to observe in base_link
     */
    void setGaze(const yarp::sig::Matrix& desiredGaze);


    yarp::sig::Matrix getGaze();

    void setOrientationErrorGain(const double& orientationErrorGain);


private:
    std::string _distal_link;
    Cartesian::Ptr _cartesian_task;
    yarp::sig::Matrix _reference_gaze;

    iDynUtils& _robot;


};

}

}

}

#endif
