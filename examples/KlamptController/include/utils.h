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

#ifndef __OPENSOT_KLAMPT_CONTROLLER_UTILS_H__
#define __OPENSOT_KLAMPT_CONTROLLER_UTILS_H__

#include <yarp/sig/all.h>
#include <KlamptController.h>

yarp::sig::Vector fromJntToiDyn(iDynUtils& model,
                                const KlamptController::JntPosition &posture);

KlamptController::JntPosition fromiDynToJnt(iDynUtils& model,
                                        const yarp::sig::Vector &q);

#endif
