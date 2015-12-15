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

#include <OpenSoT/Constraint.h>
#include <OpenSoT/Task.h>
#include <OpenSoT/utils/logger/flushers/all.h>
#include <OpenSoT/utils/logger/flushers/ConstraintFlusher.h>
#include <OpenSoT/utils/logger/flushers/TaskFlusher.h>
#include <map>
#include <fstream>
#include <string>

#ifndef __LOGGER_H__
#define __LOGGER_H__

namespace OpenSoT {
    class L
    {
        class Plotter
        {
        public:
            Plotter(){};
        };

    public:
        /**
         * @brief L creates a named logger
         * @param loggerName
         */
        L(std::string loggerName);

        ~L();

        void udpate(double t);

        bool open(std::string logName);

        bool close();

        flushers::TaskFlusher::Ptr add(      Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task);

        flushers::ConstraintFlusher::Ptr add(Constraint<yarp::sig::Matrix, yarp::sig::Vector>::ConstraintPtr constraint);

        flushers::TaskFlusher::Ptr getFlusher(      Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task);

        flushers::ConstraintFlusher::Ptr getFlusher(Constraint<yarp::sig::Matrix, yarp::sig::Vector>::ConstraintPtr constraint);

        Plotter plotter;

    private:
        typedef Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr TaskPtr;
        typedef Constraint<yarp::sig::Matrix, yarp::sig::Vector>::ConstraintPtr ConstraintPtr;

        std::ofstream _current_log;
        std::string   _current_log_filename;

        std::map<std::string, int> must_append;
        std::map<TaskPtr, flushers::TaskFlusher::Ptr> taskFlushers;
        std::map<ConstraintPtr, flushers::ConstraintFlusher::Ptr> constraintFlushers;
    };
}

#endif __LOGGER_H__
