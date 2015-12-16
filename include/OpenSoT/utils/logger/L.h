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

#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <OpenSoT/Constraint.h>
#include <OpenSoT/Task.h>
#include <OpenSoT/utils/logger/flushers/all.h>
#include <OpenSoT/utils/logger/flushers/ConstraintFlusher.h>
#include <OpenSoT/utils/logger/flushers/DataFlusher.h>
#include <OpenSoT/utils/logger/flushers/TaskFlusher.h>
#include <map>
#include <fstream>
#include <string>

namespace OpenSoT {
    class L
    {
        class Plottable
        {
            OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr _task;
            OpenSoT::Constraint<yarp::sig::Matrix, yarp::sig::Vector>::ConstraintPtr _constraint;
            OpenSoT::Indices _indices;
        public:
            /**
             * @brief Plottable plots all the elements of a task as specified by its flusher
             * @param task the task to plot
             */
            Plottable(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task)
                : _task(task), _indices(-1) {}

            /**
             * @brief Plottable plots all the elements of a constraint as specified by its flusher
             * @param constraint the constraint to plot
             */
            Plottable(OpenSoT::Constraint<yarp::sig::Matrix, yarp::sig::Vector>::ConstraintPtr constraint)
                : _constraint(constraint), _indices(-1) {}

            /**
             * @brief Plottable plots the specified elements of a task
             * @param task task to plot
             * @param indices indices to plot
             */
            Plottable(OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task,
                      OpenSoT::Indices& indices)
                : _task(task), _indices(indices) {}

            /**
             * @brief Plottable plots the specified elements of a constraint
             * @param constraint constraint to plot
             * @param indices indices to plot
             */
            Plottable(OpenSoT::Constraint<yarp::sig::Matrix, yarp::sig::Vector>::ConstraintPtr constraint,
                      OpenSoT::Indices& indices)
                : _constraint(constraint), _indices(indices) {}
        };

        class Plotter
        {
        protected:
            std::map<unsigned int, Plottable> toPlot;
        public:
            Plotter(){};

            bool setupSubPlots(unsigned int nRows, unsigned int nCols);

            Plotter& subPlot(unsigned int nSubPlot);

            bool plot(std::list<Plottable> data);

            bool plot(std::list<Plottable> data, std::list<std::string> labels);
        };

    public:
        /**
         * @brief L creates a named logger
         * @param loggerName
         */
        L(std::string loggerName);

        ~L();

        void udpate(double t, const yarp::sig::Vector& q_dot);

        bool open(std::string logName);

        bool close();

        flushers::TaskFlusher::Ptr add(      Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task);

        flushers::ConstraintFlusher::Ptr add(Constraint<yarp::sig::Matrix, yarp::sig::Vector>::ConstraintPtr constraint);

        template <class T>
        flushers::Flusher::Ptr add(const T* data)
        {
            dataFlushers[(void*)data].reset(new flushers::DataFlusher<T>(data));
            return dataFlushers[(void*)data];
        }

        flushers::TaskFlusher::Ptr getFlusher(      Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task);

        flushers::ConstraintFlusher::Ptr getFlusher(Constraint<yarp::sig::Matrix, yarp::sig::Vector>::ConstraintPtr constraint);

        flushers::Flusher::Ptr getFlusher(void* data);

        Plotter plotter;

    private:
        typedef Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr TaskPtr;
        typedef Constraint<yarp::sig::Matrix, yarp::sig::Vector>::ConstraintPtr ConstraintPtr;

        std::ofstream _current_log;
        std::string   _current_log_filename;

        std::map<std::string, int> must_append;
        std::map<TaskPtr, flushers::TaskFlusher::Ptr> taskFlushers;
        std::map<ConstraintPtr, flushers::ConstraintFlusher::Ptr> constraintFlushers;
        std::map<void*, flushers::Flusher::Ptr> dataFlushers;
    };
}

std::ostream& operator<<(std::ostream& out, const OpenSoT::flushers::Flusher& flusher)
{
    out << flusher.toString();
}

std::ostream& operator<<(std::ostream& out, const OpenSoT::flushers::Flusher::Ptr& flusher)
{
    if(flusher)
        out << flusher->toString();
}

#endif
