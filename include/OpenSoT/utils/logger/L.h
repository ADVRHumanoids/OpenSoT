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
#include <OpenSoT/utils/logger/flushers/FakeFlusher.h>
#include <OpenSoT/utils/logger/flushers/RobotFlusher.h>
#include <OpenSoT/utils/logger/flushers/TaskFlusher.h>
#include <OpenSoT/utils/logger/plotters/Plotter.h>
#include <idynutils/idynutils.h>

#include <map>
#include <fstream>
#include <string>

namespace OpenSoT {
    class L
    {
    public:
        typedef boost::shared_ptr<L> Ptr;

        /**
         * @brief The logger_format enum defines in which format we are saving data
         */
        enum logger_format {FORMAT_PYTHON};

        /**
         * @brief L creates a named logger. Logged data will be included each in a separate file.
         * All files will then be included in a file named as the logger name, with an extension
         * that depends on the logger format (e.g., loggerName.py or loggerName.m)
         * @param loggerName the name of the current logger
         * @param model the robot model used.
         * @TODO we could inherite a LSolution that automatically saves the solution of the problem
         */
        L(std::string loggerName, iDynUtils& model, logger_format format = FORMAT_PYTHON);

        ~L();

        /**
         * @brief update the logger. It will automatically flush to file all flushers, save the current time and the optimal solution.
         * Should be called after solving a stack, and before the next stack update. For it to work, at least a flusher needs to be created
         * via the add() function.
         * @param t the current time
         * @param dq_opt the optimal solution as given by solve()
         */
        void update(double t, const yarp::sig::Vector& dq_opt);

        /**
         * @brief update the logger. It will automatically flush to file all flushers, and save the current time.
         * For it to work, at least a flusher needs to be created via the add() function.
         * @param t the current time
         */
        void update(double t);

        /**
         * @brief open opens a file for logging.
         * @param logName the name of the file without extension.
         * The extension that the logger will add depends on its format
         * (e.g. logName.py or logName.m)
         * @return
         */
        bool open(std::string logName);

        bool close();

        /**
         * @brief add adds a new task flusher. It will be deleted after the current log file is closed.
         * @param task a pointer to a task
         * @return the corresponding flusher
         */
        flushers::TaskFlusher::Ptr add(      Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task);

        /**
         * @brief add adds a new constraint flusher. It will be deleted after the current log file is closed
         * @param constraint
         * @return
         */
        flushers::ConstraintFlusher::Ptr add(Constraint<yarp::sig::Matrix, yarp::sig::Vector>::ConstraintPtr constraint);

        /**
         * @brief add adds a new data flusher. It will be deleted after the current log file is closed
         * @param data a pointer to the beginning of the the data to flush
         * @return
         */
        template <class T>
        flushers::Flusher::Ptr add(const T* data, const unsigned int size)
        {
            typename flushers::DataFlusher<T>::Ptr dataFlusher(new flushers::DataFlusher<T>(data, size));
            _dataFlushers[(void*)data] = dataFlusher;
            _flushers.push_back(dataFlusher);
            return dataFlusher;
        }

        /**
         * @brief add adds a new robot flusher. It will be deleted after the current log file is closed
         * @param robot the RobotUtils object referring to the robot whose state we want to log
         * @return
         */
        flushers::RobotFlusher::Ptr add(RobotUtils& robot);

        flushers::TaskFlusher::Ptr getFlusher(      Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task);

        flushers::ConstraintFlusher::Ptr getFlusher(Constraint<yarp::sig::Matrix, yarp::sig::Vector>::ConstraintPtr constraint);

        flushers::Flusher::Ptr getFlusher(void* data);

        flushers::RobotFlusher::Ptr getFlusher(RobotUtils& robot);

        logger_format getFormat() const;

        std::string getName() const;

        /**
         * @brief getGlobalIndices transform a plottable with local indices
         * to a plottable with global indices
         * @param plottable
         * @return
         */
        Indices getGlobalIndices(plotters::Plottable plottable);

        /**
         * @brief getLastIndex gets the size of the data we are flushing
         * @return the first free global index
         */
        unsigned int getDataSize();

        /**
         * @brief plotter a utility class to generate plots
         */
        OpenSoT::plotters::Plotter::Ptr plotter;

        const iDynUtils& model;

        /**
         * @brief isAppending returns a number greater than zero if the logger
         * is currently writing to a log file in append mode
         * @return the number of "open/close" operations already executed on this file
         */
        unsigned int isAppending();

        /**
         * @brief t returns a time plottable
         * @return a plottable with global index that refers to time
         */
        OpenSoT::plotters::Plottable t();

        /**
         * @brief dq_opt returns a solution vector plottable
         * @return a plottable with global index that refers to the stored solution
         */
        OpenSoT::plotters::Plottable dq_opt();

    protected:
        typedef Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr TaskPtr;
        typedef Constraint<yarp::sig::Matrix, yarp::sig::Vector>::ConstraintPtr ConstraintPtr;

        /**
         * @brief _name the name of the current logger
         */
        std::string _name;

        logger_format _format;

        /**
         * @brief _collator_log contains a list of all log files which this logger has written to
         */
        std::ofstream _collator;

        /**
         * @brief _current_log the file on which we are currently logging
         */
        std::ofstream _current_log;
        std::string   _current_log_filename;


        /**
         * @brief _n_dofs the size of the solution.
         */
        int _n_dofs;

        OpenSoT::flushers::FakeFlusher _fakeFlusher_t;
        OpenSoT::flushers::FakeFlusher _fakeFlusher_dq;

        std::map<std::string, unsigned int> _must_append;

        std::vector<flushers::Flusher::Ptr> _flushers;
        std::map<TaskPtr, flushers::TaskFlusher::Ptr> _taskFlushers;
        std::map<ConstraintPtr, flushers::ConstraintFlusher::Ptr> _constraintFlushers;
        std::map<void*, flushers::Flusher::Ptr> _dataFlushers;
        std::map<RobotUtils*, flushers::RobotFlusher::Ptr> _robotFlushers;
    };
}

#endif
