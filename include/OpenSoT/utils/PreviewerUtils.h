
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

#ifndef __PREVIEWER_UTILS_H__
#define __PREVIEWER_UTILS_H__

#include <boost/functional/hash.hpp>
#include <OpenSoT/utils/Previewer.h>
#include <yarp/math/Math.h>

namespace OpenSoT {
    namespace PreviewerUtils
    {
        /**
         * @brief The PyRunner class instantiates a Python Interpreter
         */
        class PyRunner {
        private:
            static boost::shared_ptr<PyRunner> instance;
            PyRunner();
        public:
            ~PyRunner();
            static boost::shared_ptr<PyRunner> getInstance();

            /**
             * @brief runs a script
             * @param script the script contents
             * @return true on success
             */
            bool run(std::string script);

            /**
             * @brief numpyRun runs a script, importing default numpy+maptplotlib includes
             * @param script the script contents
             * @return true on success
             */
            bool numpyRun(std::string script);
        };

//        /**
//         * sends Previewer trajectory results to rviz via robot status message (aka moveit)
//         */
//        template <class T>
//        void sendTrajectoryToRviz(OpenSoT::Previewer<T>::Results& results);

        /**
         *  plots q and dq from the Previewer trajectory results
         */
        template <class PreviewerResults>
        bool plotPreviewerTrajectory(PreviewerResults& results) {
            typedef typename PreviewerResults::TrajectoryLogEntry TrajectoryLogEntry;

            std::ostringstream _src;
            _src << "t_q = np.array((";
            for(typename std::vector<TrajectoryLogEntry>::iterator it =
                    results.trajectory.begin();
                it != results.trajectory.end();
                ++it)
            {
                std::string qString(it->q.toString());
                std::replace(qString.begin(), qString.end(), '\t', ',');
                _src << "(" << it->t << "," << qString << ")";
                typename std::vector<TrajectoryLogEntry>::iterator it_next = it; ++it_next;
                if(it_next != results.trajectory.end())
                    _src << "," << std::endl;
            }
            _src << "))" << std::endl;

            _src << "t_dq = np.array((";
            for(typename std::vector<TrajectoryLogEntry>::iterator it =
                    results.trajectory.begin();
                it != results.trajectory.end();
                ++it)
            {
                typename std::vector<TrajectoryLogEntry>::iterator it_next = it; ++it_next;
                if(it_next != results.trajectory.end())
                {
                    std::string dqString((it_next->q - it->q).toString());
                    std::replace(dqString.begin(), dqString.end(), '\t', ',');
                    _src << "(" << it_next->t << "," << dqString << "),";
                }
            }
            _src << "))" << std::endl;

            _src << "q_plot = figure(figsize=(8,6)); p_q = plot(t_q[:,0],t_q[:,1:]); title('Joint Positions')" << std::endl;
            _src << "dq_plot = figure(figsize=(8,6)); p_dq = plot(t_dq[:,0],t_dq[:,1:]); title('Joint Velocities')" << std::endl;
            _src << "show(block=True)" << std::endl;

            return PyRunner::getInstance()->numpyRun(_src.str());
        }

        /**
         *  plots all cartesian errors from the Previewer
         */
        template <class PreviewerResults>
        bool plotPreviewerErrors(PreviewerResults& results)
        {
            typedef typename PreviewerResults::ErrorsLogEntry ErrorsLogEntry;
            typedef typename ErrorsLogEntry::CartesianError CartesianError;
            typedef OpenSoT::Task<yarp::sig::Matrix,yarp::sig::Vector> Task;
            typedef map< double, ErrorsLogEntry > ErrorsLog;
            typedef map< Task*, CartesianError > ErrorsMap;

            std::ostringstream _src;
            for(typename ErrorsLog::iterator it_time =
                    results.errorsMap.begin();
                it_time != results.errorsMap.end();
                ++it_time)
            {
                ErrorsLogEntry errorLogEntry = it_time->second;
                boost::hash<std::string> string_hash;
                for(typename ErrorsMap::iterator it_task =
                        errorLogEntry.errors.begin();
                    it_task != errorLogEntry.errors.end();
                    ++it_task)
                {
                    Task* task = it_task->first;
                    std::string taskName = task->getTaskID();
                    std::size_t taskHash = string_hash(taskName);

                    _src << "t_" << taskHash << "_err = np.array((";

                    for(typename ErrorsLog::iterator it_time_inner =
                            results.errorsMap.begin();
                        it_time_inner != results.errorsMap.end();
                        ++it_time_inner)
                    {
                        double time = it_time_inner->first;
                        CartesianError error = it_time_inner->second.errors[task];

                        std::string errString(yarp::math::cat(error.positionError, error.orientationError).toString());
                        std::replace(errString.begin(), errString.end(), '\t', ',');
                        _src << "(" << time << "," << errString << ")";
                        typename ErrorsLog::iterator it_time_inner_next = it_time_inner; ++it_time_inner_next;
                        if(it_time_inner_next != results.errorsMap.end())
                            _src << "," << std::endl;
                    }
                    _src << "))" << std::endl;

                    _src << "figure(figsize=(8,6)); pos_err = plot(t_" << taskHash << "_err[:,0],t_" << taskHash << "_err[:,1:4]); title('Task " << taskName << " Position Errors'); legend(pos_err,('x', 'y', 'z'))" << std::endl;
                    if(dynamic_cast<OpenSoT::tasks::velocity::Cartesian*>(task))
                        _src << "figure(figsize=(8,6)); orient_err = plot(t_" << taskHash << "_err[:,0],t_" << taskHash << "_err[:,4:7]); title('Task " << taskName << " Orientation Errors'); legend(orient_err,('x', 'y', 'z'))" << std::endl;
                }
                break;  // just take first element, if it exists
            }

            _src << "show(block=True)" << std::endl;

            return PyRunner::getInstance()->numpyRun(_src.str());
        }

//        /**
//         *  plots all results from Previewer (trajectory, velocities, cartesian errors)
//         */
//        template <class TrajectoryGenerator>
//        void plotPreviewerResults(OpenSoT::Previewer<TrajectoryGenerator>::Results& results);
    }
}

#endif
