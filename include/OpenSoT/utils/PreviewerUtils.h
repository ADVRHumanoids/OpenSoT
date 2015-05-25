
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

            map<std::string,bool> importedModules;

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

            /**
             * @brief setImported stores a flag in pyRunner specifying that a particular module
             *        has beel already imported in the currently running interpreter
             * @param module the module name
             */
            void setImported(const std::string module);

            /**
             * @brief imported checks whether a module has been already imported
             * @param module the module name
             * @return true if the module has been previously loaded on the interpreter
             */
            bool imported(const std::string module);
        };

        /**
         *  plots q and dq from the Previewer trajectory results
         */
        template <class PreviewerResults>
        bool plotPreviewerTrajectory(PreviewerResults& results) {
            typedef typename PreviewerResults::LogEntry LogEntry;

            if(results.log.size() > 0)
            {
                std::ostringstream _src;

                _src << "t_q = np.array((";
                for(typename std::map<double, LogEntry>::iterator it =
                        results.log.begin();
                    it != results.log.end();
                    ++it)
                {
                    std::string qString(it->second.q.toString());
                    std::replace(qString.begin(), qString.end(), '\t', ',');
                    _src << "(" << it->first << "," << qString << ")";
                    typename std::map<double, LogEntry>::iterator it_next = it; ++it_next;
                    if(it_next != results.log.end())
                        _src << "," << std::endl;
                }
                _src << "))" << std::endl;

                _src << "t_dq = np.array((";
                for(typename std::map<double, LogEntry>::iterator it =
                        results.log.begin();
                    it != results.log.end();
                    ++it)
                {
                    typename std::map<double, LogEntry>::iterator it_next = it; ++it_next;
                    if(it_next != results.log.end())
                    {
                        std::string dqString((it_next->second.q - it->second.q).toString());
                        std::replace(dqString.begin(), dqString.end(), '\t', ',');
                        _src << "(" << it_next->first << "," << dqString << "),";
                    }
                }
                _src << "))" << std::endl;

                _src << "q_plot = figure(figsize=(8,6)); p_q = plot(t_q[:,0],t_q[:,1:]); title('Joint Positions')" << std::endl;
                _src << "dq_plot = figure(figsize=(8,6)); p_dq = plot(t_dq[:,0],t_dq[:,1:]); title('Joint Velocities')" << std::endl;
                _src << "show(block=True)" << std::endl;

                return PyRunner::getInstance()->numpyRun(_src.str());
            } return false;
        }

        /**
         *  plots all cartesian errors from the Previewer
         */
        template <class PreviewerResults>
        bool plotPreviewerErrors(PreviewerResults& results)
        {
            typedef typename PreviewerResults::LogEntry LogEntry;
            typedef typename LogEntry::CartesianError CartesianError;
            typedef OpenSoT::Task<yarp::sig::Matrix,yarp::sig::Vector> Task;
            typedef map< Task*, CartesianError > ErrorsMap;

            if(results.log.size() > 0)
            {
                std::ostringstream _src;
                boost::hash<std::string> string_hash;

                for(typename ErrorsMap::iterator it_task =
                        results.log.begin()->second.errors.begin();
                    it_task != results.log.begin()->second.errors.end();
                    ++it_task)
                {
                    Task* task = it_task->first;
                    std::string taskName = task->getTaskID();
                    std::size_t taskHash = string_hash(taskName);

                    _src << "t_" << taskHash << "_err = np.array((";

                    for(typename std::map<double, LogEntry>::iterator it_time =
                            results.log.begin();
                        it_time != results.log.end();
                        ++it_time)
                    {
                        double time = it_time->first;
                        CartesianError error = it_time->second.errors[task];

                        std::string errString(yarp::math::cat(error.positionError, error.orientationError).toString());
                        std::replace(errString.begin(), errString.end(), '\t', ',');
                        _src << "(" << time << "," << errString << ")";
                        typename std::map<double, LogEntry>::iterator it_time_next = it_time; ++it_time_next;
                        if(it_time_next != results.log.end())
                            _src << "," << std::endl;
                    }
                    _src << "))" << std::endl;

                    _src << "figure(figsize=(8,6)); pos_err = plot(t_" << taskHash << "_err[:,0],t_" << taskHash << "_err[:,1:4]); title('Task " << taskName << " Position Errors'); legend(pos_err,('x', 'y', 'z'))" << std::endl;
                    if(dynamic_cast<OpenSoT::tasks::velocity::Cartesian*>(task))
                        _src << "figure(figsize=(8,6)); orient_err = plot(t_" << taskHash << "_err[:,0],t_" << taskHash << "_err[:,4:7]); title('Task " << taskName << " Orientation Errors'); legend(orient_err,('x', 'y', 'z'))" << std::endl;
                }

                _src << "show(block=True)" << std::endl;

                return PyRunner::getInstance()->numpyRun(_src.str());
            }
            return false;
        }

//        /**
//         *  plots all results from Previewer (trajectory, velocities, cartesian errors)
//         */
//        template <class TrajectoryGenerator>
//        void plotPreviewerResults(OpenSoT::Previewer<TrajectoryGenerator>::Results& results);
    }
}

#endif
