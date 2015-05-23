
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


#include <OpenSoT/utils/Previewer.h>

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

//        /**
//         *  plots all cartesian errors from the Previewer
//         */
//        template <class TrajectoryGenerator>
//        void plotPreviewerErrors(OpenSoT::Previewer<TrajectoryGenerator>::Results& results);

//        /**
//         *  plots all results from Previewer (trajectory, velocities, cartesian errors)
//         */
//        template <class TrajectoryGenerator>
//        void plotPreviewerResults(OpenSoT::Previewer<TrajectoryGenerator>::Results& results);
    }
}

#endif
