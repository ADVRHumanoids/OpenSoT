
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
        bool plotPreviewerTrajectory(OpenSoT::previewer::Results& results);

        /**
         *  plots all cartesian errors from the Previewer
         */
        bool plotPreviewerErrors(OpenSoT::previewer::Results& results);

//        /**
//         *  plots all results from Previewer (trajectory, velocities, cartesian errors)
//         */
//        template <class TrajectoryGenerator>
//        void plotPreviewerResults(OpenSoT::Previewer<TrajectoryGenerator>::Results& results);
    }
}

#endif
