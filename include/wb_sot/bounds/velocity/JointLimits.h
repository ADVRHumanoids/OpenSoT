/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
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

 #include <wb_sot/Bounds.h>

 #include <yarp/sig/all.h>
 #include <iCub/iDynTree/DynTree.h>

 namespace wb_sot {
    namespace bounds {
        template <unsigned int x_size>
        class JointLimits: public Bounds<yarp::sig::Matrix, yarp::sig::Vector, x_size> {
        private:
            iCub::iDynTree::DynTree _robot;
            yarp::sig::Vector _qLowerBounds;
            yarp::sig::Vector _qUpperBounds;
            yarp::sig::Vector _q;
            double _dT;
        public:
            /**
             * @brief JointLimits constructor
             * @param robot the robot model which includes joint limits
             * @param dT the time constant at which we are performing velocity control [s]
             */
            JointLimits(const iCub::iDynTree::DynTree& robot, const double dT);

            yarp::sig::Vector getLowerBound();
            yarp::sig::Vector getUpperBound();

            yarp::sig::Matrix getAeq();
            yarp::sig::Vector getbeq();

            yarp::sig::Matrix getAineq();
            yarp::sig::Vector getbLowerBound();
            yarp::sig::Vector getbUpperBound();

            void update(const yarp::sig::Vector &x);
        };
    }
 }
