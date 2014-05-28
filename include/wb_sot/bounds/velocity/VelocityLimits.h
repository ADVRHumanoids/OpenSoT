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

 namespace wb_sot {
    namespace bounds {
        template <unsigned int x_size>
        class VelocityLimits: public Bounds<yarp::sig::Matrix, yarp::sig::Vector, x_size> {
        private:
            yarp::sig::Vector _qDotLimit;
            yarp::sig::Vector _qLowerBound;
            yarp::sig::Vector _qUpperBound;
            double _dT;
        public:
            /**
             * @brief VelocityLimits constructor
             * @param robot the robot model which includes joint limits
             * @param dT the time constant at which we are performing velocity control [s]
             */
            VelocityLimits(const double qDotLimit, const double dT);

            yarp::sig::Vector getLowerBound();
            yarp::sig::Vector getUpperBound();

            yarp::sig::Matrix getAeq();
            yarp::sig::Vector getbeq();

            /**
             * @brief getAineq
             * @return 0xn_size Aineq matrix
             */
            yarp::sig::Matrix getAineq();
            /**
             * @brief getbLowerBound
             * @return 0 sized b lower bound
             */
            yarp::sig::Vector getbLowerBound();
            yarp::sig::Vector getbUpperBound();
        };
    }
 }
