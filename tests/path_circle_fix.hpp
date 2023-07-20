/*
 * Copyright (C) 2016 Walkman
 * Author: Enrico Mingo Hoffman and Alessio Rocchi
 * email:  enrico.mingo@iit, alessio.rocchi@iit.it
 *
 * based on path_circle.hpp in orocos_kinematics_dynamics library
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef KDL_MOTION_PATHCIRCLE_FIX_H
#define KDL_MOTION_PATHCIRCLE_FIX_H

#include <kdl/path_circle.hpp>
#include <kdl/rotational_interpolation.hpp>


namespace KDL {


    /**
     * This is a FIX for the Clone() method of Path_Circle
     * A circular Path with 'open ends'. Path_Arc would
     * have been a better name though.
     * @ingroup Motion
     */
class Path_Circle_Fix : public Path_Circle
    {
    public:

        /**
         *
         * CAN THROW Error_MotionPlanning_Circle_ToSmall
         * CAN THROW Error_MotionPlanning_Circle_No_Plane
         */
        Path_Circle_Fix(const Frame& F_base_start,const Vector& V_base_center,
            const Vector& V_base_p,
            const Rotation& R_base_end,
            double alpha,
            RotationalInterpolation* otraj,
            double eqradius,
            bool _aggregate=true);


        virtual Path* Clone();



private:
        Frame _F_base_start;
        Vector _V_base_center;
        Vector _V_base_p;
        Rotation _R_base_end;
        double _alpha;
        RotationalInterpolation* _otraj;
        double _eqradius;
        bool __aggregate;
    };
}
#endif
