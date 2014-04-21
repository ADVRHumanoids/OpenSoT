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

#ifndef SOT_VELKINCON_CONSTANTS_H
#define SOT_VELKINCON_CONSTANTS_H

/** TODO: PUT ALL THIS DEFINES IN A CONFIG FILE **/

#define LEFT_ARM_IMPEDANCE true
#define RIGHT_ARM_IMPEDANCE true
#define TORSO_IMPEDANCE true

#define TORSO_WEIGHT 1.0
#define MAX_JOINT_VELOCITY toRad(20.0) //[rad/sec]
#define ORIENTATION_ERROR_GAIN 1.0
#define SET_3_TASKS false

#define DEBUG
#define MODULE_NAME "sot_VelKinCon"

/** ####################################### **/

#include <limits>
#include <paramHelp/paramProxyBasic.h>
#include <yarp/sig/Vector.h>

using namespace paramHelp;
using namespace std;

namespace wb_sot
{

// *** DEFAULT PARAMETER VALUES
// Streaming parameters
static const double                     SOT_DEFAULT_ELAPSED(0.0);
static const yarp::sig::Vector          SOT_DEFAULT_ERROR(3, 0.0);
static const int                        SOT_DEFAULT_QPOASES_NWSR(2^32);
static const bool                       SOT_DEFAULT_QPOASES_ENABLE_REGULARISATION(true);
static const double                     SOT_DEFAULT_QPOASES_EPS_REGULARISATION_MULTIPLIER(2e2);
static const double                     SOT_DEFAULT_DT(0.025);
static const bool                       SOT_DEFAULT_USE_3_STACKS(false);
static const double                     SOT_DEFAULT_MAX_JOINT_VELOCITY(20.0);
static const double                     SOT_DEFAULT_ORIENTATION_ERROR_GAIN(1.0);
static const int                        SOT_DEFAULT_LAST_STACK_TYPE(1);
static const int                        SOT_DEFAULT_POSTURAL_WEIGHT_STRATEGY(1);
static const double                     SOT_DEFAULT_POSTURAL_WEIGHT_COEFFICIENT(1.0);
static const double                     SOT_DEFAULT_MINEFFORT_WEIGHT_COEFFICIENT(1.0);
static const bool                       SOT_DEFAULT_MINEFFORT_WEIGHT_NORMALIZATION(true);
static const double                     SOT_DEFAULT_W_TORSO_WEIGHT(1.0);


// *** IDs of all the module streaming parameters
enum sot_VelKinCon_ParamId {
// ***************************************** MONITOR PARAMETERS *****************************************************************
    PARAM_ID_COMPUTATION_TIME,
    PARAM_ID_LEFT_ARM_POSITION_ERROR,    PARAM_ID_LEFT_ARM_ORIENTATION_ERROR,
    PARAM_ID_RIGHT_ARM_POSITION_ERROR,   PARAM_ID_RIGHT_ARM_ORIENTATION_ERROR,
    PARAM_ID_SWING_FOOT_POSITION_ERROR,  PARAM_ID_SWING_FOOT_ORIENTATION_ERROR,
    PARAM_ID_COM_POSITION_ERROR,
// ***************************************** RPC PARAMETERS *********************************************************************
    PARAM_ID_LEFT_ARM_IMPEDANCE_CONTROL, PARAM_ID_RIGHT_ARM_IMPEDANCE_CONTROL,
    PARAM_ID_TORSO_IMPEDANCE_CONTROL,
    PARAM_ID_USE_3_STACKS,
    PARAM_ID_MAX_JOINT_VELOCITY,
    PARAM_ID_ORIENTATION_ERROR_GAIN,
    PARAM_ID_LAST_STACK_TYPE,
    PARAM_ID_POSTURAL_WEIGHT_STRATEGY,PARAM_ID_POSTURAL_WEIGHT_COEFFICIENT,
    PARAM_ID_MINEFFORT_WEIGHT_COEFFICIENT, PARAM_ID_MINEFFORT_WEIGHT_NORMALIZATION,
    PARAM_ID_W_TORSO_WEIGHT,
    PARAM_ID_QPOASES_NWSR0, PARAM_ID_QPOASES_NWSR1, PARAM_ID_QPOASES_NWSR2,
    PARAM_ID_QPOASES_ENABLEREGULARISATION0, PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER0,
    PARAM_ID_QPOASES_ENABLEREGULARISATION1, PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER1,
    PARAM_ID_QPOASES_ENABLEREGULARISATION2, PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER2,
// ***************************************** CONFIGURATION PARAMETERS ***********************************************************
    PARAM_ID_DT,
    PARAM_ID_SIZE /*This is the number of parameters, so it must be the last value of the enum.*/
};


// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE PARAMETERS ******************************************
// ******************************************************************************************************************************
const ParamProxyInterface *const sot_VelKinCon_ParamDescr[PARAM_ID_SIZE] =
{
//                          NAME                                   ID                                           SIZE         CONSTRAINTS                            I/O ACCESS        DEFAULT VALUE                                          DESCRIPTION
// ************************************************* MONITOR PARAMETERS ****************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<double>("t_elapsed",                           PARAM_ID_COMPUTATION_TIME,                    1,        ParamConstraint<double>(),             PARAM_MONITOR,    &SOT_DEFAULT_ELAPSED,                                    "Time taken by each invocation of run()"),
new ParamProxyBasic<double>("eLWrist_p",                           PARAM_ID_LEFT_ARM_POSITION_ERROR,             3,        ParamConstraint<double>(),             PARAM_MONITOR,    SOT_DEFAULT_ERROR.data(),                                "Position error of left arm"),
new ParamProxyBasic<double>("eLWrist_o",                           PARAM_ID_LEFT_ARM_ORIENTATION_ERROR,          3,        ParamConstraint<double>(),             PARAM_MONITOR,    SOT_DEFAULT_ERROR.data(),                                "Orientation error of left arm"),
new ParamProxyBasic<double>("eRWrist_p",                           PARAM_ID_RIGHT_ARM_POSITION_ERROR,            3,        ParamConstraint<double>(),             PARAM_MONITOR,    SOT_DEFAULT_ERROR.data(),                                "Position error of right arm"),
new ParamProxyBasic<double>("eRWrist_o",                           PARAM_ID_RIGHT_ARM_ORIENTATION_ERROR,         3,        ParamConstraint<double>(),             PARAM_MONITOR,    SOT_DEFAULT_ERROR.data(),                                "Orientation error of right arm"),
new ParamProxyBasic<double>("eSwingFoot_p",                        PARAM_ID_SWING_FOOT_POSITION_ERROR,           3,        ParamConstraint<double>(),             PARAM_MONITOR,    SOT_DEFAULT_ERROR.data(),                                "Position error of swing foot"),
new ParamProxyBasic<double>("eSwingFoot_o",                        PARAM_ID_SWING_FOOT_ORIENTATION_ERROR,        3,        ParamConstraint<double>(),             PARAM_MONITOR,    SOT_DEFAULT_ERROR.data(),                                "Orientation error of swing foot"),
new ParamProxyBasic<double>("eCoM",                                PARAM_ID_COM_POSITION_ERROR,                  3,        ParamConstraint<double>(),             PARAM_MONITOR,    SOT_DEFAULT_ERROR.data(),                                "Position error of COM"),
// ************************************************* RPC PARAMETERS ****************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<bool>("left_arm_impedance_control",            PARAM_ID_LEFT_ARM_IMPEDANCE_CONTROL,          1,                                               PARAM_IN_OUT,     NULL,                                                    "use joint impedance control for left arm"),
new ParamProxyBasic<bool>("right_arm_impedance_control",           PARAM_ID_RIGHT_ARM_IMPEDANCE_CONTROL,         1,                                               PARAM_IN_OUT,     NULL,                                                    "use joint impedance control for right arm"),
new ParamProxyBasic<bool>("torso_impedance_control",               PARAM_ID_TORSO_IMPEDANCE_CONTROL,             1,                                               PARAM_IN_OUT,     NULL,                                                    "use joint impedance control for torso"),
new ParamProxyBasic<bool>("use_3_stacks",                          PARAM_ID_USE_3_STACKS,                        1,                                               PARAM_IN_OUT,     &SOT_DEFAULT_USE_3_STACKS,                               "use 3 stacks or two? If using three, first task is used for foot placement and precise CoM control"),
new ParamProxyBasic<double>("max_joint_velocity",                  PARAM_ID_MAX_JOINT_VELOCITY,                  1,        ParamBilatBounds<double>(0,60.0),      PARAM_IN_OUT,     &SOT_DEFAULT_MAX_JOINT_VELOCITY,                         "Maximum velocity [rad/sec] for the joints"),
new ParamProxyBasic<double>("orientation_error_gain",              PARAM_ID_ORIENTATION_ERROR_GAIN,              1,        ParamConstraint<double>(),             PARAM_IN_OUT,     &SOT_DEFAULT_ORIENTATION_ERROR_GAIN,                     "the orientation gain is used to weight orientation error over position eRWrist = yarp::math::cat(eRWrist_p,-ORIENTATION_ERROR_GAIN*eRWrist_o);"),
new ParamProxyBasic<int>("last_stack_type",                        PARAM_ID_LAST_STACK_TYPE,                     1,        ParamBilatBounds<int>(0,3),            PARAM_IN_OUT,     &SOT_DEFAULT_LAST_STACK_TYPE,                            "type of last stack. It is an enum, with values 0: postural (A=I, b=(q-q_ref)) 1: postural and gravity gradient (A=[I;grad_g(q)], b=[q-q_ref;0]) 2: (gravity torque) minimum effort (A=I, b=grad_g(q)^T) 3: postural and minimum effort (wrt gravity) (A=[I;I], b =[q-q_ref;grad_g(q)^T])"),
new ParamProxyBasic<int>("postural_weight_strategy",               PARAM_ID_POSTURAL_WEIGHT_STRATEGY,            1,        ParamBilatBounds<int>(0,3),            PARAM_IN_OUT,     &SOT_DEFAULT_POSTURAL_WEIGHT_STRATEGY,                   "postural stack weight strategy. It is an enum, with values 0: postural uses Identity weight matrix 1: postural uses weight which lets distal joints move further away. The weight is normalized w.r.t. joint limits 2: postural uses diag(grad_g(q)) 3: postural uses joint-space Inertia matrix"),
new ParamProxyBasic<double>("postural_weight_coefficient",         PARAM_ID_POSTURAL_WEIGHT_COEFFICIENT,         1,        ParamConstraint<double>(),             PARAM_IN_OUT,     &SOT_DEFAULT_POSTURAL_WEIGHT_COEFFICIENT,                "postural weight coefficient. It is a scalar multiplying the postural weight matrix. Especially important when using last_stack_type 1 and 3"),
new ParamProxyBasic<double>("mineffort_weight_coefficient",        PARAM_ID_MINEFFORT_WEIGHT_COEFFICIENT,        1,        ParamConstraint<double>(),             PARAM_IN_OUT,     &SOT_DEFAULT_MINEFFORT_WEIGHT_COEFFICIENT,               "mineffort weight coefficient. It is a scalar multiplying the mineffort weight matrix. Especially important when using last_stack_type 1 and 3"),
new ParamProxyBasic<bool>("mineffort_weight_normalization",        PARAM_ID_MINEFFORT_WEIGHT_NORMALIZATION,      1,        ParamConstraint<bool>(),               PARAM_IN_OUT,     &SOT_DEFAULT_MINEFFORT_WEIGHT_NORMALIZATION,             "mineffort weight normalization. If true, normalize the gradient with the torque limits"),
new ParamProxyBasic<double>("W_torso_weight",                      PARAM_ID_W_TORSO_WEIGHT,                      1,        ParamConstraint<double>(),             PARAM_IN_OUT,     &SOT_DEFAULT_W_TORSO_WEIGHT,                             "what weight the torso should have in Postural Weight Scheme 1"),
new ParamProxyBasic<int>("qpOases_nWSR0",                          PARAM_ID_QPOASES_NWSR0,                       1,        ParamLowerBound<int>(0),               PARAM_IN_OUT,     &SOT_DEFAULT_QPOASES_NWSR,                               "qpOases Maximum Number of Working Set recalculations for the first task. If too low, the QP can fail to converge."),
new ParamProxyBasic<int>("qpOases_nWSR1",                          PARAM_ID_QPOASES_NWSR1,                       1,        ParamLowerBound<int>(0),               PARAM_IN_OUT,     &SOT_DEFAULT_QPOASES_NWSR,                               "qpOases Maximum Number of Working Set recalculations for the second task. If too low, the QP can fail to converge."),
new ParamProxyBasic<int>("qpOases_nWSR2",                          PARAM_ID_QPOASES_NWSR2,                       1,        ParamLowerBound<int>(0),               PARAM_IN_OUT,     &SOT_DEFAULT_QPOASES_NWSR,                               "qpOases Maximum Number of Working Set recalculations for the third task. If too low, the QP can fail to converge."),
new ParamProxyBasic<bool>("qpOases_enableRegularisation0",         PARAM_ID_QPOASES_ENABLEREGULARISATION0,       1,                                               PARAM_IN_OUT,     &SOT_DEFAULT_QPOASES_ENABLE_REGULARISATION,              "qpOases Regularisation term for the first task. it multiplies the default which is epsRegularisation = 5.0e3*EPS"),
new ParamProxyBasic<bool>("qpOases_enableRegularisation1",         PARAM_ID_QPOASES_ENABLEREGULARISATION1,       1,                                               PARAM_IN_OUT,     &SOT_DEFAULT_QPOASES_ENABLE_REGULARISATION,              "qpOases Regularisation term for the second task. it multiplies the default which is epsRegularisation = 5.0e3*EPS"),
new ParamProxyBasic<bool>("qpOases_enableRegularisation2",         PARAM_ID_QPOASES_ENABLEREGULARISATION2,       1,                                               PARAM_IN_OUT,     &SOT_DEFAULT_QPOASES_ENABLE_REGULARISATION,              "qpOases Regularisation term for the third task. it multiplies the default which is epsRegularisation = 5.0e3*EPS"),
new ParamProxyBasic<double>("qpOases_epsRegularisationMultiplier0",PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER0,1,        ParamConstraint<double>(),             PARAM_IN_OUT,     &SOT_DEFAULT_QPOASES_EPS_REGULARISATION_MULTIPLIER,      "enable regularisation for the first task"),
new ParamProxyBasic<double>("qpOases_epsRegularisationMultiplier1",PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER1,1,        ParamConstraint<double>(),             PARAM_IN_OUT,     &SOT_DEFAULT_QPOASES_EPS_REGULARISATION_MULTIPLIER,      "enable regularisation for the second task"),
new ParamProxyBasic<double>("qpOases_epsRegularisationMultiplier2",PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER2,1,        ParamConstraint<double>(),             PARAM_IN_OUT,     &SOT_DEFAULT_QPOASES_EPS_REGULARISATION_MULTIPLIER,      "enable regularisation for the first task"),
// ************************************************* CONFIGURATION PARAMETERS ****************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<double>("dT",                                  PARAM_ID_DT,                                  1,        ParamLowerBound<double>(1e-3),         PARAM_CONFIG,     NULL,                                                    "frequency to use for the SoT (1/dT , dT is [s])")
};

}


#endif // SOT_VELKINCON_CONSTANTS_H
