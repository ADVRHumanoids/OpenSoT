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

#define DEBUG
#define MODULE_NAME "sot_VelKinCon"
#define CONF_NAME "sot_velkincon.ini"

/** ####################################### **/

#include <limits>
#include <paramHelp/paramProxyBasic.h>
#include <yarp/sig/Vector.h>

using namespace paramHelp;
using namespace std;

namespace wb_sot
{

enum sot_VelKinCon_last_stack_type {
    // (A=I, b=(q-q_ref), q_ref <- initial_config.ini
    LAST_STACK_TYPE_POSTURAL,
    // (A=I, b=grad_g(q)^T)
    LAST_STACK_TYPE_MINIMUM_EFFORT,
    // (A=[I;I], b=[q-q_ref;grad_g(q)^T])
    LAST_STACK_TYPE_POSTURAL_AND_MINIMUM_EFFORT,

    LAST_STACK_TYPE_SIZE
};

enum sot_VelKinCon_postural_weight_strategy {
    // postural uses Identity weight matrix
    POSTURAL_WEIGHT_STRATEGY_IDENTITY,
    // diag(grad_g(q))
    POSTURAL_WEIGHT_STRATEGY_GRAD_G,
    // postural uses joint-space Inertia matrix
    POSTURAL_WEIGHT_STRATEGY_JOINT_SPACE_INERTIA,

    POSTURAL_WEIGHT_STRATEGY_SIZE
};

// *** DEFAULT PARAMETER VALUES
// Streaming parameters
static const double                     SOT_DEFAULT_ELAPSED(0.0);
static const yarp::sig::Vector          SOT_DEFAULT_ERROR(3, 0.0);
static const int                        SOT_DEFAULT_QPOASES_NWSR(2^32);
static const bool                       SOT_DEFAULT_QPOASES_ENABLE_REGULARISATION(true);
static const double                     SOT_DEFAULT_QPOASES_EPS_REGULARISATION_MULTIPLIER(2e2);
static const bool                       SOT_DEFAULT_CLIK(false);
static const bool                       SOT_DEFAULT_WORLD_UPDATE(true);
static const double                     SOT_DEFAULT_VELOCITY_BOUNDS_SCALE(0.5);
static const int                        SOT_DEFAULT_POSTURAL_WEIGHT_STRATEGY(0);

static const paramHelp::ParamBilatBounds<double> SOT_MAX_JOINT_VELOCITY_BOUNDS(ParamBilatBounds<double>(0,1.0));
static const paramHelp::ParamBilatBounds<int> SOT_LAST_STACK_TYPE_BOUNDS(ParamBilatBounds<int>(0,2));
static const paramHelp::ParamLowerBound<int> SOT_QPOASES_NWSR_BOUND(ParamLowerBound<int>(0));
static const paramHelp::ParamBilatBounds<double> SOT_POSTURAL_WEIGHT_COEFFICENT_BOUNDS(ParamBilatBounds<double>(0,1.0));
static const paramHelp::ParamBilatBounds<double> SOT_VELOCITY_BOUNDS_SCALE(ParamBilatBounds<double>(0.1,1.0));
static const paramHelp::ParamBilatBounds<int> SOT_POSTURAL_WEIGHT_STRATEGY_BOUNDS(ParamBilatBounds<int>(0,POSTURAL_WEIGHT_STRATEGY_SIZE));


// *** IDs of all the module streaming parameters
enum sot_VelKinCon_ParamId {
// ***************************************** MONITOR PARAMETERS *****************************************************************
    PARAM_ID_COMPUTATION_TIME,           PARAM_ID_GRAD_G_Q,
    PARAM_ID_LEFT_ARM_POSITION_ERROR,    PARAM_ID_LEFT_ARM_ORIENTATION_ERROR,
    PARAM_ID_RIGHT_ARM_POSITION_ERROR,   PARAM_ID_RIGHT_ARM_ORIENTATION_ERROR,
    PARAM_ID_SWING_FOOT_POSITION_ERROR,  PARAM_ID_SWING_FOOT_ORIENTATION_ERROR,
    PARAM_ID_COM_POSITION_ERROR,
// ***************************************** I/O PARAMETERS *********************************************************************
    PARAM_ID_LEFT_ARM_IMPEDANCE_CONTROL, PARAM_ID_RIGHT_ARM_IMPEDANCE_CONTROL,
    PARAM_ID_TORSO_IMPEDANCE_CONTROL,
    PARAM_ID_USE_3_STACKS,
    PARAM_ID_MAX_JOINT_VELOCITY,
    PARAM_ID_ORIENTATION_ERROR_GAIN,
    PARAM_ID_POSTURAL_WEIGHT_COEFFICIENT,
    PARAM_ID_MINEFFORT_WEIGHT_COEFFICIENT,
    PARAM_ID_POSTURAL_WEIGHT_STRATEGY,
    PARAM_ID_VELOCITY_BOUNDS_SCALE,
    PARAM_ID_QPOASES_NWSR0, PARAM_ID_QPOASES_NWSR1, PARAM_ID_QPOASES_NWSR2,
    PARAM_ID_QPOASES_ENABLEREGULARISATION0, PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER0,
    PARAM_ID_QPOASES_ENABLEREGULARISATION1, PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER1,
    PARAM_ID_QPOASES_ENABLEREGULARISATION2, PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER2,
    PARAM_ID_CLIK,
    PARAM_ID_WORLD_UPDATE,
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
new ParamProxyBasic<double>("t_elapsed",                           PARAM_ID_COMPUTATION_TIME,                    1,                                               PARAM_MONITOR,    &SOT_DEFAULT_ELAPSED,                                    "Time taken by each invocation of run()"),
new ParamProxyBasic<double>("gradientGq",                          PARAM_ID_GRAD_G_Q,                            29,                                              PARAM_MONITOR,    NULL,                                                    "Gradient of g(q)"),
new ParamProxyBasic<double>("eLWrist_p",                           PARAM_ID_LEFT_ARM_POSITION_ERROR,             3,                                               PARAM_MONITOR,    SOT_DEFAULT_ERROR.data(),                                "Position error of left arm"),
new ParamProxyBasic<double>("eLWrist_o",                           PARAM_ID_LEFT_ARM_ORIENTATION_ERROR,          3,                                               PARAM_MONITOR,    SOT_DEFAULT_ERROR.data(),                                "Orientation error of left arm"),
new ParamProxyBasic<double>("eRWrist_p",                           PARAM_ID_RIGHT_ARM_POSITION_ERROR,            3,                                               PARAM_MONITOR,    SOT_DEFAULT_ERROR.data(),                                "Position error of right arm"),
new ParamProxyBasic<double>("eRWrist_o",                           PARAM_ID_RIGHT_ARM_ORIENTATION_ERROR,         3,                                               PARAM_MONITOR,    SOT_DEFAULT_ERROR.data(),                                "Orientation error of right arm"),
new ParamProxyBasic<double>("eSwingFoot_p",                        PARAM_ID_SWING_FOOT_POSITION_ERROR,           3,                                               PARAM_MONITOR,    SOT_DEFAULT_ERROR.data(),                                "Position error of swing foot"),
new ParamProxyBasic<double>("eSwingFoot_o",                        PARAM_ID_SWING_FOOT_ORIENTATION_ERROR,        3,                                               PARAM_MONITOR,    SOT_DEFAULT_ERROR.data(),                                "Orientation error of swing foot"),
new ParamProxyBasic<double>("eCoM",                                PARAM_ID_COM_POSITION_ERROR,                  3,                                               PARAM_MONITOR,    SOT_DEFAULT_ERROR.data(),                                "Position error of COM"),
// ************************************************* RPC PARAMETERS ****************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<bool>("use_3_stacks",                          PARAM_ID_USE_3_STACKS,                        1,                                               PARAM_IN_OUT,     NULL,                                                    "use 3 stacks or two? If using three, first task is used for foot placement and precise CoM control"),
new ParamProxyBasic<double>("max_joint_velocity",                  PARAM_ID_MAX_JOINT_VELOCITY,                  1,        &SOT_MAX_JOINT_VELOCITY_BOUNDS,        PARAM_IN_OUT,     NULL,                                                    "Maximum velocity [rad/sec] for the joints"),
new ParamProxyBasic<double>("orientation_error_gain",              PARAM_ID_ORIENTATION_ERROR_GAIN,              1,                                               PARAM_IN_OUT,     NULL,                                                    "the orientation gain is used to weight orientation error over position eRWrist = yarp::math::cat(eRWrist_p,-ORIENTATION_ERROR_GAIN*eRWrist_o);"),
new ParamProxyBasic<double>("postural_weight_coefficient",         PARAM_ID_POSTURAL_WEIGHT_COEFFICIENT,         1,        &SOT_POSTURAL_WEIGHT_COEFFICENT_BOUNDS,PARAM_IN_OUT,     NULL,                                                    "postural weight coefficient. It is a scalar multiplying the postural weight gradient. Especially important when using last_stack_type 2."),
new ParamProxyBasic<double>("mineffort_weight_coefficient",        PARAM_ID_MINEFFORT_WEIGHT_COEFFICIENT,        1,                                               PARAM_IN_OUT,     NULL,                                                    "mineffort weight coefficient. It is a scalar multiplying the mineffort weight gradient."),
new ParamProxyBasic<int>("postural_weight_strategy",               PARAM_ID_POSTURAL_WEIGHT_STRATEGY,            1,        &SOT_POSTURAL_WEIGHT_STRATEGY_BOUNDS,  PARAM_IN_OUT,     &SOT_DEFAULT_POSTURAL_WEIGHT_STRATEGY,                   "weight used for joints postural last task."),
new ParamProxyBasic<double>("velocity_bounds_scale",               PARAM_ID_VELOCITY_BOUNDS_SCALE,               1,        &SOT_VELOCITY_BOUNDS_SCALE,            PARAM_IN_OUT,     &SOT_DEFAULT_VELOCITY_BOUNDS_SCALE,                      "scale of joint velocities used in Cartesian tasks"),
new ParamProxyBasic<int>("qpOases_nWSR0",                          PARAM_ID_QPOASES_NWSR0,                       1,        &SOT_QPOASES_NWSR_BOUND,               PARAM_IN_OUT,     &SOT_DEFAULT_QPOASES_NWSR,                               "qpOases Maximum Number of Working Set recalculations for the first task. If too low, the QP can fail to converge."),
new ParamProxyBasic<int>("qpOases_nWSR1",                          PARAM_ID_QPOASES_NWSR1,                       1,        &SOT_QPOASES_NWSR_BOUND,               PARAM_IN_OUT,     &SOT_DEFAULT_QPOASES_NWSR,                               "qpOases Maximum Number of Working Set recalculations for the second task. If too low, the QP can fail to converge."),
new ParamProxyBasic<int>("qpOases_nWSR2",                          PARAM_ID_QPOASES_NWSR2,                       1,        &SOT_QPOASES_NWSR_BOUND,               PARAM_IN_OUT,     &SOT_DEFAULT_QPOASES_NWSR,                               "qpOases Maximum Number of Working Set recalculations for the third task. If too low, the QP can fail to converge."),
new ParamProxyBasic<bool>("qpOases_enableRegularisation0",         PARAM_ID_QPOASES_ENABLEREGULARISATION0,       1,                                               PARAM_IN_OUT,     &SOT_DEFAULT_QPOASES_ENABLE_REGULARISATION,              "qpOases Regularisation term for the first task. it multiplies the default which is epsRegularisation = 5.0e3*EPS"),
new ParamProxyBasic<bool>("qpOases_enableRegularisation1",         PARAM_ID_QPOASES_ENABLEREGULARISATION1,       1,                                               PARAM_IN_OUT,     &SOT_DEFAULT_QPOASES_ENABLE_REGULARISATION,              "qpOases Regularisation term for the second task. it multiplies the default which is epsRegularisation = 5.0e3*EPS"),
new ParamProxyBasic<bool>("qpOases_enableRegularisation2",         PARAM_ID_QPOASES_ENABLEREGULARISATION2,       1,                                               PARAM_IN_OUT,     &SOT_DEFAULT_QPOASES_ENABLE_REGULARISATION,              "qpOases Regularisation term for the third task. it multiplies the default which is epsRegularisation = 5.0e3*EPS"),
new ParamProxyBasic<double>("qpOases_epsRegularisationMultiplier0",PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER0,1,                                               PARAM_IN_OUT,     &SOT_DEFAULT_QPOASES_EPS_REGULARISATION_MULTIPLIER,      "enable regularisation for the first task"),
new ParamProxyBasic<double>("qpOases_epsRegularisationMultiplier1",PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER1,1,                                               PARAM_IN_OUT,     &SOT_DEFAULT_QPOASES_EPS_REGULARISATION_MULTIPLIER,      "enable regularisation for the second task"),
new ParamProxyBasic<double>("qpOases_epsRegularisationMultiplier2",PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER2,1,                                               PARAM_IN_OUT,     &SOT_DEFAULT_QPOASES_EPS_REGULARISATION_MULTIPLIER,      "enable regularisation for the first task"),
new ParamProxyBasic<bool>("is_clik"                               ,PARAM_ID_CLIK,                                1,                                               PARAM_IN_OUT,     &SOT_DEFAULT_CLIK,                                       "enable clik (closed loop inverse kinematics"),
new ParamProxyBasic<bool>("update_world"                               ,PARAM_ID_WORLD_UPDATE,                                1,                                               PARAM_IN_OUT,     &SOT_DEFAULT_WORLD_UPDATE,                                       "update world pose using forward kinematics"),
// ************************************************* CONFIGURATION PARAMETERS ****************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<bool>("left_arm_impedance_control",            PARAM_ID_LEFT_ARM_IMPEDANCE_CONTROL,          1,                                               PARAM_CONFIG,     NULL,                                                    "use joint impedance control for left arm"),
new ParamProxyBasic<bool>("right_arm_impedance_control",           PARAM_ID_RIGHT_ARM_IMPEDANCE_CONTROL,         1,                                               PARAM_CONFIG,     NULL,                                                    "use joint impedance control for right arm"),
new ParamProxyBasic<bool>("torso_impedance_control",               PARAM_ID_TORSO_IMPEDANCE_CONTROL,             1,                                               PARAM_CONFIG,     NULL,                                                    "use joint impedance control for torso"),
new ParamProxyBasic<double>("dT",                                  PARAM_ID_DT,                                  1,        ParamLowerBound<double>(1e-3),         PARAM_CONFIG,     NULL,                                                    "frequency to use for the SoT (1/dT , dT is [s])")
};


// *** IDs of all the module command
enum sot_VelKinCon_CommandId {
    COMMAND_ID_HELP,
    COMMAND_ID_SAVE_PARAMS,
    COMMAND_ID_SIZE
};
// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE COMMANDS ********************************************
// ******************************************************************************************************************************
const CommandDescription sot_VelKinCon_CommandDescr[COMMAND_ID_SIZE]  =
{
//                  NAME            ID                          DESCRIPTION
CommandDescription("help",          COMMAND_ID_HELP,            "Get instructions about how to communicate with this module"),
CommandDescription("saveParams",    COMMAND_ID_SAVE_PARAMS,     "saveParams(string fileName) # Save the actual configuration parameters to file, inside the sot context folder"),
};


}


#endif // SOT_VELKINCON_CONSTANTS_H
