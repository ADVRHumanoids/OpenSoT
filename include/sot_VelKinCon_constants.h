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

// *** DEFAULT PARAMETER VALUES
// Streaming parameters
static const double                     SOT_DEFAULT_ELAPSED(0.0);
static const yarp::sig::Vector          SOT_DEFAULT_ERROR(3, 0.0);
static const int                        SOT_DEFAULT_QPOASES_NWSR(2^32);
static const bool                       SOT_DEFAULT_QPOASES_ENABLE_REGULARISATION(true);
static const double                     SOT_DEFAULT_QPOASES_EPS_REGULARISATION_MULTIPLIER(2e2);
static const bool                       SOT_DEFAULT_CLIK(false);

static const paramHelp::ParamBilatBounds<double> SOT_MAX_JOINT_VELOCITY_BOUNDS(ParamBilatBounds<double>(0,1.0));
static const paramHelp::ParamBilatBounds<int> SOT_LAST_STACK_TYPE_BOUNDS(ParamBilatBounds<int>(0,2));
static const paramHelp::ParamLowerBound<int> SOT_QPOASES_NWSR_BOUND(ParamLowerBound<int>(0));
static const paramHelp::ParamBilatBounds<double> SOT_POSTURAL_WEIGHT_COEFFICENT_BOUNDS(ParamBilatBounds<double>(0,1.0));


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
    PARAM_ID_LAST_STACK_TYPE,
    PARAM_ID_POSTURAL_WEIGHT_COEFFICIENT,
    PARAM_ID_MINEFFORT_WEIGHT_COEFFICIENT, PARAM_ID_MINEFFORT_WEIGHT_NORMALIZATION,
    PARAM_ID_W_TORSO_WEIGHT,
    PARAM_ID_QPOASES_NWSR0, PARAM_ID_QPOASES_NWSR1, PARAM_ID_QPOASES_NWSR2,
    PARAM_ID_QPOASES_ENABLEREGULARISATION0, PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER0,
    PARAM_ID_QPOASES_ENABLEREGULARISATION1, PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER1,
    PARAM_ID_QPOASES_ENABLEREGULARISATION2, PARAM_ID_QPOASES_EPSREGULARISATIONMULTIPLIER2,
    PARAM_ID_CLIK,
// ***************************************** CONFIGURATION PARAMETERS ***********************************************************
    PARAM_ID_DT,
    PARAM_ID_SIZE /*This is the number of parameters, so it must be the last value of the enum.*/
};

enum sot_VelKinCon_last_stack_type {
    // (A=I, b=(q-q_ref), q_ref <- initial_config.ini
    LAST_STACK_TYPE_POSTURAL,
    // (A=I, b=grad_g(q)^T)
    LAST_STACK_TYPE_MINIMUM_EFFORT,
    // (A=[I;I], b=[q-q_ref;grad_g(q)^T])
    LAST_STACK_TYPE_POSTURAL_AND_MINIMUM_EFFORT
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
new ParamProxyBasic<double>("postural_weight_coefficient",         PARAM_ID_POSTURAL_WEIGHT_COEFFICIENT,         1,        &SOT_POSTURAL_WEIGHT_COEFFICENT_BOUNDS,PARAM_IN_OUT,     NULL,                                                    "postural weight coefficient. It is a scalar multiplying the postural weight gradient. Especially important when using last_stack_type 1 and 3"),
new ParamProxyBasic<double>("mineffort_weight_coefficient",        PARAM_ID_MINEFFORT_WEIGHT_COEFFICIENT,        1,                                               PARAM_IN_OUT,     NULL,                                                    "mineffort weight coefficient. It is a scalar multiplying the mineffort weight gradient. Especially important when using last_stack_type 1 and 3"),
new ParamProxyBasic<bool>("mineffort_weight_normalization",        PARAM_ID_MINEFFORT_WEIGHT_NORMALIZATION,      1,                                               PARAM_IN_OUT,     NULL,                                                    "mineffort weight normalization. If true, normalize the gradient with the torque limits"),
new ParamProxyBasic<double>("W_torso_weight",                      PARAM_ID_W_TORSO_WEIGHT,                      1,                                               PARAM_IN_OUT,     NULL,                                                    "what weight the torso should have in Postural Weight Scheme 1"),
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
// ************************************************* CONFIGURATION PARAMETERS ****************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<bool>("left_arm_impedance_control",            PARAM_ID_LEFT_ARM_IMPEDANCE_CONTROL,          1,                                               PARAM_CONFIG,     NULL,                                                    "use joint impedance control for left arm"),
new ParamProxyBasic<bool>("right_arm_impedance_control",           PARAM_ID_RIGHT_ARM_IMPEDANCE_CONTROL,         1,                                               PARAM_CONFIG,     NULL,                                                    "use joint impedance control for right arm"),
new ParamProxyBasic<bool>("torso_impedance_control",               PARAM_ID_TORSO_IMPEDANCE_CONTROL,             1,                                               PARAM_CONFIG,     NULL,                                                    "use joint impedance control for torso"),
new ParamProxyBasic<int>("last_stack_type",                        PARAM_ID_LAST_STACK_TYPE,                     1,        &SOT_LAST_STACK_TYPE_BOUNDS,           PARAM_CONFIG,     NULL,                                                    "type of last stack. It is an enum, with values 0: postural (A=I, b=(q-q_ref)) 1: postural and gravity gradient (A=[I;grad_g(q)], b=[q-q_ref;0]) 2: (gravity torque) minimum effort (A=I, b=grad_g(q)^T) 3: postural and minimum effort (wrt gravity) (A=[I;I], b =[q-q_ref;grad_g(q)^T])"),
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
