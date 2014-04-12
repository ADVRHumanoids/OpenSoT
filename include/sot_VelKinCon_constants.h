#ifndef SOT_VELKINCON_CONSTANTS_H
#define SOT_VELKINCON_CONSTANTS_H

/** TODO: PUT ALL THIS DEFINES IN A CONFIG FILE **/

#define TORSO_WEIGHT 1.0
#define MAX_JOINT_VELOCITY toRad(20.0) //[rad/sec]
#define ORIENTATION_ERROR_GAIN 1.0
#define SET_3_TASKS false

#define DEBUG
#define MODULE_NAME "sot_VelKinCon"
#define dT 0.025 //[s]

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

// *** IDs of all the module streaming parameters
enum sot_VelKinCon_ParamId {
    PARAM_ID_COMPUTATION_TIME,
    PARAM_ID_LEFT_ARM_POSITION_ERROR,    PARAM_ID_LEFT_ARM_ORIENTATION_ERROR,
    PARAM_ID_RIGHT_ARM_POSITION_ERROR,   PARAM_ID_RIGHT_ARM_ORIENTATION_ERROR,
    PARAM_ID_SWING_FOOT_POSITION_ERROR,  PARAM_ID_SWING_FOOT_ORIENTATION_ERROR,
    PARAM_ID_COM_POSITION_ERROR,
    PARAM_ID_SIZE /*This is the number of parameters, so it must be the last value of the enum.*/
};


// ******************************************************************************************************************************
// ****************************************** DESCRIPTION OF ALL THE MODULE PARAMETERS ******************************************
// ******************************************************************************************************************************
const ParamProxyInterface *const sot_VelKinCon_ParamDescr[PARAM_ID_SIZE] =
{
//                          NAME                ID                                   SIZE       CONSTRAINTS                                 I/O ACCESS          DEFAULT VALUE                   DESCRIPTION
// ************************************************* STREAMING OUTPUT PARAMETERS ****************************************************************************************************************************************************************************************************************************
new ParamProxyBasic<double>("t_elapsed",       PARAM_ID_COMPUTATION_TIME,             1,        ParamConstraint<double>(),                  PARAM_MONITOR,      &SOT_DEFAULT_ELAPSED,               "Time taken by each invocation of run()"),
new ParamProxyBasic<double>("eLWrist_p",       PARAM_ID_LEFT_ARM_POSITION_ERROR,      3,        ParamConstraint<double>(),                  PARAM_MONITOR,      SOT_DEFAULT_ERROR.data(),       "Position error of left arm"),
new ParamProxyBasic<double>("eLWrist_o",       PARAM_ID_LEFT_ARM_ORIENTATION_ERROR,   3,        ParamConstraint<double>(),                  PARAM_MONITOR,     SOT_DEFAULT_ERROR.data(),       "Orientation error of left arm"),
new ParamProxyBasic<double>("eRWrist_p",       PARAM_ID_RIGHT_ARM_POSITION_ERROR,     3,        ParamConstraint<double>(),                  PARAM_MONITOR,     SOT_DEFAULT_ERROR.data(),       "Position error of right arm"),
new ParamProxyBasic<double>("eRWrist_o",       PARAM_ID_RIGHT_ARM_ORIENTATION_ERROR,  3,        ParamConstraint<double>(),                  PARAM_MONITOR,     SOT_DEFAULT_ERROR.data(),       "Orientation error of right arm"),
new ParamProxyBasic<double>("eSwingFoot_p",    PARAM_ID_SWING_FOOT_POSITION_ERROR,    3,        ParamConstraint<double>(),                  PARAM_MONITOR,     SOT_DEFAULT_ERROR.data(),       "Position error of swing foot"),
new ParamProxyBasic<double>("eSwingFoot_o",    PARAM_ID_SWING_FOOT_ORIENTATION_ERROR, 3,        ParamConstraint<double>(),                  PARAM_MONITOR,     SOT_DEFAULT_ERROR.data(),       "Orientation error of swing foot"),
new ParamProxyBasic<double>("eCoM",            PARAM_ID_COM_POSITION_ERROR,           3,        ParamConstraint<double>(),                  PARAM_MONITOR,     SOT_DEFAULT_ERROR.data(),       "Position error of COM")
};

}


#endif // SOT_VELKINCON_CONSTANTS_H
