#ifndef _OPENSOT_GAIN_TYPE_
#define _OPENSOT_GAIN_TYPE_

namespace OpenSoT { namespace tasks { namespace acceleration {

/**
 * @brief The GainType enum is used to select the
 * feedback type between force (Kp, Kd have the meaning
 * of stiffness and damping) and acceleration (Kp, Kd
 * are treated as a matrix-valued lambda1, lambda2)
 */
enum GainType
{
    Force,
    Acceleration
};

}
}
}

#endif
