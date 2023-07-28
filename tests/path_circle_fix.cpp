#include <trajectory_utils/paths/path_circle_fix.hpp>

using namespace KDL;

Path_Circle_Fix::Path_Circle_Fix(const Frame &F_base_start,
                                 const Vector &V_base_center,
                                 const Vector &V_base_p,
                                 const Rotation &R_base_end,
                                 double alpha,
                                 RotationalInterpolation *otraj,
                                 double eqradius, bool _aggregate):
    Path_Circle(F_base_start, V_base_center, V_base_p, R_base_end, alpha, otraj, eqradius, _aggregate)
{
    _F_base_start = F_base_start;
    _V_base_center = V_base_center;
    _V_base_p = V_base_p;
    _R_base_end = R_base_end;
    _alpha = alpha;
    _otraj = otraj;
    _eqradius = eqradius;
    __aggregate = _aggregate;
}

Path* Path_Circle_Fix::Clone() {
    return new Path_Circle_Fix(
        _F_base_start,
        _V_base_center,
        _V_base_p,
        _R_base_end,
        _alpha,
        _otraj->Clone(),
        _eqradius,
        __aggregate
    );
}
