#include <OpenSoT/tasks/velocity/MinimizeAcceleration.h>
#include <yarp/math/Math.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;
using namespace yarp::math;

MinimizeAcceleration::MinimizeAcceleration(const yarp::sig::Vector &x):
    Task("MinimizeAcceleration", x.size()),
    _x_before(x.size(),0.0)
{
    _W.resize(_x_size, _x_size);
    _W.eye();

    _A.resize(_x_size, _x_size);
    _A.eye();

    _hessianType = HST_IDENTITY;

    this->_update(x);
}

MinimizeAcceleration::~MinimizeAcceleration()
{

}

void MinimizeAcceleration::_update(const yarp::sig::Vector &x) {
    _b = x - _x_before;
    _x_before = x;
}

void MinimizeAcceleration::setLambda(double lambda){
    this->_lambda = lambda;
}
