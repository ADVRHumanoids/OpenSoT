#include <OpenSoT/tasks/velocity/Manipulability.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;
using namespace yarp::math;

Manipulability::Manipulability(const yarp::sig::Vector& x, const iDynUtils& robot_model,
                               const Cartesian::Ptr CartesianTask):
    Task("manipulability::"+CartesianTask->getTaskID(), x.size()),
    _manipulabilityIndexGradientWorker(x, robot_model, CartesianTask),
    _x(x)
{
    _W.resize(_x_size, _x_size);
    _W.eye();

    _hessianType = HST_POSDEF;

    _A.resize(_x_size, _x_size);
    _A.eye();

    /* first update. Setting desired pose equal to the actual pose */
    this->_update(x);

}

Manipulability::Manipulability(const yarp::sig::Vector& x, const iDynUtils& robot_model,
                               const CoM::Ptr CartesianTask):
    Task("manipulability::"+CartesianTask->getTaskID(), x.size()),
    _manipulabilityIndexGradientWorker(x, robot_model,CartesianTask),
    _x(x)
{
    _W.resize(_x_size, _x_size);
    _W.eye();

    _hessianType = HST_POSDEF;

    _A.resize(_x_size, _x_size);
    _A.eye();

    /* first update. Setting desired pose equal to the actual pose */
    this->_update(x);
}

Manipulability::~Manipulability()
{

}

void Manipulability::_update(const yarp::sig::Vector &x) {

    _x = x;
    /************************* COMPUTING TASK *****************************/

    _b = _lambda*cartesian_utils::computeGradient(x, _manipulabilityIndexGradientWorker, this->getActiveJointsMask());

    /**********************************************************************/
}

double Manipulability::ComputeManipulabilityIndex()
{
    return _manipulabilityIndexGradientWorker.compute(_x);
}
