#include <OpenSoT/tasks/force/Wrench.h>

using namespace OpenSoT::tasks::force;

Wrench::Wrench(const Eigen::VectorXd &x):
    Task("Wrench", x.size()), _x(x),
    _x_desired(x.size())
{
    _x_desired.setZero(_x_size);

    _W.setIdentity(_x_size, _x_size);
    _A.setIdentity(_x_size, _x_size);

    _hessianType = HST_IDENTITY;

    /* first update. Setting desired pose equal to the actual pose */
    this->setReference(x);
    this->_update(x);
}

void Wrench::_update(const Eigen::VectorXd &x) {
    _x = x;

    /************************* COMPUTING TASK *****************************/

    this->update_b();


    /**********************************************************************/
}

void Wrench::setReference(const Eigen::VectorXd& x_desired) {
    if(x_desired.size() == _x_size)
    {
        _x_desired = x_desired;
        this->update_b();
    }
}

Eigen::VectorXd Wrench::getReference() const
{
    return _x_desired;
}

void Wrench::update_b() {
    _b = _lambda*(_x_desired - _x);
}

void Wrench::setLambda(double lambda)
{
    if(lambda >= 0.0){
        this->_lambda = lambda;
        this->update_b();
    }
}

Eigen::VectorXd Wrench::getError()
{
    return _x_desired - _x;
}

Eigen::VectorXd Wrench::getActualWrench()
{
    return _x;
}
