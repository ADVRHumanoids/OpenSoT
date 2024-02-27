#include <OpenSoT/tasks/velocity/Manipulability.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;

Manipulability::Manipulability(const XBot::ModelInterface& robot_model,
                               const Cartesian::Ptr CartesianTask, const double step):
    Task("manipulability::"+CartesianTask->getTaskID(), robot_model.getNv()),
    _manipulabilityIndexGradientWorker(robot_model, CartesianTask),
    _model(robot_model),
    _step(step)
{
    _W.resize(_x_size, _x_size);
    _W.setIdentity(_x_size, _x_size);

    _hessianType = HST_POSDEF;

    _A.resize(_x_size, _x_size);
    _A.setIdentity(_x_size, _x_size);

    _gradient.resize(_model.getNv());
    _deltas.resize(_model.getNv());

    /* first update. Setting desired pose equal to the actual pose */
    this->_update(Eigen::VectorXd(0));

}

Manipulability::Manipulability(const XBot::ModelInterface& robot_model,
                               const CoM::Ptr CartesianTask, const double step):
    Task("manipulability::"+CartesianTask->getTaskID(), robot_model.getNv()),
    _manipulabilityIndexGradientWorker(robot_model,CartesianTask),
    _model(robot_model),
    _step(step)
{
    _W.resize(_x_size, _x_size);
    _W.setIdentity(_x_size, _x_size);

    _hessianType = HST_POSDEF;

    _A.resize(_x_size, _x_size);
    _A.setIdentity(_x_size, _x_size);

    _gradient.resize(_model.getNv());
    _deltas.resize(_model.getNv());


    /* first update. Setting desired pose equal to the actual pose */
    this->_update(Eigen::VectorXd(0));
}

Manipulability::~Manipulability()
{

}

void Manipulability::_update(const Eigen::VectorXd &x) {

    _model.getJointPosition(_q);

    /************************* COMPUTING TASK *****************************/
    _gradient.setZero();
    _deltas.setZero();


    for(unsigned int i = 0; i < _gradient.size(); ++i)
    {
        if(this->getActiveJointsMask()[i])
        {
            _deltas[i] = _step;
            double fun_a = _manipulabilityIndexGradientWorker.compute(_model.sum(_q, _deltas));
            double fun_b = _manipulabilityIndexGradientWorker.compute(_model.sum(_q, -_deltas));

            _gradient[i] = (fun_a - fun_b)/(2.0*_step);
            _deltas[i] = 0.0;
        } else
            _gradient[i] = 0.0;
    }

    _b = _lambda * _gradient;

    /**********************************************************************/
}

double Manipulability::ComputeManipulabilityIndex()
{
    return _manipulabilityIndexGradientWorker.compute(_model.getJointPosition());
}
