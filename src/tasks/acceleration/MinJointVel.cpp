#include <OpenSoT/tasks/acceleration/MinJointVel.h>
#include <memory>

using namespace OpenSoT::tasks::acceleration;

MinJointVel::MinJointVel(const XBot::ModelInterface &robot, double dT, AffineHelper qddot):
    Task< Eigen::MatrixXd, Eigen::VectorXd >("MinJointVel", qddot.getInputSize()),
    _robot(robot),
    _dT(dT)
{
    _postural = std::make_shared<OpenSoT::tasks::acceleration::Postural>(robot, qddot);

    _postural->setLambda(0.0, 1./_dT);

    _hessianType = HST_SEMIDEF;

    _update(Eigen::VectorXd(1));


    I = _postural->getWeight();


    setWeight(I);


}

void MinJointVel::_update(const Eigen::VectorXd &x)
{
    _postural->update(x);

    _A = _postural->getA();
    _b = _postural->getb();
}

bool MinJointVel::setEps(const double eps)
{
    if(eps < 0.)
    {
        XBot::Logger::error("eps should be positive!");
        return false;
    }

    setWeight(eps*I);
    return true;
}
