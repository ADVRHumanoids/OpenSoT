#include <OpenSoT/tasks/velocity/Gaze.h>
#define GAZE_THRESHOLD 0.2 //[m]

using namespace OpenSoT::tasks::velocity;


Gaze::Gaze(std::string task_id,
           const Eigen::VectorXd& x,
           XBot::ModelInterface &robot,
           std::string base_link,
           std::string distal_link) :
    Task(task_id, robot.getNv()),
    _distal_link(distal_link),
    _cartesian_task(new Cartesian(task_id, x, robot, _distal_link, base_link)),
    _subtask(new SubTask(_cartesian_task, Indices::range(4,5))),
    _robot(robot), _tmp_vector(3), _bl_T_gaze_kdl(),
    _gaze_goal()
{
    this->_update(x);
}

Gaze::~Gaze()
{

}

void Gaze::setLambda(double lambda)
{
    _subtask->setLambda(lambda);
    _lambda = _subtask->getLambda();
}

void Gaze::setGaze(const KDL::Frame& desiredGaze)
{
    _tmpEigenM2.setIdentity();
    //Here we just need the position part
    _tmpEigenM2.translation().x() = desiredGaze.p.x();
    _tmpEigenM2.translation().y() = desiredGaze.p.y();
    _tmpEigenM2.translation().z() = desiredGaze.p.z();
    setGaze(_tmpEigenM2);
}

void Gaze::setGaze(const Eigen::MatrixXd& desiredGaze)
{
    _tmpEigenM2.setIdentity();
    //Here we just need the position part
    _tmpEigenM2.translation().x() = desiredGaze(0,3);
    _tmpEigenM2.translation().y() = desiredGaze(1,3);
    _tmpEigenM2.translation().z() = desiredGaze(2,3);
    setGaze(_tmpEigenM2);
}

void Gaze::setGaze(const Eigen::Affine3d &desiredGaze)
{    
    _tmpEigenM.setIdentity();

    if(_cartesian_task->baseLinkIsWorld())
        _robot.getPose(_distal_link, _tmpEigenM);
    else
        _robot.getPose(_distal_link, _cartesian_task->getBaseLink(), _tmpEigenM);


    _gaze_T_obj = _tmpEigenM.inverse()*desiredGaze;
    _tmp_vector(0) = _gaze_T_obj.translation().x();
    _tmp_vector(1) = _gaze_T_obj.translation().y();
    _tmp_vector(2) = _gaze_T_obj.translation().z();

    _gaze_goal = _gaze_goal.Identity();

    if(_tmp_vector.norm() >= GAZE_THRESHOLD){
        cartesian_utils::computePanTiltMatrix(_tmp_vector, _gaze_goal);
    //cartesian_utils::computePanTiltMatrix(gaze_T_obj.subcol(0, 3, 3), gaze_goal);

        _cartesian_task->setReference(_bl_T_gaze_kdl*_gaze_goal);}
}

void Gaze::setOrientationErrorGain(const double& orientationErrorGain)
{
    _cartesian_task->setOrientationErrorGain(orientationErrorGain);
}

const double Gaze::getOrientationErrorGain() const
{
    return _cartesian_task->getOrientationErrorGain();
}

void Gaze::setWeight(const Eigen::MatrixXd &W)
{
    this->_W = W;
    _subtask->setWeight(W);
}

std::list<OpenSoT::SubTask::ConstraintPtr> &Gaze::getConstraints()
{
    return _subtask->getConstraints();
}

const unsigned int Gaze::getTaskSize() const
{
    return _subtask->getTaskSize();
}

void Gaze::_update(const Eigen::VectorXd &x)
{
    _subtask->update(x);
    this->_A = _subtask->getA();
    this->_b = _subtask->getb();
    this->_hessianType = _subtask->getHessianAtype();
    this->_W = _subtask->getWeight();
}

std::vector<bool> Gaze::getActiveJointsMask()
{
    return _subtask->getActiveJointsMask();
}

bool Gaze::setActiveJointsMask(const std::vector<bool> &active_joints_mask)
{
    return _subtask->setActiveJointsMask(active_joints_mask);
}

bool Gaze::setBaseLink(const std::string& base_link)
{
    return _cartesian_task->setBaseLink(base_link);
}
