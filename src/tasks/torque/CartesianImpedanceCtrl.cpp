/*
 * Copyright (C) 2016 Cogimon
 * Authors: Enrico Mingo Hoffman, Alessio Rocchi
 * email:  enrico.mingo@iit.it, alessio.rocchi@iit.it
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

#include <OpenSoT/tasks/torque/CartesianImpedanceCtrl.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::torque;

#define LAMBDA_THS 1E-12

CartesianImpedanceCtrl::CartesianImpedanceCtrl(std::string task_id,
                     const Eigen::VectorXd& x,
                     XBot::ModelInterface &robot,
                     std::string distal_link,
                     std::string base_link, const std::list<unsigned int> rowIndices) :
    Task(task_id, x.size()), _robot(robot),
    _distal_link(distal_link), _base_link(base_link),
    _desiredTwist(6), _F_ff(6), _use_inertia_matrix(true), _qdot(_x_size),
     _rows_indices(rowIndices)
{   
    _desiredTwist.setZero(_desiredTwist.size());
    _F_ff.setZero(_F_ff.size());

    this->_base_link_is_world = (_base_link == WORLD_FRAME_NAME);

    if(!this->_base_link_is_world) {
        this->_base_link_index = robot.getLinkID(_base_link);
        assert(this->_base_link_index >= 0);
    }

    this->_distal_link_index = robot.getLinkID(_distal_link);
    assert(this->_distal_link_index >= 0);

    if(!this->_base_link_is_world)
        assert(this->_distal_link_index != _base_link_index);

    _K.resize(6, 6);
    _K.setIdentity(_K.rows(), _K.cols()); _K = 100.0*_K;
    _D.resize(6, 6);
    _D.setIdentity(_D.rows(), _D.cols()); _D = 1.0*_D;

    _tmpA.resize(6, _x_size);
    _tmpA.setZero(6, _x_size);

    _tmpF.resize(6);
    _tmpF.setZero(6);

    _tmp_vec.resize(_x_size);
    _tmp_vec.setZero(_x_size);

    if(rowIndices.size() > 0)
    {
        _A.resize(rowIndices.size(), _x_size);
        _F.resize(rowIndices.size());

        _W.resize(rowIndices.size(), rowIndices.size());
        _W.setIdentity(rowIndices.size(), rowIndices.size());
    }
    else
    {
        _W.resize(6, 6);
        _W.setIdentity(6, 6);
    }

    /* first update. Setting desired pose equal to the actual pose */
    this->_update(x);

    _lambda = 1.0;

    _hessianType = HST_SEMIDEF;
}

CartesianImpedanceCtrl::~CartesianImpedanceCtrl()
{
}

void CartesianImpedanceCtrl::generateA(const Eigen::MatrixXd &_tmpA)
{
    for(unsigned int i = 0; i < _rows_indices.asVector().size(); ++i)
        _A.row(i) = _tmpA.row(_rows_indices.asVector()[i]);
}

void CartesianImpedanceCtrl::generateF(const Eigen::VectorXd &_tmpF)
{
    for(unsigned int i = 0; i < _rows_indices.asVector().size(); ++i)
        _F.row(i) = _tmpF.row(_rows_indices.asVector()[i]);
}

void CartesianImpedanceCtrl::_update(const Eigen::VectorXd &x) {

    /************************* COMPUTING TASK *****************************/

    if(_base_link_is_world) {
        bool res =_robot.getJacobian(_distal_link, _tmpJ);
        assert(res);
    } else{
        bool res = _robot.getRelativeJacobian(_distal_link, _base_link, _tmpJ);
        assert(res);
    }

    if(_rows_indices.asVector().size() > 0)
        generateA(_tmpJ);
    else
        _A = _tmpJ;


    _J = _A;

    if (_use_inertia_matrix)
    {
        _robot.getInertiaInverse(_Minv);
        _A.noalias() = _J*_Minv;
    }


    if(_base_link_is_world)
        _robot.getPose(_distal_link, _tmp_affine);
    else
        _robot.getPose(_distal_link, _base_link, _tmp_affine);
    _actualPose = _tmp_affine.matrix();

    if(_desiredPose.rows() == 0) {
        /* initializing to zero error */
        _desiredPose = _actualPose;
        _b.setZero(_b.size());
    }

    this->update_b();

    this->_desiredTwist.setZero(_desiredTwist.size());

    /**********************************************************************/
}

void CartesianImpedanceCtrl::setStiffness(const Eigen::MatrixXd &Stiffness)
{
    if(Stiffness.cols() == 6 && Stiffness.rows() == 6)
        _K = Stiffness;
}

void CartesianImpedanceCtrl::setDamping(const Eigen::MatrixXd &Damping)
{
    if(Damping.cols() == 6 && Damping.rows() == 6)
        _D = Damping;
}

void CartesianImpedanceCtrl::setStiffnessDamping(const Eigen::MatrixXd &Stiffness, const Eigen::MatrixXd &Damping)
{
    setStiffness(Stiffness);
    setDamping(Damping);
}

void CartesianImpedanceCtrl::getStiffness(Eigen::MatrixXd &Stiffness)
{
    Stiffness = _K;
}

void CartesianImpedanceCtrl::getDamping(Eigen::MatrixXd &Damping){
    Damping = _D;
}

void CartesianImpedanceCtrl::getStiffnessDamping(Eigen::MatrixXd &Stiffness, Eigen::MatrixXd &Damping)
{
    getStiffness(Stiffness);
    getDamping(Damping);
}

void CartesianImpedanceCtrl::setFeedForwardForces(const Eigen::VectorXd& Fff)
{
    _F_ff = Fff;
}

void CartesianImpedanceCtrl::setFeedForwardForces(const KDL::Wrench& Fff)
{
    _F_ff[0] = Fff.force.x();
    _F_ff[1] = Fff.force.y();
    _F_ff[2] = Fff.force.z();
    _F_ff[3] = Fff.torque.x();
    _F_ff[4] = Fff.torque.y();
    _F_ff[5] = Fff.torque.z();
}

void CartesianImpedanceCtrl::getFeedForwardForces(Eigen::VectorXd& Fff)
{
    Fff = _F_ff;
}

void CartesianImpedanceCtrl::getFeedForwardForces(KDL::Wrench& Fff)
{
    Fff.force.x(_F_ff[0]);
    Fff.force.y(_F_ff[1]);
    Fff.force.z(_F_ff[2]);
    Fff.torque.x(_F_ff[3]);
    Fff.torque.y(_F_ff[4]);
    Fff.torque.z(_F_ff[5]);
}

void CartesianImpedanceCtrl::setReference(const Eigen::MatrixXd& desiredPose) {
    assert(desiredPose.rows() == 4);
    assert(desiredPose.cols() == 4);

    _desiredPose = desiredPose;
    _desiredTwist.setZero(_desiredTwist.size());
    this->update_b();
}

void CartesianImpedanceCtrl::setReference(const KDL::Frame& desiredPose)
{
    _desiredPose.setIdentity(4,4);
    _desiredPose(0,0) = desiredPose.M(0,0); _desiredPose(0,1) = desiredPose.M(0,1); _desiredPose(0,2) = desiredPose.M(0,2);
    _desiredPose(1,0) = desiredPose.M(1,0); _desiredPose(1,1) = desiredPose.M(1,1); _desiredPose(1,2) = desiredPose.M(1,2);
    _desiredPose(2,0) = desiredPose.M(2,0); _desiredPose(2,1) = desiredPose.M(2,1); _desiredPose(2,2) = desiredPose.M(2,2);
    _desiredPose(0,3) = desiredPose.p.x();
    _desiredPose(1,3) = desiredPose.p.y();
    _desiredPose(2,3) = desiredPose.p.z();

    _desiredTwist.setZero(_desiredTwist.size());
    this->update_b();
}

void CartesianImpedanceCtrl::setReference(const Eigen::MatrixXd &desiredPose,
                                          const Eigen::VectorXd &desiredTwist)
{
    assert(desiredTwist.size() == 6);
    assert(desiredPose.rows() == 4);
    assert(desiredPose.cols() == 4);

    _desiredPose = desiredPose;
    _desiredTwist = desiredTwist;
    this->update_b();
}

void CartesianImpedanceCtrl::setReference(const KDL::Frame& desiredPose,
                                          const KDL::Twist& desiredTwist)
{
    _desiredPose.setIdentity(4,4);
    _desiredPose(0,0) = desiredPose.M(0,0); _desiredPose(0,1) = desiredPose.M(0,1); _desiredPose(0,2) = desiredPose.M(0,2);
    _desiredPose(1,0) = desiredPose.M(1,0); _desiredPose(1,1) = desiredPose.M(1,1); _desiredPose(1,2) = desiredPose.M(1,2);
    _desiredPose(2,0) = desiredPose.M(2,0); _desiredPose(2,1) = desiredPose.M(2,1); _desiredPose(2,2) = desiredPose.M(2,2);
    _desiredPose(0,3) = desiredPose.p.x();
    _desiredPose(1,3) = desiredPose.p.y();
    _desiredPose(2,3) = desiredPose.p.z();

    _desiredTwist.setZero(6);
    _desiredTwist[0] = desiredTwist.vel.x();
    _desiredTwist[1] = desiredTwist.vel.y();
    _desiredTwist[2] = desiredTwist.vel.z();
    _desiredTwist[3] = desiredTwist.rot.x();
    _desiredTwist[4] = desiredTwist.rot.y();
    _desiredTwist[5] = desiredTwist.rot.z();

    this->update_b();
}

const void CartesianImpedanceCtrl::getReference(Eigen::MatrixXd& desired_pose) const {
    desired_pose = _desiredPose;
}

const void CartesianImpedanceCtrl::getReference(KDL::Frame& desired_pose) const {
    desired_pose.Identity();

    desired_pose.p.x(_desiredPose(0,3));
    desired_pose.p.y(_desiredPose(1,3));
    desired_pose.p.z(_desiredPose(2,3));

    desired_pose.M(0,0) = _desiredPose(0,0); desired_pose.M(0,1) = _desiredPose(0,1); desired_pose.M(0,2) = _desiredPose(0,2);
    desired_pose.M(1,0) = _desiredPose(1,0); desired_pose.M(1,1) = _desiredPose(1,1); desired_pose.M(1,2) = _desiredPose(1,2);
    desired_pose.M(2,0) = _desiredPose(2,0); desired_pose.M(2,1) = _desiredPose(2,1); desired_pose.M(2,2) = _desiredPose(2,2);
}

void CartesianImpedanceCtrl::getReference(Eigen::MatrixXd &desiredPose,
                                                       Eigen::VectorXd &desiredTwist) const
{
    desiredPose = _desiredPose;
    desiredTwist = _desiredTwist;
}

void CartesianImpedanceCtrl::getReference(KDL::Frame& desiredPose,
                                          KDL::Twist& desiredTwist) const
{
    getReference(desiredPose);
    desiredTwist.vel.x(_desiredTwist[0]);
    desiredTwist.vel.y(_desiredTwist[1]);
    desiredTwist.vel.z(_desiredTwist[2]);
    desiredTwist.rot.x(_desiredTwist[3]);
    desiredTwist.rot.y(_desiredTwist[4]);
    desiredTwist.rot.z(_desiredTwist[5]);
}

const void CartesianImpedanceCtrl::getActualPose(Eigen::MatrixXd &actual_pose) const
{
    actual_pose = _actualPose;
}

const void CartesianImpedanceCtrl::getActualPose(KDL::Frame& actual_pose) const
{
    actual_pose.Identity();

    actual_pose.p.x(_actualPose(0,3));
    actual_pose.p.y(_actualPose(1,3));
    actual_pose.p.z(_actualPose(2,3));

    actual_pose.M(0,0) = _actualPose(0,0); actual_pose.M(0,1) = _actualPose(0,1); actual_pose.M(0,2) = _actualPose(0,2);
    actual_pose.M(1,0) = _actualPose(1,0); actual_pose.M(1,1) = _actualPose(1,1); actual_pose.M(1,2) = _actualPose(1,2);
    actual_pose.M(2,0) = _actualPose(2,0); actual_pose.M(2,1) = _actualPose(2,1); actual_pose.M(2,2) = _actualPose(2,2);
}

const std::string CartesianImpedanceCtrl::getDistalLink() const
{
    return _distal_link;
}

const std::string CartesianImpedanceCtrl::getBaseLink() const
{
    return _base_link;
}

const bool CartesianImpedanceCtrl::baseLinkIsWorld() const
{
    return _base_link_is_world;
}

void CartesianImpedanceCtrl::getSpringForce(Eigen::VectorXd &spring_force)
{
    spring_force.resize(positionError.size()+orientationError.size());
    spring_force<<positionError, -1.0*orientationError;
    spring_force = _K*spring_force;
}

void CartesianImpedanceCtrl::getDamperForce(Eigen::VectorXd &damper_force)
{
    damper_force.resize(linearVelocityError.size()+orientationVelocityError.size());
    damper_force<<linearVelocityError, orientationVelocityError;
    damper_force = _D*damper_force;
}

void CartesianImpedanceCtrl::update_b() {
    cartesian_utils::computeCartesianError(_actualPose, _desiredPose,
                                           positionError, orientationError);



    _robot.getJointVelocity(_qdot);
    _xdot = _tmpJ*_qdot;
    linearVelocityError = _desiredTwist.segment(0,3) - _xdot.segment(0,3);
    orientationVelocityError = _desiredTwist.segment(3,3) - _xdot.segment(3,3);

    /// TODO: add -Mc*(ddx_d - Jdot*qdot)
    getDamperForce(_damping_force);
    getSpringForce(_spring_force);
    _tmpF = _damping_force + _spring_force + _F_ff;

    if(_rows_indices.size() > 0)
        generateF(_tmpF);
    else
        _F = _tmpF;

    _tmp_vec = _J.transpose()*_F;
    _b = _A*_tmp_vec;
}

bool CartesianImpedanceCtrl::isCartesianImpedanceCtrl(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return (bool)boost::dynamic_pointer_cast<CartesianImpedanceCtrl>(task);
}

CartesianImpedanceCtrl::Ptr CartesianImpedanceCtrl::asCartesianImpedanceCtrl(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task)
{
    return boost::dynamic_pointer_cast<CartesianImpedanceCtrl>(task);
}

void CartesianImpedanceCtrl::useInertiaMatrix(const bool use)
{
    _use_inertia_matrix = use;
}

void CartesianImpedanceCtrl::_log(XBot::MatLogger::Ptr logger)
{
    logger->add(_task_id+"_actualPose", _actualPose);
    logger->add(_task_id+"_desiredPose", _desiredPose);
    logger->add(_task_id+"_desiredTwist", _desiredTwist);

    logger->add(_task_id+"_K", _K);
    logger->add(_task_id+"_D", _D);

    logger->add(_task_id+"_positionError", positionError);
    logger->add(_task_id+"_orientationError", orientationError);
    logger->add(_task_id+"_linearVelocityError", linearVelocityError);
    logger->add(_task_id+"_orientationVelocityError", orientationVelocityError);

    logger->add(_task_id+"_Minv", _Minv);
    logger->add(_task_id+"_J", _J);

    logger->add(_task_id+"_qdot", _qdot);
    logger->add(_task_id+"_xdot", _xdot);

    logger->add(_task_id+"_spring_force", _spring_force);
    logger->add(_task_id+"_damping_force", _damping_force);

    logger->add(_task_id+"_F", _F);
}
