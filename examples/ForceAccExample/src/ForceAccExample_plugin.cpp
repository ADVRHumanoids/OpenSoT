/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:
 * email:
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <ForceAccExample_plugin.h>
#include <boost/make_shared.hpp>
#include <OpenSoT/SubTask.h>
#include <OpenSoT/constraints/GenericConstraint.h>

/* Specify that the class XBotPlugin::ForceAccExample is a XBot RT plugin with name "ForceAccExample" */
REGISTER_XBOT_PLUGIN_(XBotPlugin::ForceAccExample)


const std::string floating_base_name = "pelvis";

bool XBotPlugin::ForceAccExample::init_control_plugin(XBot::Handle::Ptr handle)
{
    _robot = handle->getRobotInterface();
    _logger = XBot::MatLogger::getLogger("/tmp/opensot_force_acc_example");
    
    _robot->getStiffness(_k);
    _robot->getDamping(_d);
    _k /= 16;
    _d /= 4;
    
    _imu = _robot->getImu().begin()->second;
    
    _model = XBot::ModelInterface::getModel(handle->getPathToConfigFile());
    
    Eigen::VectorXd qhome;
    _model->getRobotState("home", qhome);
    _model->setJointPosition(qhome);
    _model->update();
    
    _model->initLog(_logger, 10000);
    
    _sh_fb_pos = handle->getSharedMemory()->getSharedObject<Eigen::Vector3d>("/gazebo/floating_base_position");
    _sh_fb_vel = handle->getSharedMemory()->getSharedObject<Eigen::Vector3d>("/gazebo/floating_base_velocity");
    _sh_fb_pos.set(Eigen::Vector3d::Zero());
    _sh_fb_vel.set(Eigen::Vector3d::Zero());
    
    
    _contact_links = {"foot_fl", "foot_fr", "foot_hr", "foot_hl"};
    
    
    _wrench_value.assign(_contact_links.size(), Eigen::VectorXd::Zero(6));
    
    OpenSoT::OptvarHelper::VariableVector vars;
    vars.emplace_back("qddot", _model->getJointNum());
    
    for(auto cl : _contact_links){
        vars.emplace_back(cl, 3); // put 6 for full wrench
    }
    
    OpenSoT::OptvarHelper opt(vars);
    
    _qddot = opt.getVariable("qddot");
    
    Eigen::VectorXd wrench_ub(6), wrench_lb(6);
    wrench_ub << 1000, 1000, 1000, 1, 1, 1;
    wrench_lb << -1000, -1000, 10, -1, -1, -1;
    
    std::vector<OpenSoT::constraints::GenericConstraint::Ptr> wrench_bounds;
    
    for(auto cl : _contact_links){
        _wrenches.emplace_back(opt.getVariable(cl) / OpenSoT::AffineHelper::Zero(opt.getSize(), 3));
        
        _feet_cartesian.push_back(
            boost::make_shared<OpenSoT::tasks::acceleration::Cartesian>(cl + "_cartesian", 
                                                                        *_model, 
                                                                        cl, 
                                                                        "world", 
                                                                        _qddot)
                                 );
        
        wrench_bounds.push_back( boost::make_shared<OpenSoT::constraints::GenericConstraint>(cl+"_bound", 
                                                                                             _wrenches.back(), 
                                                                                             wrench_ub, 
                                                                                             wrench_lb) 
                               );
        
    }
    
    
    
    
    
    _com_task = boost::make_shared<OpenSoT::tasks::force::CoM>(_wrenches, _contact_links, *_model);
    
    _postural_task = boost::make_shared<OpenSoT::tasks::acceleration::Postural>("POSTURAL", 
                                                                                *_model, 
                                                                                _qddot);
    
    _dyn_feas = boost::make_shared<OpenSoT::constraints::acceleration::DynamicFeasibility>("DYN_FEAS", 
                                                                                           *_model, 
                                                                                           _qddot, 
                                                                                           _wrenches,
                                                                                            _contact_links
                                                                                          );
    
    auto feet_cart_aggr = _feet_cartesian[0] + _feet_cartesian[1] + _feet_cartesian[2] + _feet_cartesian[3];
    
    _waist_task = boost::make_shared<OpenSoT::tasks::acceleration::Cartesian>("waist_task", 
                                                                        *_model, 
                                                                        floating_base_name, 
                                                                        "world", 
                                                                        _qddot);
                                 
    
    std::list<uint> pos_idx = {0,1,2};
    std::list<uint> or_idx = {3,4,5};
    auto feet_pos_aggr = boost::make_shared<OpenSoT::SubTask>(feet_cart_aggr, pos_idx);
    auto feet_or_aggr = boost::make_shared<OpenSoT::SubTask>(feet_cart_aggr, or_idx);
    auto waist_or = boost::make_shared<OpenSoT::SubTask>(_waist_task, or_idx);
    
    _autostack = (  _waist_task  ) / ( _postural_task + feet_cart_aggr  ) << 
        _dyn_feas << 
        wrench_bounds[0] << wrench_bounds[1] << wrench_bounds[2] << wrench_bounds[3];
    
    _solver = boost::make_shared<OpenSoT::solvers::QPOases_sot>(_autostack->getStack(), 
                                                                _autostack->getBounds(), 
                                                                1e4);
    
    
    return true;
}

void XBotPlugin::ForceAccExample::on_start(double time)
{


//     _model->syncFrom(*_robot, XBot::Sync::Position);
//     

//     _model->getJointPosition(_q);
    
    _start_time = time;
    _qdot.setZero(_q.size());
    
    sync_model();
    
    for(auto ct : _feet_cartesian)
    {
        ct->resetReference();
    }
    
    _waist_task->resetReference();
    
    _model->getPointPosition(floating_base_name, Eigen::Vector3d::Zero(),_initial_com);
}

void XBotPlugin::ForceAccExample::control_loop(double time, double period)
{
    const bool enable_torque_ctrl = true;
    const bool enable_feedback = true;
    
    if(enable_feedback){
        
        sync_model();
        
//         _model->syncFrom(*_robot);
        
    }
    
    /* Set reference*/
    _waist_task->setPositionReference(_initial_com - 0.1*Eigen::Vector3d::UnitZ());
    
    /* Update stack */
    _autostack->update(Eigen::VectorXd::Zero(0));
    _autostack->log(_logger);
    
    /* Solve QP */
    _x.setZero(_x.size());
    if(!_solver->solve(_x))
    {
        Logger::error("Unable to solve!!!");
        return;
    }
    
    /* Retrieve values */
    _qddot.getValue(_x, _qddot_value);
    
    for(int i = 0; i < _wrenches.size(); i++){
        _wrenches[i].getValue(_x, _wrench_value[i]);
        _logger->add(_contact_links[i] + "_wrench", _wrench_value[i]);
    }
    
    Logger::info() << "Dyn feas: " << _dyn_feas->checkConstraint(_x).transpose() << Logger::endl();
    
    /* Compute torques due to contacts */
    _tau_c.setZero(_model->getJointNum());
    for(int i = 0; i < _contact_links.size(); i++){
        _model->getJacobian(_contact_links[i], _Jtmp);
        _tau_c.noalias() += _Jtmp.transpose()*_wrench_value[i];
    }
    
    /* Set solution inside model */
    _model->setJointAcceleration(_qddot_value);
    _model->update();
    
    /* Compute full ID */
    _model->computeInverseDynamics(_tau);
    _tau -= _tau_c;
    _model->setJointEffort(_tau);
    
    /* Update model */
    _model->getJointPosition(_q);
    _model->getJointVelocity(_qdot);
    
    _q.noalias() += 0.5*period*period*_qddot_value + period*_qdot;
    _qdot.noalias() += period*_qddot_value;
    
    _model->setJointPosition(_q);
    _model->setJointVelocity(_qdot);
    _model->update();
    
    
    _logger->add("tau", _tau);
    _logger->add("tau_c", _tau_c);
    _logger->add("qddot_value", _qddot_value);
    _logger->add("x", _x);
    
    /* Send commands to robot */
    if(enable_torque_ctrl){
        _robot->setStiffness(_k);
        _robot->setDamping(_d);
        _robot->setReferenceFrom(*_model, XBot::Sync::Position, XBot::Sync::Effort);
    }
    else{
        _robot->setReferenceFrom(*_model, XBot::Sync::Position, XBot::Sync::Effort);
    }
    
    _robot->move();
    _model->log(_logger, time);
    
    
    
}


void XBotPlugin::ForceAccExample::sync_model()
{
        _model->syncFrom(*_robot);
        
        Eigen::Affine3d w_T_fb;
        Eigen::Matrix3d w_R_fb;
        Eigen::Vector6d fb_twist;
        Eigen::Vector3d fb_vel, fb_omega, fb_pos;
        
        _sh_fb_pos.get(fb_pos);
        _sh_fb_vel.get(fb_vel);
        _imu->getAngularVelocity(fb_omega);
        _imu->getOrientation(w_R_fb);
        
        w_T_fb.linear() = w_R_fb;
        w_T_fb.translation() = fb_pos;
        fb_twist << fb_vel, fb_omega;
        
        _model->setFloatingBaseState(w_T_fb, fb_twist);
        _model->update();
        
        Logger::info(Logger::Severity::DEBUG) << "Pose:\n" << w_T_fb.matrix() << "\nVel: " << fb_twist.transpose() << Logger::endl();
        
        _model->getFloatingBasePose(w_T_fb);
        
        Logger::info(Logger::Severity::DEBUG) << "Actual pose:\n" << w_T_fb.matrix() << Logger::endl();
}


