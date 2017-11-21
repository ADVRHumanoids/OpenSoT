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

#ifndef ForceAccExample_PLUGIN_H_
#define ForceAccExample_PLUGIN_H_

#include <XCM/XBotControlPlugin.h>
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/constraints/acceleration/DynamicFeasibility.h>
#include <OpenSoT/tasks/force/CoM.h>
#include <OpenSoT/utils/AutoStack.h>

namespace XBotPlugin {

/**
 * @brief ForceAccExample XBot RT Plugin
 *
 **/
class ForceAccExample : public XBot::XBotControlPlugin
{

public:

    virtual bool init_control_plugin(XBot::Handle::Ptr handle);

    virtual bool close();

    virtual void on_start(double time);

    virtual void on_stop(double time);
    
    virtual ~ForceAccExample();

protected:

    virtual void control_loop(double time, double period);

private:

    XBot::RobotInterface::Ptr _robot;
    XBot::ModelInterface::Ptr _model;

    double _start_time;

    Eigen::VectorXd _q0;

    XBot::MatLogger::Ptr _logger;
    
    OpenSoT::AffineHelper _qddot;
    std::vector<OpenSoT::AffineHelper> _wrenches;
    
    OpenSoT::tasks::force::CoM::Ptr _com_task;
    OpenSoT::tasks::acceleration::Postural::Ptr _postural_task;
    OpenSoT::constraints::acceleration::DynamicFeasibility::Ptr _dyn_feas;
    
    OpenSoT::AutoStack::Ptr _autostack;
    OpenSoT::solvers::QPOases_sot::Ptr _solver;

};

}


bool XBotPlugin::ForceAccExample::init_control_plugin(XBot::Handle::Ptr handle)
{
    _robot = handle->getRobotInterface();
    _logger = XBot::MatLogger::getLogger("opensot_force_acc_example");
    
    _model = XBot::ModelInterface::getModel(handle->getPathToConfigFile());
    
    std::vector<std::string> contact_links = {"foot_fl", "foot_fr", "foot_hr", "foot_hl"};
    
    OpenSoT::OptvarHelper::VariableVector vars;
    vars.emplace_back("qddot", _model->getJointNum());
    
    for(auto cl : contact_links){
        vars.emplace_back(cl, 6);
    }
    
    OpenSoT::OptvarHelper opt(vars);
    
    _qddot = opt.getVariable("qddot");
    
    for(auto cl : contact_links){
        _wrenches.emplace_back(opt.getVariable(cl));
    }
    
    
    
    
    
    _com_task = boost::make_shared<OpenSoT::tasks::force::CoM>(_wrenches, contact_links, *_model);
    
    _postural_task = boost::make_shared<OpenSoT::tasks::acceleration::Postural>("POSTURAL", 
                                                                                *_model, 
                                                                                _qddot);
    
    _dyn_feas = boost::make_shared<OpenSoT::constraints::acceleration::DynamicFeasibility>("DYN_FEAS", 
                                                                                           *_model, 
                                                                                           _qddot, 
                                                                                           _wrenches,
                                                                                            contact_links
                                                                                          );
    
//     _autostack = (_postural_task + _com_task) << _dyn_feas;
    
    _solver = boost::make_shared<OpenSoT::solvers::QPOases_sot>(_autostack->getStack(), _autostack->getBounds());
    
    return true;
}


#endif // ForceAccExample_PLUGIN_H_
