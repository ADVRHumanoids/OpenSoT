/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
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

#include <OpenSoT/OpenSoT.h>
#include <OpenSoT/utils/logger/L.h>
#include <OpenSoT/utils/logger/plotters/Plotter.h>

void OpenSoT::L::udpate(double t, const yarp::sig::Vector &q_dot)
{
    _current_log << "(" << t << ",";
    for(unsigned int i = 0; i < q_dot.size(); ++i)
        _current_log << q_dot[i] << ",";
                    /*
        << (DHS.leftArm->getActualPose())(0,3) << ","       // 1
        << (DHS.leftArm->getActualPose())(1,3) << ","
        << (DHS.leftArm->getActualPose())(2,3) << ","
        << (DHSns.leftArm->getActualPose())(0,3) << ","     // 4
        << (DHSns.leftArm->getActualPose())(1,3) << ","
        << (DHSns.leftArm->getActualPose())(2,3) << ","
        << (DHS.leftArm->getReference())(0,3) << ","        // 7
        << (DHS.leftArm->getReference())(1,3) << ","
        << (DHS.leftArm->getReference())(2,3) << ","
        << r  << ","                                        // 10
        << p  << ","
        << y  << ","
        << r_ns  << ","                                     // 13
        << p_ns  << ","
        << y_ns  << ","
        << r_ref  << ","                                    // 16
        << p_ref  << ","
        << y_ref  << ","
        << e_com << ","                                     // 19
        << e_com_ns << ","
        << e_arms << ","                                    // 21
        << e_arms_ns << ","
        << e_post << ","                                    // 23
        << e_post_ns << ","
        << t_loop << ","                                    // 25
        << t_loopns */
    {
        typedef std::map<TaskPtr, flushers::TaskFlusher::Ptr>::const_iterator it_t;
        typedef std::map<ConstraintPtr, flushers::ConstraintFlusher::Ptr>::const_iterator it_c;

        for(it_t it = _taskFlushers.begin(); it != _taskFlushers.end(); ++it)
        {
            it->second->updateSolution(q_dot);
            _current_log << it->second;
        }

        for(it_c it = _constraintFlushers.begin(); it != _constraintFlushers.end(); ++it)
        {
            it->second->updateSolution(q_dot);
            _current_log << it->second;
        }
    }
    _current_log << ")," << std::endl;
}

bool OpenSoT::L::open(std::string logName)
{
    if(_format == FORMAT_PYTHON)
    {
        _current_log_filename = logName + ".py";
    }
    if(_must_append.count(_current_log_filename) > 0)
    {
        _current_log.open(_current_log_filename.c_str(), std::fstream::app);
        _must_append[_current_log_filename]++;
    }
    else
    {
        _current_log.open(_current_log_filename.c_str());
        _must_append[_current_log_filename] = 1;
    }

    if(_format == FORMAT_PYTHON)
    {
        _current_log << "#! /usr/bin/env python" << std::endl
             << std::endl
             << "import numpy as np" << std::endl
             << "import matplotlib" << std::endl
             << "from matplotlib.pyplot import *" << std::endl;
        /* @TODO this should be done by the flusher
        _log << "#t, x, y, z, r, p, y,"                 // 0-6
             << " xns, yns, zns, rns, pns, yns,"        // 7-12
             << " xref, yref, zref, rref, pref, yref,"  // 13-18
             << " e_com, e_com_ns,"                     // 19-20
             << " e_arms, e_arms_ns,"                   // 21-22
             << " e_post, e_post_ns,"                   // 23-24
             << " t_loop, t_loop_nva" << std::endl;     // 25-26
         */

        // @TODO here we should append a number to test_data
        _current_log << "data = np.array((";
    }

    return true;
}

bool OpenSoT::L::close()
{

    if(_format == FORMAT_PYTHON)
    {
        _current_log << "));" << std::endl;
        _current_log << std::endl;
        _current_log << plotter->getCommands() << std::endl;
        _collator << "execfile('" << _current_log_filename << "')" << std::endl;
    }

    /*

    _log << "se = figure('"<< smoothing_strategy_stream.str() << "- Cartesian Errors',figsize=(10.27,7.68));" << std::endl;
    std::string smoothing = smoothing_params_stream.str();
    std::string no_smoothing = regular_params_stream.str();

    _log << "subplot(3,2,1); p = plot(test_data[:,0], test_data[:,(1,4,7)]); title('l_arm x');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "', 'reference'));" << std::endl;
    _log << "ylabel('hand position [m]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(3,2,2); p = plot(test_data[:,0], test_data[:,(2,5,8)]); title('l_arm y');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "', 'reference'));" << std::endl;
    _log << "ylabel('hand position [m]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(3,2,3); p = plot(test_data[:,0], test_data[:,(3,6,9)]); title('l_arm z');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "', 'reference'));" << std::endl;
    _log << "ylabel('hand position [m]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(3,2,4); p = plot(test_data[:,0], test_data[:,(10,13,16)]); title('l_arm r');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "', 'reference'));" << std::endl;
    _log << "ylabel('hand orientation [rad]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(3,2,5); p = plot(test_data[:,0], test_data[:,(11,14,17)]); title('l_arm p');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "', 'reference'));" << std::endl;
    _log << "ylabel('hand orientation [rad]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(3,2,6); p = plot(test_data[:,0], test_data[:,(12,15,18)]); title('l_arm y');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "', 'reference'));" << std::endl;
    _log << "ylabel('hand orientation [rad]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "et = figure('"<< smoothing_strategy_stream.str() << "- Task Errors',figsize=(8,6));" << std::endl << std::endl;

    _log << "subplot(2,2,1); p = plot(test_data[:,0],test_data[:,(25, 26)]);" << std::endl;
    _log << "title('Computation Time');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "'));" << std::endl;
    _log << "ylabel('Solve Time [s]'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(2,2,2); p = plot(test_data[:,0],test_data[:,(19, 20)]);" << std::endl;
    _log << "title('CoM_XY Task Error');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "'));" << std::endl;
    _log << "ylabel('norm2 of task error'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(2,2,3); p = plot(test_data[:,0],test_data[:,(21, 22)]);" << std::endl;
    _log << "title('l_arm + r_arm Task Error');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "'));" << std::endl;
    _log << "ylabel('norm2 of task error'); xlabel('t [s]');" << std::endl << std::endl;

    _log << "subplot(2,2,4); p = plot(test_data[:,0],test_data[:,(23, 24)]);" << std::endl;
    _log << "title('Postural Task Error');" << std::endl;
    _log << "legend(p,('" << smoothing << "', '" << no_smoothing << "'));" << std::endl;
    _log << "ylabel('norm2 of task error'); xlabel('t [s]');" << std::endl << std::endl;

    if(params.strategy == STRATEGY_CARTESIAN_TUNING_1) {
        _log << "se.savefig('" << getPlotFilename(STRATEGY_CARTESIAN_TUNING_1, TEST_SCA_CT1_DISTANCES_FILE) << "', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << getPlotFilename(STRATEGY_CARTESIAN_TUNING_1, TEST_SCA_CT1_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_COM_TUNING) {
        _log << "se.savefig('" << getPlotFilename(STRATEGY_COM_TUNING, TEST_SCA_COM_DISTANCES_FILE) << "', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << getPlotFilename(STRATEGY_COM_TUNING, TEST_SCA_COM_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_BOUNDSCALING_TUNING) {
        _log << "se.savefig('" << getPlotFilename(STRATEGY_BOUNDSCALING_TUNING, TEST_SCA_BST_DISTANCES_FILE) << "', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << getPlotFilename(STRATEGY_BOUNDSCALING_TUNING, TEST_SCA_BST_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_EPS_TUNING) {
        _log << "se.savefig('" << getPlotFilename(STRATEGY_EPS_TUNING, TEST_SCA_EPS_DISTANCES_FILE) << "', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << getPlotFilename(STRATEGY_EPS_TUNING, TEST_SCA_EPS_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_SMALLER_STACK) {
        _log << "se.savefig('" << getPlotFilename(STRATEGY_SMALLER_STACK, TEST_SCA_SS_DISTANCES_FILE) << "', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << getPlotFilename(STRATEGY_SMALLER_STACK, TEST_SCA_SS_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_POSTURAL_TUNING) {
        _log << "se.savefig('" << getPlotFilename(STRATEGY_POSTURAL_TUNING, TEST_SCA_PT_DISTANCES_FILE) << "', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << getPlotFilename(STRATEGY_POSTURAL_TUNING, TEST_SCA_PT_ERRORS_FILE)    << "',format='eps',transparent=True);" << std::endl;
    } else if(params.strategy == STRATEGY_DISTANCE_SMOOTHING) {
        _log << "se.savefig('" << TEST_SCA_BST_DISTANCES_FILE << ".eps', format='eps', transparent=True);" << std::endl;
        _log << "et.savefig('" << TEST_SCA_BST_ERRORS_FILE << ".eps',format='eps',transparent=True);" << std::endl;
    } else {
        std::cerr << "Unhandled exception at line " << __LINE__ << std::endl;
        exit(1);
    }
    _log << "show(block=True)" << std::endl;

     */
    _current_log.close();
    _flushers.clear();
    _constraintFlushers.clear();
    _dataFlushers.clear();
    _taskFlushers.clear();

    return true;
}

OpenSoT::L::L(std::string loggerName, iDynUtils& model, OpenSoT::L::logger_format format)
    : _name(loggerName), _model(model), _format(format),
      _n_dofs(model.iDyn3_model.getNrOfDOFs()),
      fakeFlusher(_n_dofs+1)
{
    if(_format == FORMAT_PYTHON)
    {
        _collator.open((loggerName + ".py").c_str());
        _collator << "#! /usr/bin/env python" << std::endl;
        _collator << std::endl;
    }

    std::list<std::string> descriptions;
    descriptions.push_back("time");
    std::vector<std::string> joint_names = model.getJointNames();
    for(unsigned int i = 0; i < joint_names.size(); ++i)
        descriptions.push_back(joint_names[i] + "dq");
    fakeFlusher.setDescription(descriptions);
}

OpenSoT::L::~L()
{
    if(_current_log.is_open())
        this->close();
}

OpenSoT::flushers::TaskFlusher::Ptr OpenSoT::L::add(             OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task)
{
OpenSoT::flushers::TaskFlusher::Ptr taskFlusher;
// no task flushers are implemented at the moment
return taskFlusher;
}

OpenSoT::flushers::ConstraintFlusher::Ptr OpenSoT::L::add(       OpenSoT::Constraint<yarp::sig::Matrix, yarp::sig::Vector>::ConstraintPtr constraint)
{
    OpenSoT::flushers::ConstraintFlusher::Ptr constraintFlusher;
    if(boost::dynamic_pointer_cast<OpenSoT::constraints::velocity::Dynamics>(constraint))
    {
        // the Dynamics flusher need a model to obtain the proper description
        constraintFlusher.reset(
            new OpenSoT::flushers::constraints::velocity::Dynamics(
                boost::dynamic_pointer_cast<OpenSoT::constraints::velocity::Dynamics>(constraint),
                _model));
        _constraintFlushers[constraint] = constraintFlusher;
        _flushers.push_back(constraintFlusher);
    }
    return constraintFlusher;
}

OpenSoT::flushers::TaskFlusher::Ptr OpenSoT::L::getFlusher(      OpenSoT::Task<yarp::sig::Matrix, yarp::sig::Vector>::TaskPtr task)
{
    return _taskFlushers[task];
}

OpenSoT::flushers::ConstraintFlusher::Ptr OpenSoT::L::getFlusher(OpenSoT::Constraint<yarp::sig::Matrix, yarp::sig::Vector>::ConstraintPtr constraint)
{
    return _constraintFlushers[constraint];
}

OpenSoT::flushers::Flusher::Ptr OpenSoT::L::getFlusher(void* data)
{
    return _dataFlushers[data];
}

OpenSoT::L::logger_format OpenSoT::L::getFormat() const
{
    return _format;
}

std::string OpenSoT::L::getName() const
{
    return _name;
}

OpenSoT::Indices OpenSoT::L::getGlobalIndices(std::pair<OpenSoT::flushers::Flusher::Ptr, Indices> plottable)
{
    if(boost::dynamic_pointer_cast<OpenSoT::flushers::FakeFlusher>(plottable.first))
        return plottable.second;

    unsigned int minIndex = 0;
    minIndex += fakeFlusher.getSize()-1;
    for(unsigned int i = 0; i < _flushers.size(); ++i)
    {
        if(_flushers[i] == plottable.first)
        {
            Indices indices = plottable.second;
            std::vector<unsigned int> indices_v = indices.getRowsVector();
            for(unsigned int j = 0; j < indices_v.size(); ++j)
                indices_v[j] += minIndex;
            return Indices(indices_v);
        }
        else minIndex += _flushers[i]->getSize();
    }
}

unsigned int OpenSoT::L::getMaximumIndex()
{
    unsigned int maxIndex = 0;
    maxIndex += fakeFlusher.getSize()-1;
    for(unsigned int i = 0; i < _flushers.size(); ++i)
        maxIndex += _flushers[i]->getSize();
    return maxIndex;
}
