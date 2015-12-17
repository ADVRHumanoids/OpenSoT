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
#include <stdexcept>

void OpenSoT::L::udpate(double t, const yarp::sig::Vector &q_dot)
{
    _current_log << "(" << t << ",";
    for(unsigned int i = 0; i < q_dot.size(); ++i)
        _current_log << q_dot[i] << ",";
    {
        typedef std::vector<flushers::Flusher::Ptr>::const_iterator it_f;

        for(it_f it = _flushers.begin(); it != _flushers.end(); ++it)
        {
            (*it)->updateSolution(q_dot);
            _current_log << *it;
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

    _current_log.close();
    _flushers.clear();
    _constraintFlushers.clear();
    _dataFlushers.clear();
    _taskFlushers.clear();

    return true;
}

OpenSoT::L::L(std::string loggerName, iDynUtils& model_, OpenSoT::L::logger_format format)
    : _name(loggerName), model(model_), _format(format),
      _n_dofs(model_.iDyn3_model.getNrOfDOFs()),
      _fakeFlusher_t(1),
      _fakeFlusher_dq(_n_dofs,1),
      plotter(new OpenSoT::plotters::Plotter(this))
{
    if(_format == FORMAT_PYTHON)
    {
        _collator.open((loggerName + ".py").c_str());
        _collator << "#! /usr/bin/env python" << std::endl;
        _collator << std::endl;
    }

    std::vector<std::string> descriptions;
    descriptions.push_back("time");
    _fakeFlusher_t.setDescription(descriptions);

    descriptions.clear();
    std::vector<std::string> joint_names = model.getJointNames();
    for(unsigned int i = 0; i < joint_names.size(); ++i)
        descriptions.push_back(joint_names[i] + " dq_opt");
    _fakeFlusher_dq.setDescription(descriptions);
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
                model));
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

OpenSoT::Indices OpenSoT::L::getGlobalIndices(OpenSoT::plotters::Plottable plottable)
{
    if(dynamic_cast<OpenSoT::flushers::FakeFlusher*>(plottable.first))
    {
        unsigned int offset =
            (dynamic_cast<OpenSoT::flushers::FakeFlusher*>(plottable.first))->getIndicesOffset();
        return (plottable.second).shift(offset);
    }

    unsigned int minIndex = 0;
    minIndex += _fakeFlusher_t.getSize()+_fakeFlusher_dq.getSize()-1;
    for(unsigned int i = 0; i < _flushers.size(); ++i)
    {
        if(_flushers[i].get() == plottable.first)
            plottable.second.shift(minIndex);
        else minIndex += _flushers[i]->getSize();
    }

    throw new std::invalid_argument("Error: could not find specified flusher");
}

unsigned int OpenSoT::L::getMaximumIndex()
{
    unsigned int maxIndex = 0;
    maxIndex += _fakeFlusher_t.getSize()+_fakeFlusher_dq.getSize()-1;
    for(unsigned int i = 0; i < _flushers.size(); ++i)
        maxIndex += _flushers[i]->getSize();
    return maxIndex;
}

unsigned int OpenSoT::L::isAppending()
{
    return _must_append[_current_log_filename];
}

OpenSoT::plotters::Plottable OpenSoT::L::t()
{
    return _fakeFlusher_t.i(OpenSoT::flushers::FakeFlusher::ALL);
}

OpenSoT::plotters::Plottable OpenSoT::L::dq_opt()
{
    return _fakeFlusher_dq.i(OpenSoT::flushers::FakeFlusher::ALL);
}
