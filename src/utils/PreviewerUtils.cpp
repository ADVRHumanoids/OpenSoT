#include <OpenSoT/utils/PreviewerUtils.h>
#include <Python.h>

boost::shared_ptr<OpenSoT::PreviewerUtils::PyRunner>
    OpenSoT::PreviewerUtils::PyRunner::instance(
        new OpenSoT::PreviewerUtils::PyRunner());

OpenSoT::PreviewerUtils::PyRunner::PyRunner()
{Py_Initialize();}

OpenSoT::PreviewerUtils::PyRunner::~PyRunner()
{Py_Finalize();}

boost::shared_ptr<OpenSoT::PreviewerUtils::PyRunner> OpenSoT::PreviewerUtils::PyRunner::getInstance()
{
    if(!PyRunner::instance)
        PyRunner::instance.reset(new PyRunner);
    return PyRunner::instance;
}

bool OpenSoT::PreviewerUtils::PyRunner::run(string script)
{
    PyRun_SimpleString(script.c_str());
    return true;
}

bool OpenSoT::PreviewerUtils::PyRunner::numpyRun(string script)
{
    std::ostringstream _src;
    if(!OpenSoT::PreviewerUtils::PyRunner::getInstance()->imported("numpy"))
    {
        _src << "#! /usr/bin/env python" << std::endl
             << std::endl
             << "import numpy as np" << std::endl
             << "import matplotlib" << std::endl
             << "from matplotlib.pyplot import *" << std::endl;
    }
    _src << script;
    this->run(_src.str());
    return true;
}

void OpenSoT::PreviewerUtils::PyRunner::setImported(const string module)
{
    importedModules[module] = true;
}

bool OpenSoT::PreviewerUtils::PyRunner::imported(const string module)
{
    return OpenSoT::PreviewerUtils::PyRunner::importedModules.count(module) > 0;
}


bool OpenSoT::PreviewerUtils::plotPreviewerTrajectory(OpenSoT::previewer::Results &results) {
    typedef OpenSoT::previewer::Results::LogEntry LogEntry;

    if(results.log.size() > 0)
    {
        std::ostringstream _src;

        _src << "t_q = np.array((";
        for(typename std::map<double, LogEntry>::iterator it =
            results.log.begin();
            it != results.log.end();
            ++it)
        {
            std::string qString(it->second.q.toString());
            std::replace(qString.begin(), qString.end(), '\t', ',');
            _src << "(" << it->first << "," << qString << ")";
            typename std::map<double, LogEntry>::iterator it_next = it; ++it_next;
            if(it_next != results.log.end())
                _src << "," << std::endl;
        }
        _src << "))" << std::endl;

        _src << "t_dq = np.array((";
        for(typename std::map<double, LogEntry>::iterator it =
            results.log.begin();
            it != results.log.end();
            ++it)
        {
            typename std::map<double, LogEntry>::iterator it_next = it; ++it_next;
            if(it_next != results.log.end())
            {
                using namespace yarp::math;
                std::string dqString((it_next->second.q - it->second.q).toString());
                std::replace(dqString.begin(), dqString.end(), '\t', ',');
                _src << "(" << it_next->first << "," << dqString << "),";
            }
        }
        _src << "))" << std::endl;

        _src << "q_plot = figure(figsize=(8,6)); p_q = plot(t_q[:,0],t_q[:,1:]); title('Joint Positions')" << std::endl;
        _src << "dq_plot = figure(figsize=(8,6)); p_dq = plot(t_dq[:,0],t_dq[:,1:]); title('Joint Velocities')" << std::endl;
        _src << "show(block=True)" << std::endl;

        return PyRunner::getInstance()->numpyRun(_src.str());
    } return false;
}


bool OpenSoT::PreviewerUtils::plotPreviewerErrors(OpenSoT::previewer::Results &results)
{
    typedef OpenSoT::previewer::Results::LogEntry LogEntry;
    typedef typename LogEntry::CartesianError CartesianError;
    typedef OpenSoT::Task<yarp::sig::Matrix,yarp::sig::Vector> Task;
    typedef map< Task*, CartesianError > ErrorsMap;

    if(results.log.size() > 0)
    {
        std::ostringstream _src;
        boost::hash<std::string> string_hash;

        for(typename ErrorsMap::iterator it_task =
            results.log.begin()->second.errors.begin();
            it_task != results.log.begin()->second.errors.end();
            ++it_task)
        {
            Task* task = it_task->first;
            std::string taskName = task->getTaskID();
            std::size_t taskHash = string_hash(taskName);

            _src << "t_" << taskHash << "_err = np.array((";

            for(typename std::map<double, LogEntry>::iterator it_time =
                results.log.begin();
                it_time != results.log.end();
                ++it_time)
            {
                double time = it_time->first;
                CartesianError error = it_time->second.errors[task];

                std::string errString(yarp::math::cat(error.positionError, error.orientationError).toString());
                std::replace(errString.begin(), errString.end(), '\t', ',');
                _src << "(" << time << "," << errString << ")";
                typename std::map<double, LogEntry>::iterator it_time_next = it_time; ++it_time_next;
                if(it_time_next != results.log.end())
                    _src << "," << std::endl;
            }
            _src << "))" << std::endl;

            _src << "figure(figsize=(8,6)); pos_err = plot(t_" << taskHash << "_err[:,0],t_" << taskHash << "_err[:,1:4]); title('Task " << taskName << " Position Errors'); legend(pos_err,('x', 'y', 'z'))" << std::endl;
            if(dynamic_cast<OpenSoT::tasks::velocity::Cartesian*>(task))
                _src << "figure(figsize=(8,6)); orient_err = plot(t_" << taskHash << "_err[:,0],t_" << taskHash << "_err[:,4:7]); title('Task " << taskName << " Orientation Errors'); legend(orient_err,('x', 'y', 'z'))" << std::endl;
        }

        _src << "show(block=True)" << std::endl;

        return PyRunner::getInstance()->numpyRun(_src.str());
    }
    return false;
}
