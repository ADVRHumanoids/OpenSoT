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
