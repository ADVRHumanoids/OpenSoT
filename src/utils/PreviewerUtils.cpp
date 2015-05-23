#include <OpenSoT/utils/PreviewerUtils.h>
#include <Python.h>

boost::shared_ptr<OpenSoT::PreviewerUtils::PyRunner>
    OpenSoT::PreviewerUtils::PyRunner::instance(
        new OpenSoT::PreviewerUtils::PyRunner());

OpenSoT::PreviewerUtils::PyRunner::PyRunner()
{}

OpenSoT::PreviewerUtils::PyRunner::~PyRunner()
{}

boost::shared_ptr<OpenSoT::PreviewerUtils::PyRunner> OpenSoT::PreviewerUtils::PyRunner::getInstance()
{
    if(PyRunner::instance)
        PyRunner::instance.reset(new PyRunner);
    return PyRunner::instance;
}

bool OpenSoT::PreviewerUtils::PyRunner::run(string script)
{
    Py_Initialize();
    PyRun_SimpleString(script.c_str());
    Py_Finalize();
    return true;
}

bool OpenSoT::PreviewerUtils::PyRunner::numpyRun(string script)
{
    std::ostringstream _src;
    _src << "#! /usr/bin/env python" << std::endl
         << std::endl
         << "import numpy as np" << std::endl
         << "import matplotlib" << std::endl
         << "from matplotlib.pyplot import *" << std::endl;
    _src << script;
    this->run(_src.str());
    return true;
}
