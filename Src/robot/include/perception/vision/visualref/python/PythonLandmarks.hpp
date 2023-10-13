#pragma once

#include <Python.h>
#include <boost/python.hpp>
#include <blackboard/Adapter.hpp>

#include <blackboard/Blackboard.hpp>

class PythonLandmarks
{
public:
    PythonLandmarks();
    ~PythonLandmarks();

    void execute();

private:
    void startPython();
    void handlePyError(const boost::python::error_already_set &ex);

    bool errorOccured;

    const char *visualRefModuleName;
    const char *robotModuleName;

    boost::python::object mainModule;
    boost::python::object sysModule;

    boost::python::object initialModules;
    boost::python::object visualRefModule;
    boost::python::object visualRefTick;
    boost::python::object pyKeyboardInterrupt;
};