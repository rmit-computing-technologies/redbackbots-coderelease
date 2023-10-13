#include "perception/vision/visualref/python/PythonLandmarks.hpp"

#include "thread/ThreadManager.hpp"

extern "C" 
void initrobot(void);

using namespace boost::python;

PythonLandmarks::PythonLandmarks()
{
    visualRefModuleName = "ref_pose";

    visualRefModule = object();
    visualRefTick = object();
}

PythonLandmarks::~PythonLandmarks()
{
    Py_Finalize();
}

void PythonLandmarks::startPython()
{
    errorOccured = false;

    // Kill any stray interpreters
    if (Py_IsInitialized())
    {
        Py_Finalize();
    }

    // Start interpreter
    Py_Initialize();

    // This try block is peculiar to the interpreter initialisation
    try
    {
        // Load robotmodule C extension module
        initrobot();

        // Get handle to the special main module and sys
        mainModule = import("__main__");
        sysModule = import("sys");

        // Add behaviour.path option value to python path
        // object sysPath = sysModule.attr("path");
        // sysPath.attr("append")(path);

        // Obtain KeyboardInterrupt exception
        object exceptionsModule = import("exceptions");
        pyKeyboardInterrupt = exceptionsModule.attr("KeyboardInterrupt");

        // Import robot
        // Implements 'import robot' at the top of each Python behaviour file
        object robotModule = import(robotModuleName);
    }
    catch (const error_already_set &ex)
    {
        handlePyError(ex);

        // If the initialisation fails, we are unable to handle the error
        llog(ERROR) << "Python failed to start." << std::endl;
        std::cout << "Python failed to start. Shutting Down." << std::endl;

        // Rethrowing the exception will simply result in perception restarting.
        // Init shutdown instead.
        attemptingShutdown = true;
        return;
    }
}

void PythonLandmarks::handlePyError(const error_already_set &ex)
{
    PyObject *pyException = PyErr_Occurred();
    if (pyException != NULL)
    {
        if (PyErr_GivenExceptionMatches(pyException, pyKeyboardInterrupt.ptr()))
        {
            handleSignals(SIGINT, NULL, NULL);
        }
        else
        {
            errorOccured = true;
        }
        PyErr_Print();
        if (Py_FlushLine())
        {
            PyErr_Clear();
        }
    }
    else
    {
        /* We received an exception but PyErr was clear. Since we don't know what
         * to do, throw the exception and likely cause the perception thead to be
         * restarted.
         */
        throw ex;
    }
}

void PythonLandmarks::execute()
{
}