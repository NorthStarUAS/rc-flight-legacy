/**
 * C++ interface to a python PropertyNode()
 */

#include <Python.h>
#include <string>
using std::string;

#include "pyprops.hxx"


static PyObject *pModuleProps = NULL;


// This function must be called first (before any pyPropertyNode
// usage) to setup the python intepreter and import the python props
// module
void pyPropsInit(int argc, char **argv) {
    Py_SetProgramName(argv[0]);	// optional but recommended
    Py_Initialize();
    PySys_SetArgv(argc, argv);	// for relative imports to work

    // python property system
    string module_name = "props";
    PyObject *pModuleProps;
    pModuleProps = PyImport_ImportModule(module_name.c_str());
    if (pModuleProps == NULL) {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", module_name.c_str());
    }
}

// This function can be called prior to exit (after the last property
// node usage) to clean up the python interpreter
extern void pyPropsClose() {
    Py_XDECREF(pModuleProps);
    Py_Finalize();
}


// Default constructor.
pyPropertyNode::pyPropertyNode()
{
}

// Destructor.
pyPropertyNode::~pyPropertyNode() {
}

