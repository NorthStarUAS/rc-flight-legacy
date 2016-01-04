/**
 * C++ interface to a python PropertyNode()
 */

#include <Python.h>
#include <string>
using std::string;

#include "pyprops.hxx"


static PyObject *pModuleProps = NULL;


// This function must be called first (before any pyPropertyNode
// usage.) It sets up the python intepreter and imports the python
// props module.
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
// node usage) to properly shutdown and clean up the python
// interpreter.
extern void pyPropsClose() {
    Py_XDECREF(pModuleProps);
    Py_Finalize();
}

// Return a pyPropertyNode object that points to the specified path in
// the property tree.  This is a 'heavier' operation so it is
// recommended to call this function from initialization routines and
// save the result.  Then use the pyPropertyNode for direct read/write
// access in your update routines.
pyPropertyNode pyGetNode(string abs_path, bool create) {
    return NULL;
}

// Default constructor.
pyPropertyNode::pyPropertyNode()
{
}

// Destructor.
pyPropertyNode::~pyPropertyNode() {
}
