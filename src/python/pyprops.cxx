/**
 * C++ interface to a python PropertyNode()
 */

#include <Python.h>
#include <string>
using std::string;

#include "pyprops.hxx"


// Default constructor.
pyPropertyNode::pyPropertyNode():
    pObj(NULL)
{
}

pyPropertyNode::pyPropertyNode(PyObject *p)
{
    pObj = p;
}

// Destructor.
pyPropertyNode::~pyPropertyNode() {
    Py_XDECREF(pObj);
}

// testers
bool pyPropertyNode::isBranch() {
    string type = (string)pObj->ob_type->tp_name;
    if ( type == "instance" || type == "list" ) {
	return true;
    } else {
	return false;
    }
}

bool pyPropertyNode::isLeaf() {
    string type = (string)pObj->ob_type->tp_name;
    if ( type == "instance" || type == "list" ) {
	return false;
    } else {
	return true;
    }
}

// getters
double pyPropertyNode::getDoubleValue() {
    double result = 0.0;
    if ( pObj != NULL ) {
	if ( PyFloat_Check(pObj) ) {
	    result = PyFloat_AsDouble(pObj);
	} else if ( PyInt_Check(pObj) ) {
	    result = PyInt_AsLong(pObj);
	} else if ( PyLong_Check(pObj) ) {
	    result = PyLong_AsLong(pObj);
	} else if ( PyString_Check(pObj) ) {
	    PyObject *pFloat = PyFloat_FromString(pObj, NULL);
	    result = PyFloat_AsDouble(pFloat);
	    Py_DECREF(pFloat);
	} else {
	    printf("Unknown object type: '%s' ", pObj->ob_type->tp_name);
	    PyObject *pStr = PyObject_Str(pObj);
	    char *s = PyString_AsString(pStr);
	    printf("%s\n", s);
	    Py_DECREF(pStr);
	}
    }
    return result;
}

long pyPropertyNode::getLongValue() {
    long result = 0;
    if ( pObj != NULL ) {
	if ( PyLong_Check(pObj) ) {
	    result = PyLong_AsLong(pObj);
	} else if ( PyInt_Check(pObj) ) {
	    result = PyInt_AsLong(pObj);
	} else if ( PyFloat_Check(pObj) ) {
	    result = (long)PyFloat_AsDouble(pObj);
	} else if ( PyString_Check(pObj) ) {
	    PyObject *pFloat = PyFloat_FromString(pObj, NULL);
	    result = (long)PyFloat_AsDouble(pFloat);
	    Py_DECREF(pFloat);
	} else {
	    printf("Unknown object type: '%s' ", pObj->ob_type->tp_name);
	    PyObject *pStr = PyObject_Str(pObj);
	    char *s = PyString_AsString(pStr);
	    printf("%s\n", s);
	    Py_DECREF(pStr);
	}
    }
    return result;
}

bool pyPropertyNode::getBoolValue() {
    bool result = false;
    if ( pObj != NULL ) {
	result = PyObject_IsTrue(pObj);
    }
    return result;
}

string pyPropertyNode::getStringValue() {
    string result = "";
    if ( pObj != NULL ) {
	PyObject *pStr = PyObject_Str(pObj);
	result = (string)PyString_AsString(pStr);
    }
    return result;
}


// These only need to be looked up once and then saved
static PyObject *pModuleProps = NULL;
static PyObject *pFuncGetNode = NULL;

// This function must be called first (before any pyPropertyNode
// usage.) It sets up the python intepreter and imports the python
// props module.
void pyPropsInit(int argc, char **argv) {
    Py_SetProgramName(argv[0]);	// optional but recommended
    Py_Initialize();
    PySys_SetArgv(argc, argv);	// for relative imports to work

    // python property system
    pModuleProps = PyImport_ImportModule("props");
    if (pModuleProps == NULL) {
        PyErr_Print();
        fprintf(stderr, "Failed to load 'props'\n");
    }
    // getNode() function
    pFuncGetNode = PyObject_GetAttrString(pModuleProps, "getNode");
    if ( pFuncGetNode == NULL || ! PyCallable_Check(pFuncGetNode) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	fprintf(stderr, "Cannot find function 'getNode()'\n");
    }
}

// This function can be called prior to exit (after the last property
// node usage) to properly shutdown and clean up the python
// interpreter.
extern void pyPropsClose() {
    Py_XDECREF(pFuncGetNode);
    Py_XDECREF(pModuleProps);
    Py_Finalize();
}

// Return a pyPropertyNode object that points to the specified path in
// the property tree.  This is a 'heavier' operation so it is
// recommended to call this function from initialization routines and
// save the result.  Then use the pyPropertyNode for direct read/write
// access in your update routines.
pyPropertyNode pyGetNode(string abs_path, bool create) {
    PyObject *pArgs = PyTuple_New(2);
    PyObject *pPath = PyString_FromString(abs_path.c_str());
    PyObject *pCreate = PyBool_FromLong(create);
    if (!pPath || !pCreate) {
	Py_DECREF(pArgs);
	Py_XDECREF(pPath);
	Py_XDECREF(pCreate);
	fprintf(stderr, "Cannot convert argument\n");
	return pyPropertyNode();
    }
    PyTuple_SetItem(pArgs, 0, pPath);
    PyTuple_SetItem(pArgs, 1, pCreate);
    PyObject *pValue = PyObject_CallObject(pFuncGetNode, pArgs);
    Py_DECREF(pArgs);
    if (pValue != NULL) {
	// give pValue over to the returned property node
	return pyPropertyNode(pValue);
    } else {
	PyErr_Print();
	fprintf(stderr,"Call failed\n");
	return pyPropertyNode();
    }
    return pyPropertyNode();
}

