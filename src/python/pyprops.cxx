/**
 * C++ interface to a python PropertyNode()
 */

#include <Python.h>
#include <string>
using std::string;

#include "pyprops.hxx"


// Constructor

pyPropertyNode::pyPropertyNode(PyObject *p)
{
    pObj = p;
}

// Destructor.
pyPropertyNode::~pyPropertyNode() {
    printf("~pyPropertyNode destructor\n");
    Py_XDECREF(pObj);
}

// getters
double pyPropertyNode::getDouble(const char *name) {
    double result = 0.0;
    if ( pObj != NULL ) {
	if ( PyObject_HasAttrString(pObj, name) ) {
	    PyObject *pAttr = PyObject_GetAttrString(pObj, name);
	    if ( pAttr != NULL ) {
		if ( PyFloat_Check(pAttr) ) {
		    result = PyFloat_AsDouble(pAttr);
		} else if ( PyInt_Check(pAttr) ) {
		    result = PyInt_AsLong(pAttr);
		} else if ( PyLong_Check(pAttr) ) {
		    result = PyLong_AsLong(pAttr);
		} else if ( PyString_Check(pAttr) ) {
		    PyObject *pFloat = PyFloat_FromString(pAttr, NULL);
		    result = PyFloat_AsDouble(pFloat);
		    Py_DECREF(pFloat);
		} else {
		    printf("Unknown object type: '%s' ", pObj->ob_type->tp_name);
		    PyObject *pStr = PyObject_Str(pObj);
		    char *s = PyString_AsString(pStr);
		    printf("%s\n", s);
		    Py_DECREF(pStr);
		}
		//Py_DECREF(pAttr);
	    }
	}
    }
    return result;
}

long pyPropertyNode::getLong(const char *name) {
    long result = 0;
    if ( pObj != NULL ) {
	if ( PyObject_HasAttrString(pObj, name) ) {
	    PyObject *pAttr = PyObject_GetAttrString(pObj, name);
	    if ( pAttr != NULL ) {
		if ( PyLong_Check(pAttr) ) {
		    result = PyLong_AsLong(pAttr);
		} else if ( PyInt_Check(pAttr) ) {
		    result = PyInt_AsLong(pAttr);
		} else if ( PyFloat_Check(pAttr) ) {
		    result = (long)PyFloat_AsDouble(pAttr);
		} else if ( PyString_Check(pAttr) ) {
		    PyObject *pFloat = PyFloat_FromString(pAttr, NULL);
		    result = (long)PyFloat_AsDouble(pFloat);
		    Py_DECREF(pFloat);
		} else {
		    printf("Unknown object type: '%s' ", pAttr->ob_type->tp_name);
		    PyObject *pStr = PyObject_Str(pAttr);
		    char *s = PyString_AsString(pStr);
		    printf("%s\n", s);
		    Py_DECREF(pStr);
		}
		Py_DECREF(pAttr);
	    }
	}
    }
    return result;
}

bool pyPropertyNode::getBool(const char *name) {
    bool result = false;
    if ( pObj != NULL ) {
	if ( PyObject_HasAttrString(pObj, name) ) {
	    PyObject *pAttr = PyObject_GetAttrString(pObj, name);
	    if ( pAttr != NULL ) {
		result = PyObject_IsTrue(pObj);
		Py_DECREF(pAttr);
	    }
	}
    }
    return result;
}

string pyPropertyNode::getString(const char *name) {
    string result = "";
    if ( pObj != NULL ) {
	if ( PyObject_HasAttrString(pObj, name) ) {
	    PyObject *pAttr = PyObject_GetAttrString(pObj, name);
	    if ( pAttr != NULL ) {
		PyObject *pStr = PyObject_Str(pObj);
		result = (string)PyString_AsString(pStr);
		Py_DECREF(pAttr);
	    }
	}
    }
    return result;
}

// setters
bool pyPropertyNode::setDouble( const char *name, double val ) {
    if ( pObj != NULL ) {
	PyObject *pFloat = PyFloat_FromDouble(val);
	int result = PyObject_SetAttrString(pObj, name, pFloat);
	Py_DECREF(pFloat);
	return result != -1;
    } else {
	return false;
    }
}

bool pyPropertyNode::setLong( const char *name, long val ) {
    if ( pObj != NULL ) {
	PyObject *pLong = PyLong_FromLong(val);
	int result = PyObject_SetAttrString(pObj, name, pLong);
	Py_DECREF(pLong);
	return result != -1;
    } else {
	return false;
    }
}

bool pyPropertyNode::setBool( const char *name, bool val ) {
    if ( pObj != NULL ) {
	PyObject *pBool = PyBool_FromLong((long)val);
	int result = PyObject_SetAttrString(pObj, name, pBool);
	Py_DECREF(pBool);
	return result != -1;
    } else {
	return false;
    }
}

bool pyPropertyNode::setString( const char *name, string val ) {
    if ( pObj != NULL ) {
	PyObject *pString = PyString_FromString(val.c_str());
	int result = PyObject_SetAttrString(pObj, name, pString);
	Py_DECREF(pString);
	return result != -1;
    } else {
	return false;
    }
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
extern void pyPropsCleanup(void) {
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
	return pyPropertyNode(NULL);
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
	return pyPropertyNode(NULL);
    }
    return pyPropertyNode(NULL);
}

