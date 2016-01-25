/**
 * C++ interface to a python PropertyNode()
 */

#include <Python.h>
#include <string>
#include <sstream>
using std::string;
using std::ostringstream;

#include "pyprops.hxx"


// Constructor

pyPropertyNode::pyPropertyNode()
{
    // printf("pyPropertyNode()\n");
    pObj = NULL;
}

pyPropertyNode::pyPropertyNode(const pyPropertyNode &node)
{
    // printf("visiting pyPropertyNode() copy constructor!\n");
    pObj = node.pObj;
    Py_INCREF(pObj);
}

pyPropertyNode::pyPropertyNode(PyObject *p)
{
    // printf("pyPropertyNode(pObj)\n");
    pObj = p;
}

// Destructor.
pyPropertyNode::~pyPropertyNode() {
    // printf("~pyPropertyNode destructor\n");
    if ( pObj == NULL ) {
	printf("WARNING: calling destructor on null pyPropertyNode\n");
	Py_DECREF(pObj);
    }
    Py_XDECREF(pObj);
    pObj = NULL;
}

// Assignment operator
pyPropertyNode & pyPropertyNode::operator= (const pyPropertyNode &node) {
    // printf("Visiting pyPropertyNode operator=\n");
    if (this != &node) {
	// protect against invalid self-assignment

	if ( pObj != NULL ) {
	    // 1: decrement current pObj reference because we are losing it
	    // printf("decrementing existing pObj before overwritting it\n");
	    Py_DECREF(pObj);
	}

	// 2: copy new value to pObj
	pObj = node.pObj;

	// 3: increment pObj ref count because we just created another
	// reference to it.
	Py_INCREF(pObj);
    }
    // by convention, always return *this
    return *this;
}

// test if pObj has named child attribute
bool pyPropertyNode::hasChild(const char *name) {
    if ( pObj != NULL ) {
	if ( PyObject_HasAttrString(pObj, name) ) {
	    return true;
	}
    }
    return false;
}

// Return a pyPropertyNode object that points to the named child
pyPropertyNode pyPropertyNode::getChild(const char *name, bool create)
{
    if ( pObj == NULL ) {
	return pyPropertyNode();
    }
    PyObject *pValue = PyObject_CallMethod(pObj,
					   (char *)"getChild", (char *)"sb",
					   name, create);
    if (pValue == NULL) {
	PyErr_Print();
	fprintf(stderr,"Call failed\n");
	return pyPropertyNode();
    }

    // give pValue over to the returned property node
    return pyPropertyNode(pValue);
}

pyPropertyNode pyPropertyNode::getChild(const char *name, int index,
					bool create)
{
    if ( pObj == NULL ) {
	return pyPropertyNode();
    }
    ostringstream str;
    str << name << '[' << index << ']';
    string ename = str.str();
    printf("ename = %s\n", ename.c_str());
    return getChild(ename.c_str(), create);
}

// return true if pObj pointer is NULL
bool pyPropertyNode::isNull() {
    return pObj == NULL;
}    

// return true if pObj is a list (enumerated)
int pyPropertyNode::getLen(const char *name) {
    if ( pObj != NULL ) {
	PyObject *pValue = PyObject_CallMethod(pObj,
					       (char *)"getLen", (char *)"s",
					       name);
	if ( pValue != NULL ) {
	    int len = PyInt_AsLong(pValue);
	    Py_DECREF(pValue);
	    return len;
	}
    }
    return 0;
}    

// return true if pObj is a list (enumerated)
void pyPropertyNode::setLen(const char *name, int size) {
    if ( pObj != NULL ) {
	PyObject *pValue = PyObject_CallMethod(pObj,
					       (char *)"setLen", (char *)"si",
					       name, size);
	if ( pValue != NULL ) {
	    Py_DECREF(pValue);
	}
    }
}    

// return true if pObj is a list (enumerated)
void pyPropertyNode::setLen(const char *name, int size, double init_val) {
    if ( pObj != NULL ) {
	PyObject *pValue = PyObject_CallMethod(pObj,
					       (char *)"setLen", (char *)"sif",
					       name, size, init_val);
	if ( pValue != NULL ) {
	    Py_DECREF(pValue);
	}
    }
}    

// return true if pObj is a list (enumerated)
vector <string> pyPropertyNode::getChildren() {
    vector <string> result;
    if ( pObj != NULL ) {
	PyObject *pList = PyObject_CallMethod(pObj,
					      (char *)"getChildren",
					      (char *)"");
	if ( pList != NULL ) {
	    if ( PyList_Check(pList) ) {
		int len = PyList_Size(pList);
		for ( int i = 0; i < len; i++ ) {
		    PyObject *pItem = PyList_GetItem(pList, i);
		    // note: PyList_GetItem doesn't give us ownership
		    // of pItem so we should not decref() it.
		    PyObject *pStr = PyObject_Str(pItem);
		    result.push_back( (string)PyString_AsString(pStr) );
		    Py_DECREF(pStr);
		}
	    }
	    Py_DECREF(pList);
	}
    }
    return result;
}    

// return true if pObj/name is leaf
bool pyPropertyNode::isLeaf(const char *name) {
    if ( pObj == NULL ) {
	return false;
    }
    PyObject *pValue = PyObject_CallMethod(pObj,
					   (char *)"isLeaf", (char *)"s",
					   name);
    bool result = PyObject_IsTrue(pValue);
    Py_DECREF(pValue);
    return result;
}    

// note: expects the calling layer to Py_DECREF(pAttr)
double pyPropertyNode::PyObject2Double(const char *name, PyObject *pAttr) {
    double result = 0.0;
    if ( pAttr != NULL ) {
	if ( PyFloat_Check(pAttr) ) {
	    result = PyFloat_AsDouble(pAttr);
	} else if ( PyInt_Check(pAttr) ) {
	    result = PyInt_AsLong(pAttr);
	} else if ( PyLong_Check(pAttr) ) {
	    result = PyLong_AsLong(pAttr);
	} else if ( PyString_Check(pAttr) ) {
	    PyObject *pFloat = PyFloat_FromString(pAttr, NULL);
	    if ( pFloat != NULL ) {
		result = PyFloat_AsDouble(pFloat);
		Py_DECREF(pFloat);
	    } else {
		PyErr_Print();
		printf("WARNING: conversion from string to float failed\n");
		PyObject *pStr = PyObject_Str(pAttr);
		char *s = PyString_AsString(pStr);
		printf("  %s='%s'\n", name, s);
		Py_DECREF(pStr);
	    }
	} else {
	    printf("Unknown object type: '%s' ", pObj->ob_type->tp_name);
	    PyObject *pStr = PyObject_Str(pObj);
	    char *s = PyString_AsString(pStr);
	    printf("  %s='%s'\n", name, s);
	    Py_DECREF(pStr);
	}
    }
    return result;
}

// note: expects the calling layer to Py_DECREF(pAttr)
long pyPropertyNode::PyObject2Long(const char *name, PyObject *pAttr) {
    long result = 0;
    if ( pAttr != NULL ) {
	if ( PyLong_Check(pAttr) ) {
	    result = PyLong_AsLong(pAttr);
	} else if ( PyInt_Check(pAttr) ) {
	    result = PyInt_AsLong(pAttr);
	} else if ( PyFloat_Check(pAttr) ) {
	    result = (long)PyFloat_AsDouble(pAttr);
	} else if ( PyString_Check(pAttr) ) {
	    PyObject *pFloat = PyFloat_FromString(pAttr, NULL);
	    if ( pFloat != NULL ) {
		result = PyFloat_AsDouble(pFloat);
		Py_DECREF(pFloat);
	    } else {
		PyErr_Print();
		printf("WARNING: conversion from string to long failed\n");
		PyObject *pStr = PyObject_Str(pAttr);
		char *s = PyString_AsString(pStr);
		printf("  %s='%s'\n", name, s);
		Py_DECREF(pStr);
	    }
	} else {
	    printf("Unknown object type: '%s' ", pAttr->ob_type->tp_name);
	    PyObject *pStr = PyObject_Str(pAttr);
	    char *s = PyString_AsString(pStr);
	    printf("  %s='%s'\n", name, s);
	    Py_DECREF(pStr);
	}
    }
    return result;
}

// value getters
double pyPropertyNode::getDouble(const char *name) {
    double result = 0.0;
    if ( pObj != NULL ) {
	PyObject *pAttr = PyObject_GetAttrString(pObj, name);
	if ( pAttr != NULL ) {
	    result = PyObject2Double(name, pAttr);
	    Py_DECREF(pAttr);
	} else {
	    // printf("WARNING: request non-existent attr: %s\n", name);
	}
    }
    return result;
}

long pyPropertyNode::getLong(const char *name) {
    long result = 0;
    if ( pObj != NULL ) {
	PyObject *pAttr = PyObject_GetAttrString(pObj, name);
	if ( pAttr != NULL ) {
	    result = PyObject2Long(name, pAttr);
	    Py_DECREF(pAttr);
	} else {
	    // printf("WARNING: request non-existent attr: %s\n", name);
	}
    }
    return result;
}

bool pyPropertyNode::getBool(const char *name) {
    bool result = false;
    if ( pObj != NULL ) {
	PyObject *pAttr = PyObject_GetAttrString(pObj, name);
	if ( pAttr != NULL ) {
	    result = PyObject_IsTrue(pAttr);
	    Py_DECREF(pAttr);
	} else {
	    // printf("WARNING: request non-existent attr: %s\n", name);
	}
    }
    return result;
}

string pyPropertyNode::getString(const char *name) {
    string result = "";
    if ( pObj != NULL ) {
	PyObject *pAttr = PyObject_GetAttrString(pObj, name);
	if ( pAttr != NULL ) {
	    PyObject *pStr = PyObject_Str(pAttr);
	    if ( pStr != NULL ) {
		result = (string)PyString_AsString(pStr);
		Py_DECREF(pStr);
	    }
	    Py_DECREF(pAttr);
	} else {
	    // printf("WARNING: request non-existent attr: %s\n", name);
	}
    }
    return result;
}

// indexed value getters
double pyPropertyNode::getDouble(const char *name, int index) {
    double result = 0.0;
    if ( pObj != NULL ) {
	PyObject *pList = PyObject_GetAttrString(pObj, name);
	if ( pList != NULL ) {
	    if ( PyList_Check(pList) ) {
		if ( index < PyList_Size(pList) ) {
		    PyObject *pAttr = PyList_GetItem(pList, index);
		    // note: PyList_GetItem doesn't give us ownership
		    // of pItem so we should not decref() it.
		    if ( pAttr != NULL ) {
			result = PyObject2Double(name, pAttr);
		    }
		}
	    } else {
		printf("WARNING: request indexed value of plain node: %s!\n", name);
	    }
	    Py_DECREF(pList);
	} else {
	    // printf("WARNING: request non-existent attr: %s[%d]\n", name, index);
	}
    }
    return result;
}

long pyPropertyNode::getLong(const char *name, int index) {
    long result = 0;
    if ( pObj != NULL ) {
	PyObject *pList = PyObject_GetAttrString(pObj, name);
	if ( pList != NULL ) {
	    if ( PyList_Check(pList) ) {
		if ( index < PyList_Size(pList) ) {
		    PyObject *pAttr = PyList_GetItem(pList, index);
		    // note: PyList_GetItem doesn't give us ownership
		    // of pItem so we should not decref() it.
		    if ( pAttr != NULL ) {
			result = PyObject2Long(name, pAttr);
		    }
		}
	    } else {
		printf("WARNING: request indexed value of plain node: %s!\n", name);
	    }
	    Py_DECREF(pList);
	} else {
	    // printf("WARNING: request non-existent attr: %s[%d]\n", name, index);
	}
    }
    return result;
}

// value setters
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

// indexed value setters
bool pyPropertyNode::setDouble( const char *name, int index, double val ) {
    if ( pObj != NULL ) {
	PyObject *pList = PyObject_GetAttrString(pObj, name);
	if ( pList != NULL ) {
	    if ( PyList_Check(pList) ) {
		if ( index < PyList_Size(pList) ) {
		    PyObject *pFloat = PyFloat_FromDouble(val);
		    // note setitem() steals the reference so we can't
		    // decrement it
		    PyList_SetItem(pList, index, pFloat );
		} else {
		    // index out of range
		}
	    } else {
		// not a list
	    }
	    Py_DECREF(pList);
	} else {
	    // list lookup failed
	}
    } else {
	return false;
    }
    return true;
}

// Return a pyPropertyNode object that points to the named child
void pyPropertyNode::pretty_print()
{
    if ( pObj == NULL ) {
	printf("pretty_print(): Null pyPropertyNode()\n");
    } else {
	PyObject *pValue = PyObject_CallMethod(pObj,
					       (char *)"pretty_print",
					       (char *)"");
	if (pValue != NULL) {
	    Py_DECREF(pValue);
	} else {
	    PyErr_Print();
	    fprintf(stderr,"Call failed\n");
	}
    }
}

// These only need to be looked up once and then saved
static PyObject *pModuleProps = NULL;
static PyObject *pModuleXML = NULL;

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

    // xml I/O system
    pModuleXML = PyImport_ImportModule("props_xml");
    if (pModuleXML == NULL) {
        PyErr_Print();
        fprintf(stderr, "Failed to load 'props_xml'\n");
    }

}

// This function can be called prior to exit (after the last property
// node usage) to properly shutdown and clean up the python
// interpreter.
extern void pyPropsCleanup(void) {
    printf("running pyPropsCleanup()\n");
    Py_XDECREF(pModuleProps);
    Py_XDECREF(pModuleXML);
    Py_Finalize();
}

// Return a pyPropertyNode object that points to the specified path in
// the property tree.  This is a 'heavier' operation so it is
// recommended to call this function from initialization routines and
// save the result.  Then use the pyPropertyNode for direct read/write
// access in your update routines.
pyPropertyNode pyGetNode(string abs_path, bool create) {
    // python property system
    PyObject *pModuleProps = PyImport_ImportModule("props");
    if (pModuleProps == NULL) {
        PyErr_Print();
        fprintf(stderr, "Failed to load 'props'\n");
    }

    PyObject *pFuncGetNode = PyObject_GetAttrString(pModuleProps, "getNode");
    if ( pFuncGetNode == NULL || ! PyCallable_Check(pFuncGetNode) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	fprintf(stderr, "Cannot find function 'getNode()'\n");
    }

    // FIXME decref pModuleProp and pFuncGetNode
    
    PyObject *pPath = PyString_FromString(abs_path.c_str());
    PyObject *pCreate = PyBool_FromLong(create);
    if (!pPath || !pCreate) {
	Py_XDECREF(pPath);
	Py_XDECREF(pCreate);
	fprintf(stderr, "Cannot convert argument\n");
	return pyPropertyNode();
    }
    PyObject *pValue
	= PyObject_CallFunctionObjArgs(pFuncGetNode, pPath, pCreate, NULL);
    Py_DECREF(pPath);
    Py_DECREF(pCreate);
    if (pValue != NULL) {
	// give pValue over to the returned property node
	/*printf("pyGetNode() success, creating pyPropertyNode\n");
	pyPropertyNode tmp(pValue);
	printf("before return\n");*/
	return pyPropertyNode(pValue);
    } else {
	PyErr_Print();
	printf("Call failed\n");
	return pyPropertyNode();
    }
    return pyPropertyNode();
}

#if 0
pyPropertyNode pyGetNodeORIG(string abs_path, bool create) {
    PyObject *pArgs = PyTuple_New(2);
    PyObject *pPath = PyString_FromString(abs_path.c_str());
    PyObject *pCreate = PyBool_FromLong(create);
    if (!pArgs || !pPath || !pCreate) {
	Py_XDECREF(pArgs);
	Py_XDECREF(pPath);
	Py_XDECREF(pCreate);
	fprintf(stderr, "Cannot convert argument\n");
	return pyPropertyNode();
    }
    PyTuple_SetItem(pArgs, 0, pPath);
    PyTuple_SetItem(pArgs, 1, pCreate);
    if ( PyCallable_Check(pFuncGetNode) ) {
	PyObject *pValue = PyObject_CallObject(pFuncGetNode, pArgs);
	Py_DECREF(pPath);
	Py_DECREF(pCreate);
	Py_DECREF(pArgs);
	if (pValue != NULL) {
	    // give pValue over to the returned property node
	    printf("pyGetNode() success, creating pyPropertyNode\n");
	    pyPropertyNode tmp(pValue);
	    printf("before return\n");
	    return tmp;
	} else {
	    PyErr_Print();
	    printf("Call failed\n");
	    return pyPropertyNode();
	}
	return pyPropertyNode();
    } else {
	printf("pFuncGetNode has become uncallable.\n");
    }
}
#endif

bool readXML(string filename, pyPropertyNode *node) {
    // getNode() function
    PyObject *pFuncLoad = PyObject_GetAttrString(pModuleXML, "load");
    if ( pFuncLoad == NULL || ! PyCallable_Check(pFuncLoad) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	fprintf(stderr, "Cannot find function 'load()'\n");
	return false;
    }
    PyObject *pArgs = PyTuple_New(2);
    PyObject *pPath = PyString_FromString(filename.c_str());
    PyObject *pNode = node->pObj;
    if (!pPath || !pNode) {
	Py_DECREF(pArgs);
	Py_XDECREF(pPath);
	Py_XDECREF(pNode);
	Py_XDECREF(pFuncLoad);
	fprintf(stderr, "Cannot convert argument\n");
	return false;
    }
    PyTuple_SetItem(pArgs, 0, pPath);
    PyTuple_SetItem(pArgs, 1, pNode);
    PyObject *pValue = PyObject_CallObject(pFuncLoad, pArgs);
    Py_DECREF(pArgs);
    Py_DECREF(pFuncLoad);
    if (pValue != NULL) {
	// give pValue over to the returned property node
	bool result = PyObject_IsTrue(pValue);
	Py_DECREF(pValue);
	return result;
    } else {
	PyErr_Print();
	fprintf(stderr,"Call failed\n");
    }
    return false;
}

bool writeXML(string filename, pyPropertyNode *node) {
    // getNode() function
    PyObject *pFuncSave = PyObject_GetAttrString(pModuleXML, "save");
    if ( pFuncSave == NULL || ! PyCallable_Check(pFuncSave) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	fprintf(stderr, "Cannot find function 'save()'\n");
	return false;
    }
    PyObject *pArgs = PyTuple_New(2);
    PyObject *pPath = PyString_FromString(filename.c_str());
    PyObject *pNode = node->pObj;
    if (!pPath || !pNode) {
	Py_DECREF(pArgs);
	Py_XDECREF(pPath);
	Py_XDECREF(pNode);
	Py_XDECREF(pFuncSave);
	fprintf(stderr, "Cannot convert argument\n");
	return false;
    }
    PyTuple_SetItem(pArgs, 0, pPath);
    PyTuple_SetItem(pArgs, 1, pNode);
    PyObject *pValue = PyObject_CallObject(pFuncSave, pArgs);
    Py_DECREF(pArgs);
    Py_DECREF(pFuncSave);
    if (pValue != NULL) {
	// give pValue over to the returned property node
	bool result = PyObject_IsTrue(pValue);
	Py_DECREF(pValue);
	return result;
    } else {
	PyErr_Print();
	fprintf(stderr,"Call failed\n");
    }
    return false;
}

