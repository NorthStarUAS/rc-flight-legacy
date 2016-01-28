#include "pymodule.hxx"

pyModuleBase::pyModuleBase():
    pModuleObj(NULL)
{
}

pyModuleBase::~pyModuleBase()
{
    if ( pModuleObj != NULL ) {
	Py_XDECREF(pModuleObj);
    }
}

bool pyModuleBase::init(const char *import_name)
{
    pModuleObj = PyImport_ImportModule(import_name);
    if (pModuleObj == NULL) {
        PyErr_Print();
        fprintf(stderr, "Failed to load '%s'\n", import_name);
	return false;
    }

    PyObject *pFuncInit = PyObject_GetAttrString(pModuleObj, "init");
    if ( pFuncInit == NULL || ! PyCallable_Check(pFuncInit) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	fprintf(stderr, "Cannot find function '%s.init()'\n", import_name);
	Py_DECREF(pModuleObj);
	pModuleObj = NULL;
	return false;
    }

    // FIXME decref pFuncInit
    
    PyObject *pValue = PyObject_CallFunction(pFuncInit, NULL);
    if (pValue != NULL) {
	double result = PyObject_IsTrue(pValue);
	Py_DECREF(pValue);
	return result;
    } else {
	PyErr_Print();
	printf("init() failed\n");
	return false;
    }
    return false;
}


bool pyModuleBase::update() {
    if (pModuleObj == NULL) {
	fprintf(stderr, "ERROR: mission_mgr.init() failed\n");
	return false;
    }
    PyObject *pFuncUpdate = PyObject_GetAttrString(pModuleObj, "update");
    if ( pFuncUpdate == NULL || ! PyCallable_Check(pFuncUpdate) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	fprintf(stderr, "Cannot find function 'update()'\n");
	return false;
    }

    // FIXME decref pFuncUpdate
    
    PyObject *pValue = PyObject_CallFunction(pFuncUpdate, NULL);
    if (pValue != NULL) {
	double result = PyObject_IsTrue(pValue);
	Py_DECREF(pValue);
	return result;
    } else {
	PyErr_Print();
	printf("Call failed\n");
	return false;
    }
    return false;
}
