#include "display.h"

bool display_on = false;

pyModuleDisplay::pyModuleDisplay()
{
}

bool pyModuleDisplay::show(const char *message)
{
    if (pModuleObj == NULL) {
	printf("ERROR: events.init() failed\n");
	return false;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "show");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'show()'\n");
	return false;
    }

    // FIXME decref pFuncLog
    
    PyObject *pValue = PyObject_CallFunction(pFuncLog, (char *)"s", message);
    if (pValue != NULL) {
	bool result = PyObject_IsTrue(pValue);
	Py_DECREF(pValue);
	return result;
    } else {
	PyErr_Print();
	printf("ERROR: call failed\n");
	return false;
    }
    return false;
}


void pyModuleDisplay::status_summary() {
    if (pModuleObj == NULL) {
	printf("ERROR: import logging module failed\n");
	return;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "status_summary");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'status_summary()'\n");
	return;
    }
    PyObject *pResult = PyObject_CallFunction(pFuncLog, NULL);
    if (pResult != NULL) {
	Py_DECREF(pResult);
	return;
    } else {
	PyErr_Print();
	printf("ERROR: call failed\n");
	return;
    }
}
