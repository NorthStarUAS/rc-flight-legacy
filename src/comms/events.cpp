#include "events.h"

pyModuleEventLog::pyModuleEventLog()
{
}

bool pyModuleEventLog::log(const char *header, const char *message)
{
    if (pModuleObj == NULL) {
	printf("ERROR: events.init() failed\n");
	return false;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "log");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'log()'\n");
	return false;
    }

    // FIXME decref pFuncLog
    
    PyObject *pValue = PyObject_CallFunction(pFuncLog, (char *)"ss", header, message);
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
