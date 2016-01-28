#include "mission.hxx"

// These only need to be looked up once and then saved
static PyObject *pModuleMission = NULL;

// This function must be called (once) before update()
bool pyMissionInit() {
    pModuleMission = PyImport_ImportModule("mission.mission_mgr");
    if (pModuleMission == NULL) {
        PyErr_Print();
        fprintf(stderr, "Failed to load 'mission_mgr'\n");
	return false;
    }

    PyObject *pFuncInit = PyObject_GetAttrString(pModuleMission, "init");
    if ( pFuncInit == NULL || ! PyCallable_Check(pFuncInit) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	fprintf(stderr, "Cannot find function 'mission_mgr.init()'\n");
	Py_DECREF(pModuleMission);
	pModuleMission = NULL;
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
	printf("Call failed\n");
	return false;
    }
    return false;
}

// This function must be called before update()
bool pyMissionUpdate() {
    if (pModuleMission == NULL) {
	fprintf(stderr, "ERROR: mission_mgr.init() failed\n");
	return false;
    }
    PyObject *pFuncUpdate = PyObject_GetAttrString(pModuleMission, "update");
    if ( pFuncUpdate == NULL || ! PyCallable_Check(pFuncUpdate) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	fprintf(stderr, "Cannot find function 'mission_mgr.update()'\n");
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
