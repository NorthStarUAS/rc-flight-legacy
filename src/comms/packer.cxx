#include "packer.hxx"

pyModulePacker::pyModulePacker()
{
}

#if 0
bool pyModulePacker::open(const char *path)
{
    if (pModuleObj == NULL) {
	printf("ERROR: events.init() failed\n");
	return false;
    }
    PyObject *pFuncOpen = PyObject_GetAttrString(pModuleObj, "open");
    if ( pFuncOpen == NULL || ! PyCallable_Check(pFuncOpen) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'update()'\n");
	return false;
    }

    // FIXME decref pFuncOpen
    
    PyObject *pValue = PyObject_CallFunction(pFuncOpen, (char *)"s", path);
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
#endif

#if 0
bool pyModulePacker::log(const char *header, const char *message)
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
#endif

#if 0
bool pyModulePacker::close()
{
    return true;
}
#endif

int pyModulePacker::pack_gps_v1(int index, uint8_t *buf) {
    int len = 0;
    if (pModuleObj == NULL) {
	printf("ERROR: import packer failed\n");
	return false;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "pack_gps_v1");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'pack_gps_v1()'\n");
	return false;
    }
    PyObject *pResult = PyObject_CallFunction(pFuncLog, (char *)"d", index);
    if (pResult != NULL) {
	PyObject *pStr = PyObject_Str(pResult);
	if ( pStr != NULL ) {
	    string result = (string)PyString_AsString(pStr);
	    len = PyString_Size(pStr);
	    strncpy((char *)buf, result.c_str(), len);
	    Py_DECREF(pStr);
	}
	Py_DECREF(pResult);
	return len;
    } else {
	PyErr_Print();
	printf("ERROR: call failed\n");
	return false;
    }
    return false;
}
