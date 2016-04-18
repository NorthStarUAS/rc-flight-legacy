#include "packer.hxx"

pyModulePacker::pyModulePacker()
{
}

int pyModulePacker::pack_gps_v1(int index, uint8_t *buf) {
    int len = 0;
    if (pModuleObj == NULL) {
	printf("ERROR: import packer failed\n");
	return 0;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "pack_gps_v1");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'pack_gps_v1()'\n");
	return 0;
    }
    PyObject *pResult = PyObject_CallFunction(pFuncLog, (char *)"i", index);
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
	return 0;
    }
    return 0;
}
