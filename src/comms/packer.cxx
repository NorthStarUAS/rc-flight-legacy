#include "packer.hxx"

#include <string.h>		// memcpy()

pyModulePacker::pyModulePacker()
{
}

int pyModulePacker::pack(int index, const char *pack_function, uint8_t *buf) {
    int len = 0;
    if (pModuleObj == NULL) {
	printf("ERROR: import packer failed\n");
	return 0;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, pack_function);
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function '%s()'\n", pack_function);
	return 0;
    }
    PyObject *pResult = PyObject_CallFunction(pFuncLog, (char *)"i", index);
    if (pResult != NULL) {
	PyObject *pStr = PyObject_Str(pResult);
	if ( pStr != NULL ) {
	    char *ptr = PyString_AsString(pStr);
	    len = PyString_Size(pStr);
	    memcpy((char *)buf, ptr, len);
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

int pyModulePacker::pack_gps(int index, uint8_t *buf) {
    return pack(index, "pack_gps_v2", buf);
}

int pyModulePacker::pack_imu(int index, uint8_t *buf) {
    return pack(index, "pack_imu_v3", buf);
}

int pyModulePacker::pack_airdata(int index, uint8_t *buf) {
    return pack(index, "pack_airdata_v5", buf);
}
