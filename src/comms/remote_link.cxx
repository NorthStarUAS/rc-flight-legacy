#include "python/pyprops.hxx"

#include "remote_link.hxx"

pyModuleRemoteLink::pyModuleRemoteLink()
{
}

// bool pyModuleRemoteLink::open()
// {
//     if (pModuleObj == NULL) {
// 	printf("ERROR: remote_link.init() failed\n");
// 	return false;
//     }
//     PyObject *pFuncOpen = PyObject_GetAttrString(pModuleObj, "open");
//     if ( pFuncOpen == NULL || ! PyCallable_Check(pFuncOpen) ) {
// 	if ( PyErr_Occurred() ) PyErr_Print();
// 	printf("ERROR: cannot find function 'open()'\n");
// 	return false;
//     }

//     // FIXME decref pFuncOpen
    
//     PyObject *pValue = PyObject_CallFunction(pFuncOpen, NULL);
//     if (pValue != NULL) {
// 	bool result = PyObject_IsTrue(pValue);
// 	Py_DECREF(pValue);
// 	return result;
//     } else {
// 	PyErr_Print();
// 	printf("ERROR: call failed\n");
// 	return false;
//     }
//     return false;
// }

void pyModuleRemoteLink::send_message( uint8_t *buf, int size ) {
    if (pModuleObj == NULL) {
	printf("ERROR: import logging module failed\n");
	return;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "send_message");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'send_message()'\n");
	return;
    }
    PyObject *pResult = PyObject_CallFunction(pFuncLog, (char *)"s#", buf, size);
    if (pResult != NULL) {
	Py_DECREF(pResult);
	return;
    } else {
	PyErr_Print();
	printf("ERROR: call failed\n");
	return;
    }
}

bool pyModuleRemoteLink::command()
{
    if (pModuleObj == NULL) {
	printf("ERROR: remote_link.init() failed\n");
	return false;
    }
    PyObject *pFuncOpen = PyObject_GetAttrString(pModuleObj, "command");
    if ( pFuncOpen == NULL || ! PyCallable_Check(pFuncOpen) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'command()'\n");
	return false;
    }

    // FIXME decref pFuncOpen
    
    PyObject *pValue = PyObject_CallFunction(pFuncOpen, NULL);
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


bool pyModuleRemoteLink::flush_serial()
{
    if (pModuleObj == NULL) {
	printf("ERROR: remote_link.init() failed\n");
	return false;
    }
    PyObject *pFuncOpen = PyObject_GetAttrString(pModuleObj, "flush_serial");
    if ( pFuncOpen == NULL || ! PyCallable_Check(pFuncOpen) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'flush_serial()'\n");
	return false;
    }

    // FIXME decref pFuncOpen
    
    PyObject *pValue = PyObject_CallFunction(pFuncOpen, NULL);
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

bool pyModuleRemoteLink::decode_fcs_update( const char *buf ) {
    if (pModuleObj == NULL) {
	printf("ERROR: import logging module failed\n");
	return false;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "decode_fcs_update");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'decode_fcs_update()'\n");
	return false;
    }
    PyObject *pResult = PyObject_CallFunction(pFuncLog, (char *)"s", buf);
    if (pResult != NULL) {
	bool result = PyObject_IsTrue(pResult);
	Py_DECREF(pResult);
	return result;
    } else {
	PyErr_Print();
	printf("ERROR: call failed\n");
	return false;
    }
}
