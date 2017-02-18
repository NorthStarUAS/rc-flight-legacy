#include "python/pyprops.hxx"
#include "util/sg_path.hxx"

#include "logging.hxx"


pyModuleLogging::pyModuleLogging()
{
}

bool pyModuleLogging::open(const char *path)
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

void pyModuleLogging::update() {
    if (pModuleObj == NULL) {
	printf("ERROR: import logging module failed\n");
	return;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "update");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'update_configs()'\n");
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


// FIXME: close log file!
bool pyModuleLogging::close()
{
    return true;
}

void pyModuleLogging::log_actuator( uint8_t *buf, int size ) {
    if (pModuleObj == NULL) {
	printf("ERROR: import logging module failed\n");
	return;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "log_actuator");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'log_actuator()'\n");
	return;
    }
    PyObject *pResult = PyObject_CallFunction(pFuncLog, (char *)"s#i", buf, size, size);
    if (pResult != NULL) {
	Py_DECREF(pResult);
	return;
    } else {
	PyErr_Print();
	printf("ERROR: call failed\n");
	return;
    }
}

void pyModuleLogging::log_airdata( uint8_t *buf, int size ) {
    if (pModuleObj == NULL) {
	printf("ERROR: import logging module failed\n");
	return;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "log_airdata");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'log_airdata()'\n");
	return;
    }
    PyObject *pResult = PyObject_CallFunction(pFuncLog, (char *)"s#i", buf, size, size);
    if (pResult != NULL) {
	Py_DECREF(pResult);
	return;
    } else {
	PyErr_Print();
	printf("ERROR: call failed\n");
	return;
    }
}

void pyModuleLogging::log_ap( uint8_t *buf, int size ) {
    if (pModuleObj == NULL) {
	printf("ERROR: import logging module failed\n");
	return;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "log_ap");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'log_ap()'\n");
	return;
    }
    PyObject *pResult = PyObject_CallFunction(pFuncLog, (char *)"s#i", buf, size, size);
    if (pResult != NULL) {
	Py_DECREF(pResult);
	return;
    } else {
	PyErr_Print();
	printf("ERROR: call failed\n");
	return;
    }
}

void pyModuleLogging::log_filter( uint8_t *buf, int size ) {
    if (pModuleObj == NULL) {
	printf("ERROR: import logging module failed\n");
	return;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "log_filter");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'log_filter()'\n");
	return;
    }
    PyObject *pResult = PyObject_CallFunction(pFuncLog, (char *)"s#i", buf, size, size);
    if (pResult != NULL) {
	Py_DECREF(pResult);
	return;
    } else {
	PyErr_Print();
	printf("ERROR: call failed\n");
	return;
    }
}

void pyModuleLogging::log_gps( uint8_t *buf, int size ) {
    if (pModuleObj == NULL) {
	printf("ERROR: import logging module failed\n");
	return;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "log_gps");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'log_gps()'\n");
	return;
    }
    PyObject *pResult = PyObject_CallFunction(pFuncLog, (char *)"s#i", buf, size, size);
    if (pResult != NULL) {
	Py_DECREF(pResult);
	return;
    } else {
	PyErr_Print();
	printf("ERROR: call failed\n");
	return;
    }
}

void pyModuleLogging::log_health( uint8_t *buf, int size ) {
    if (pModuleObj == NULL) {
	printf("ERROR: import logging module failed\n");
	return;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "log_health");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'log_health()'\n");
	return;
    }
    PyObject *pResult = PyObject_CallFunction(pFuncLog, (char *)"s#i", buf, size, size);
    if (pResult != NULL) {
	Py_DECREF(pResult);
	return;
    } else {
	PyErr_Print();
	printf("ERROR: call failed\n");
	return;
    }
}

void pyModuleLogging::log_imu( uint8_t *buf, int size ) {
    if (pModuleObj == NULL) {
	printf("ERROR: import logging module failed\n");
	return;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "log_imu");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'log_imu()'\n");
	return;
    }
    PyObject *pResult = PyObject_CallFunction(pFuncLog, (char *)"s#i", buf, size, size);
    if (pResult != NULL) {
	Py_DECREF(pResult);
	return;
    } else {
	PyErr_Print();
	printf("ERROR: call failed\n");
	return;
    }
}

void pyModuleLogging::log_payload( uint8_t *buf, int size ) {
    if (pModuleObj == NULL) {
	printf("ERROR: import logging module failed\n");
	return;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "log_payload");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'log_payload()'\n");
	return;
    }
    PyObject *pResult = PyObject_CallFunction(pFuncLog, (char *)"s#i", buf, size, size);
    if (pResult != NULL) {
	Py_DECREF(pResult);
	return;
    } else {
	PyErr_Print();
	printf("ERROR: call failed\n");
	return;
    }
}

void pyModuleLogging::log_pilot( uint8_t *buf, int size ) {
    if (pModuleObj == NULL) {
	printf("ERROR: import logging module failed\n");
	return;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "log_pilot");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'log_pilot()'\n");
	return;
    }
    PyObject *pResult = PyObject_CallFunction(pFuncLog, (char *)"s#i", buf, size, size);
    if (pResult != NULL) {
	Py_DECREF(pResult);
	return;
    } else {
	PyErr_Print();
	printf("ERROR: call failed\n");
	return;
    }
}

void pyModuleLogging::log_raven( uint8_t *buf, int size ) {
    if (pModuleObj == NULL) {
	printf("ERROR: import logging module failed\n");
	return;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "log_raven");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'log_raven()'\n");
	return;
    }
    PyObject *pResult = PyObject_CallFunction(pFuncLog, (char *)"s#i", buf, size, size);
    if (pResult != NULL) {
	Py_DECREF(pResult);
	return;
    } else {
	PyErr_Print();
	printf("ERROR: call failed\n");
	return;
    }
}

void pyModuleLogging::write_configs() {
    if (pModuleObj == NULL) {
	printf("ERROR: import logging module failed\n");
	return;
    }
    PyObject *pFuncLog = PyObject_GetAttrString(pModuleObj, "write_configs");
    if ( pFuncLog == NULL || ! PyCallable_Check(pFuncLog) ) {
	if ( PyErr_Occurred() ) PyErr_Print();
	printf("ERROR: cannot find function 'write_configs()'\n");
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


// write out the imu calibration parameters associated with this data
// (this allows us to later rederive the original raw sensor values.)
bool write_imu_calibration( pyPropertyNode *config ) {
    pyPropertyNode logging_node = pyGetNode( "/config/logging", true );
    SGPath jsonfile = logging_node.getString("flight_dir");
    jsonfile.append( "imucal.json" );
    writeJSON( jsonfile.str(), config );
    return true;
}
