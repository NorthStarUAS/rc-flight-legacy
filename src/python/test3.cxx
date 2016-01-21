#include <Python.h>
#include <string>
using std::string;

int
main(int argc, char *argv[])
{
    PyObject *pName, *pModuleMultiply, *pModuleProps, *pDict, *pFunc;
    PyObject *pArgs, *pValue;
    int i;

    if (argc < 3) {
        fprintf(stderr,"Usage: call pythonfile funcname [args]\n");
        return 1;
    }

    Py_SetProgramName(argv[0]);  /* optional but recommended */
    Py_Initialize();

    PySys_SetArgv(argc, argv);	// for relative imports to work
    pName = PyString_FromString(argv[1]);
    /* Error checking of pName left out */

    pModuleMultiply = PyImport_Import(pName);
    Py_DECREF(pName);

    // Test creating/setting a variable in python global space
    PyRun_SimpleString("import props\n");
    
    PyRun_SimpleString("print 'root:', props.root.pretty_print()\n");

    if (pModuleMultiply != NULL) {
        pFunc = PyObject_GetAttrString(pModuleMultiply, argv[2]);
        /* pFunc is a new reference */

        if (pFunc && PyCallable_Check(pFunc)) {
            pArgs = PyTuple_New(argc - 3);
            for (i = 0; i < argc - 3; ++i) {
                pValue = PyInt_FromLong(atoi(argv[i + 3]));
                if (!pValue) {
                    Py_DECREF(pArgs);
                    Py_DECREF(pModuleMultiply);
                    fprintf(stderr, "Cannot convert argument\n");
                    return 1;
                }
                /* pValue reference stolen here: */
                PyTuple_SetItem(pArgs, i, pValue);
            }
            pValue = PyObject_CallObject(pFunc, pArgs);
            Py_DECREF(pArgs);
            if (pValue != NULL) {
                printf("Result of call: %ld\n", PyInt_AsLong(pValue));
                Py_DECREF(pValue);
            } else {
                Py_DECREF(pFunc);
                Py_DECREF(pModuleMultiply);
                PyErr_Print();
                fprintf(stderr,"Call failed\n");
                return 1;
            }
        } else {
            if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"%s\"\n", argv[2]);
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModuleMultiply);
    } else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", argv[1]);
        return 1;
    }

    string module_name = "props";
    pModuleProps = PyImport_ImportModule(module_name.c_str());
    if (pModuleProps != NULL) {
	string var_name = "root";
	PyObject *pRoot = PyObject_GetAttrString(pModuleProps,
						 var_name.c_str());
	if ( pRoot ) {
	    PyObject_Print(pRoot, stdout, 0);
        } else {
            if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find variable \"%s\"\n", var_name.c_str());
	}
	Py_XDECREF(pRoot);
	printf("\n");
    } else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", module_name.c_str());
    }
    Py_XDECREF(pModuleProps);

    PyRun_SimpleString("print 'root:'\n");
    PyRun_SimpleString("props.root.pretty_print()\n");
    
    Py_Finalize();
    return 0;
}
