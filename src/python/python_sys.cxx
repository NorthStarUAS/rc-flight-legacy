#include "python_sys.hxx"

// This function must be called first (before any other python usage.)
// It sets up the python intepreter.
void AuraPythonInit(int argc, char **argv) {
    Py_SetProgramName(argv[0]); // optional but recommended
    Py_Initialize();
    PySys_SetArgv(argc, argv);  // for relative imports to work
}

// This function can be called from atexit() (after all the global
// destructors are called) to properly shutdown and clean up the
// python interpreter.
void AuraPythonCleanup(void) {
    Py_Finalize();
}
