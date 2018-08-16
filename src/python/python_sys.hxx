#pragma once

#include <Python.h>

#include <string>
using std::string;


// This function must be called first (before any other python usage.)
// It sets up the python intepreter.
extern void AuraPythonInit(int argc, char **argv, string extra_module_path);

// This function can be called from atexit() (after all the global
// destructors are called) to properly shutdown and clean up the
// python interpreter.
extern void AuraPythonCleanup(void);
