#ifndef _AURA_PYTHON_SYS_HXX
#define _AURA_PYTHOS_SYS_HXX

#include <Python.h>


// This function must be called first (before any other python usage.)
// It sets up the python intepreter.
extern void AuraPythonInit(int argc, char **argv);

// This function can be called from atexit() (after all the global
// destructors are called) to properly shutdown and clean up the
// python interpreter.
extern void AuraPythonCleanup(void);


#endif // _AURA_PYTHON_SYS_HXX
