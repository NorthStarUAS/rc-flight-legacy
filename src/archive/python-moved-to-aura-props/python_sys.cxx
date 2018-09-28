#include "python_sys.hxx"

#include <sstream>
#include <string>
using std::ostringstream;
using std::string;

// This function must be called first (before any other python usage.)
// It sets up the python intepreter.
void AuraPythonInit(int argc, char **argv, string extra_module_path) {
    wchar_t* program = Py_DecodeLocale(argv[0], NULL);
    Py_SetProgramName(program); // optional but recommended
    Py_Initialize();
    #if 0
    PySys_SetArgv(argc, argv);  // for relative imports to work (not needed for python3?)
    #endif
    if ( extra_module_path != "" ) {
	ostringstream command;
	command << "import sys\n";
	command << "sys.path.append(\"";
	command << extra_module_path;
	command << "\")\n";
	PyRun_SimpleString(command.str().c_str());
    }
}

// This function can be called from atexit() (after all the global
// destructors are called) to properly shutdown and clean up the
// python interpreter.
void AuraPythonCleanup(void) {
    Py_Finalize();
}
