#include <Python.h>

// This function must be called first (before any pyPropertyNode
// usage) to setup the python intepreter and import the python props
// module
extern void pyPropsInit(int argc, char **argv);

// This function can be called prior to exit (after the last property
// node usage) to clean up the python interpreter
extern void pyPropsClose();


//
// C++ interface to a python PropertyNode()
//
class pyPropertyNode
{
public:
    // Default constructor.
    pyPropertyNode();

    // Destructor.
    ~pyPropertyNode();
};
