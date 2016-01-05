#include <Python.h>

#include <string>
using std::string;


//
// C++ interface to a python PropertyNode()
//
class pyPropertyNode
{
public:
    // Default constructor.
    pyPropertyNode();
    
    pyPropertyNode(PyObject *p);

    // Destructor.
    ~pyPropertyNode();

    // testers
    bool isBranch();		// returns true if this node can be a parent
    bool isLeaf();		// returns true if this is a leaf node (value)
    
    // getters
    double getDoubleValue();
    long getLongValue();
    string getStringValue();
    
    // setters
    bool setDoubleValue( double val );
    bool setLongValue( long val );
    bool setStringValue( string val );
    
private:
    PyObject *pObj;
};


// This function must be called first (before any pyPropertyNode
// usage.) It sets up the python intepreter and imports the python
// props module.
void pyPropsInit(int argc, char **argv);

// This function can be called prior to exit (after the last property
// node usage) to properly shutdown and clean up the python
// interpreter.
extern void pyPropsClose();

// Return a pyPropertyNode object that points to the specified path in
// the property tree.  This is a 'heavier' operation so it is
// recommended to call this function from initialization routines and
// save the result.  Then use the pyPropertyNode for direct read/write
// access in your update routines.
pyPropertyNode pyGetNode(string abs_path, bool create=false);

