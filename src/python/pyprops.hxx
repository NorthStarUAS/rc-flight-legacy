#ifndef _AURA_PYPROPS_HXX
#define _AURA_PYPROPS_HXX

#include <Python.h>

#include <string>
using std::string;


//
// C++ interface to a python PropertyNode()
//
class pyPropertyNode
{
public:
    // Constructor.
    pyPropertyNode();
    pyPropertyNode(PyObject *p);

    // Destructor.
    ~pyPropertyNode();

    bool hasChild(const char *name );
    pyPropertyNode getChild( const char *name, bool create=false );
    pyPropertyNode getChild( const char *name, int index );

    bool isNull();		// return true if pObj pointer is NULL
    int getLen( const char *name); // return len of pObj if a list, else 0
    
    // value getters
    double getDouble( const char *name ); // return value as a double
    long getLong( const char *name );	  // return value as a long
    bool getBool( const char *name );	  // return value as a boolean
    string getString( const char *name ); // return value as a string
    
    // value setters
    bool setDouble( const char *name, double val ); // returns true if successful
    bool setLong( const char *name, long val );     // returns true if successful
    bool setBool( const char *name, bool val );     // returns true if successful
    bool setString( const char *name, string val ); // returns true if successful
    
private:
    PyObject *pObj;
};


// This function must be called first (before any pyPropertyNode
// usage.) It sets up the python intepreter and imports the python
// props module.
void pyPropsInit(int argc, char **argv);

// This function can be called from atexit() (after all the global
// destructors are called) to properly shutdown and clean up the
// python interpreter.
extern void pyPropsCleanup(void);

// Return a pyPropertyNode object that points to the specified path in
// the property tree.  This is a 'heavier' operation so it is
// recommended to call this function from initialization routines and
// save the result.  Then use the pyPropertyNode for direct read/write
// access in your update routines.
pyPropertyNode pyGetNode(string abs_path, bool create=false);

#endif // _AURA_PYPROPS_HXX
