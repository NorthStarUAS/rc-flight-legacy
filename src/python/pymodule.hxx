#ifndef _AURA_PYMODULE_HXX
#define _AURA_PYMODULE_HXX

// import a python module and call it's init() and update() routines
// requires imported python modules to follow some basic rules to play
// nice.  (see examples in the code for now.)

#include <Python.h>

class pyModuleBase {

public:

    // constructor / destructor
    pyModuleBase();
    virtual ~pyModuleBase();

    bool init(const char *import_name);
    bool update();
    
protected:

    PyObject *pModuleObj;
    
};

#endif // _AURA_PYMODULE_HXX
