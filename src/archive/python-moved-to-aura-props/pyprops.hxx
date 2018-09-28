#pragma once

#include <Python.h>

#include <string>
#include <vector>
using std::string;
using std::vector;


// a simple attribute caching class to save frequent and repeated
// str->unicode conversion work.
#include <unordered_map>
using std::unordered_map;
typedef unordered_map<string, PyObject *> attr_cache_t ;

class pyAttrCache {
    
private:
    attr_cache_t cache;

public:

    pyAttrCache() {
        cache.clear();
    }
    
    ~pyAttrCache() {
        cache.clear();
    }

    // return a cached attribute or create it
    PyObject *get_attr(const char *name) {
        attr_cache_t::iterator it;
        it = cache.find(name);
        if ( it != cache.end() ) {
            return it->second;
        } else {
            PyObject* attr = PyUnicode_FromString(name);
            cache[name] = attr;
            return attr;
        }
    }
};

    
//
// C++ interface to a python PropertyNode()
//
class pyPropertyNode
{
public:
    // Constructor.
    pyPropertyNode();
    pyPropertyNode(const pyPropertyNode &node); // copy constructor
    pyPropertyNode(PyObject *p);

    // Destructor.
    ~pyPropertyNode();

    // operator =
    pyPropertyNode & operator= (const pyPropertyNode &node);

    bool hasChild(const char *name );
    pyPropertyNode getChild( const char *name, bool create=false );
    pyPropertyNode getChild( const char *name, int index, bool create=false );

    bool isNull();		// return true if pObj pointer is NULL
    
    int getLen( const char *name); // return len of pObj if a list, else 0
    void setLen( const char *name, int size); // set len of name
    void setLen( const char *name, int size, double init_val); // set len of name

    vector <string> getChildren(bool expand=true); // return list of children

    bool isLeaf( const char *name); // return true if pObj/name is leaf
    
    // value getters
    double getDouble( const char *name ); // return value as a double
    long getLong( const char *name );	  // return value as a long
    bool getBool( const char *name );	  // return value as a bool
    string getString( const char *name ); // return value as a string

    // indexed value getters
    double getDouble( const char *name, int index ); // return value as a double
    long getLong( const char *name, int index ); // return value as a long
    bool getBool( const char *name, int index ); // return value as a bool
    string getString( const char *name, int index ); // return value as a string

    // value setters
    bool setDouble( const char *name, double val ); // returns true if successful
    bool setLong( const char *name, long val );     // returns true if successful
    bool setBool( const char *name, bool val );     // returns true if successful
    bool setString( const char *name, string val ); // returns true if successful

    // indexed value setters
    bool setDouble( const char *name, int index, double val  ); // returns true if successful
    
    void pretty_print();

    // semi-private (pretend you can't touch this!) : :-)
    PyObject *pObj;

private:
    // really private
    double PyObject2Double(const char *name, PyObject *pAttr);
    long PyObject2Long(const char *name, PyObject *pAttr);
};


// This function must be called before any pyPropertyNode usage. It
// imports the python props and props_xml/json modules.
extern void pyPropsInit();

// This function can be called at exit to properly free resources
// requested by init()
extern void pyPropsCleanup(void);

// Return a pyPropertyNode object that points to the specified path in
// the property tree.  This is a 'heavier' operation so it is
// recommended to call this function from initialization routines and
// save the result.  Then use the pyPropertyNode for direct read/write
// access in your update routines.
extern pyPropertyNode pyGetNode(string abs_path, bool create=false);

// Read an xml file and place the results at specified node
extern bool readXML(string filename, pyPropertyNode *node);
    
// Write an xml file beginning with the specified node
extern bool writeXML(string filename, pyPropertyNode *node);
    
// Read a json file and place the results at specified node
extern bool readJSON(string filename, pyPropertyNode *node);
    
// Write a json file beginning with the specified node
extern bool writeJSON(string filename, pyPropertyNode *node);
