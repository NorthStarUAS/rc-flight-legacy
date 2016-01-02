"""

props.py: a property tree system for python

Provides a hierarchical tree of shared data values.
 - Modules can use this tree as a way to share data
   (i.e. communicate) in a loosly structure / flexible way.
 - Both reader and writer can create properties in the shared tree so there
   is less worry about initialization order dependence.
 - Tree values can be accessed in native python code as nested class
   members: a = root; a.b.c.var1 = 42
 - Children nodes can be enumerated: /sensors/gps[0], /sensors/gps[1], etc.
 - C++ interface (in the future) allows complex but flexible data sharing
   between mixed C++ and python modules.
 - Maps well to xml or json data storage (i.e. xml/json config files can
   be loaded into a subtree (child) in the root shared property tree.

Notes:
 - getChild(path, True) will create 'path' as a set of PropertyNodes() if
   it doens't exist.   If the final component of the path is intended to be
   a leaf node, don't include it in the path ... it will be created as a
   parent node in the tree, not as a leaf node variable.
 - To create /path/to/variable and assign if a value, call:
   node = getNode("/path/to", create=True)
   node.variable = value

"""

import re

class PropertyNode:
    def getChild(self, path, create=False):
        # require relative paths
        if path[:1] == '/':
            return None
        tokens = path.split('/');
        node = self
        for i, token in enumerate(tokens):
            # test for enumerated form: ident[index]
            parts = re.split('([\w-]+)\[(\d+)\]', token)
            if len(parts) == 4:
                token = parts[1]
                index = int(parts[2])
            else:
                index = None
            if token in node.__dict__:
                # node exists
                if index == None:
                    node = node.__dict__[token]
                else:
                    # enumerated (list) child
                    tmp = node.__dict__[token]
                    if type(tmp) is list and len(tmp) > index:
                        node = tmp[index]
                    elif create:
                        # base node exists and list is not large enough and
                        # create flag requested: extend the list
                        self.__extendEnumeratedNode(tmp, index)
                        node = tmp[index]
                    else:
                        return None
                if not isinstance(node, PropertyNode) and i < len(tokens)-1:
                    print "path includes leaf nodes, sorry"
                    return None
            elif create:
                # token not found and create flag is true
                if index == None:
                    node.__dict__[token] = PropertyNode()
                    node = node.__dict__[token]
                else:
                    # test if base list exists, and extend size if needed
                    node.__dict__[token] = []
                    tmp = node.__dict__[token]
                    self.__extendEnumeratedNode(tmp, index)
                    node = tmp[index]
            else:
                # requestion token not found
                return None
        # return the last node in the path
        return node

    def pretty_print(self, indent=""):
        for child in self.__dict__:
            node = self.__dict__[child]
            if isinstance(node, PropertyNode):
                print indent + "/" + child
                node.pretty_print(indent + "  ")
            elif type(node) is list:
                for i, ele in enumerate(node):
                    print indent + "/" + child + "[" + str(i) + "]:"
                    ele.pretty_print(indent + "  ")
            else:
                print indent + child + ": " + str(node)
        
    def __extendEnumeratedNode(self, node, index):
        for i in range(len(node), index+1):
            print "appending:", i
            node.append( PropertyNode() )
            
        
root = PropertyNode()

def getNode(path, create=False):
    if path[:1] != '/':
        # require leading /
        return None
    elif path == "/":
        # catch trivial case
        return root
    print "getchild on", path[1:], "relative to root"
    return root.getChild(path[1:], create)
        
n1 = getNode("/a/b/c/d/e/f/g", True)
n1.var1 = 42
n1.var2 = 43
print getNode("/a/b/c/d/e1/f/g", True)
n2 = getNode("/a/b/c/d/e/f/g", False)
print n2.__dict__
print getNode("a/b/c/d/e/f/g")

a = getNode("/a", False)
print "a dict=", a.__dict__
print a.b.c.d.e.f.g.var1

n3 = getNode("/a/b/c/d/e/f/g/var1", False)
print "n3:", n3

n4 = getNode("/a/b/c")
n5 = n4.getChild("d/e/f/g")
print n5.__dict__
n6 = n5.getChild("var1")
print n6

# correct way to create a path with a new child node
gps = getNode("/sensors/gps[5]", True)
gps.alt_m = 275.3

# az get's created a parent node
az = getNode("/sensors/imu[2]/accel/az", True)
# this doesn't work
az = -9.81
# this should work
root.sensors.imu[2].accel.az = -9.81

root.pretty_print()

print "alt_m:", root.sensors.gps[5].alt_m

