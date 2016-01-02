# Todo: enumerated children like gps[0], gps[1], etc.

class PropertyNode:
    def getChild(self, path, create=False):
        # require relative paths
        if path[:1] == '/':
            return None
        tokens = path.split('/');
        node = self
        for i, token in enumerate(tokens):
            if token in node.__dict__:
                #print "  found:", token
                node = node.__dict__[token]
                if not isinstance(node, PropertyNode) and i < len(tokens)-1:
                    print "path includes leaf nodes, sorry"
                    return None
            elif create:
                #print "  creating:", token
                node.__dict__[token] = PropertyNode()
                node = node.__dict__[token]
            else:
                return None
        return node

    def pretty_print(self, indent=""):
        for child in self.__dict__:
            node = self.__dict__[child]
            if isinstance(node, PropertyNode):
                print indent + "/" + child
                node.pretty_print(indent + "  ")
            else:
                print indent + child + ": " + str(node)
        
        
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

n3 = getNode("/a/b/c/d/e/f/g/var1", True)

n4 = getNode("/a/b/c")
n5 = n4.getChild("d/e/f/g")
print n5.__dict__
n6 = n5.getChild("var1")
print n6

root.pretty_print()

