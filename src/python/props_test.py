#!/usr/bin/python

from props import getNode, root

# run the system through it's paces

n1 = getNode("/a/b/c/d/e/f/g", create=True)
n1.var1 = 42
n1.var2 = 43
print getNode("/a/b/c/d/e1/f/g", create=True)
n2 = getNode("/a/b/c/d/e/f/g", create=False)
print n2.__dict__
print getNode("a/b/c/d/e/f/g")

a = getNode("/a", create=False)
print "a dict=", a.__dict__
print a.b.c.d.e.f.g.var1

n3 = getNode("/a/b/c/d/e/f/g/var1", create=False)
print "n3:", n3
n3 = getNode("/a/b/c/d/e/f/g/var1", create=True)
print "n3:", n3

n4 = getNode("/a/b/c")
n5 = n4.getChild("d/e/f/g")
print n5.__dict__
n6 = n5.getChild("var1")
print n6

# correct way to create a path with a new child node
gps = getNode("/sensors/gps[5]", create=True)
gps.alt_m = 275.3

# az get's created a parent node
imu = getNode("/sensors/imu[2]", create=True)
# this works, but is bad form because az is originally created as a
# PropertyNode() branch that can't have a value, only childredn
imu.az = -9.80
# this should work
root.sensors.imu[2].az = -9.81

root.pretty_print()

print "alt_m:", root.sensors.gps[5].alt_m
