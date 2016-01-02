import props

def multiply(a,b):
    print "by the way, the value of 'root:' is", props.root
    print "Will compute", a, "times", b
    c = 0
    for i in range(0, a):
        c = c + b
    return c

