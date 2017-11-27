# an ultra simple point class

class Point():
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def pretty(self):
        return (self.x, self.y)
