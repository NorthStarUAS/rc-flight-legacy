# an ultra simple point class

class Point():
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def pretty(self):
        return (self.x, self.y)
