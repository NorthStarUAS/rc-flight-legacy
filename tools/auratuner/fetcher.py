import numpy as np
import threading

from collections import deque
import fgtelnet

data_fetcher_quit = False

class Fetcher():
    def __init__(self):
        self.hz = 10
        self.dt = 1.0 / float(self.hz)
        self.seconds = 30
        self.samples = deque()
        self.t = None
        self.points = None
        
    def connect(self, host="localhost", port=6499):
        self.t = fgtelnet.FGTelnet(host, port)
        self.t.send("data")

    def disconnect(self):
        self.t.quit()
        
    def update_data(self):
        self.t.send("fcs all")
        result = self.t.receive()
        print "result '%s'" % result
        if result == '':
            # probably no server running
            return
        elif result == 'Valid commands are:':
            print "fcs all not supported by remote server"
            return    
        tokens = map(float, result.split(','))
        if tokens[3] < 0.0:
            tokens[3] += 360.0
        #line = " ".join(map(str, tokens))
        #print line
        self.samples.append(tokens)
        cur_time = tokens[0]
        cutoff_time = cur_time - self.seconds
        while len(self.samples) > 1 and self.samples[0][0] < cutoff_time:
            self.samples.popleft()
        #print len(self.samples)

        #time.sleep(self.dt)
        if not data_fetcher_quit:
            # Timer spawns a thread that executes the function after
            # the specified time interval
            threading.Timer(self.dt, self.update_data).start()

    def get_data(self):
        #print 'shape:', np.array(self.samples).shape
        #shape = np.array(self.samples).shape
        #return np.random.rand(shape[0], shape[1])
        return np.array(self.samples)
    
df = Fetcher()
