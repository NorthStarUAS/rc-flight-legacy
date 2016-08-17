from collections import deque
import time

import fgtelnet

hz = 10
dt = 1.0 / float(hz)
seconds = 100

lines = deque()

#port = 6499
port = 5402
t = fgtelnet.FGTelnet("localhost", port)
t.send("data")

count = 1

while True:
    t.send("fcs all")
    result = t.receive()
    tokens = map(float, result.split(','))
    if tokens[3] < 0.0:
        tokens[3] += 360.0
        print str(tokens[3])
    tokens[5] *= 25.0
    tokens[11] *= 25.0
    tokens[15] *= 200.0
    line = " ".join(map(str, tokens))
    print line
    lines.append(line)
    while len(lines) > hz * seconds:
        lines.popleft()
    print len(lines)
    if count >= hz:
        count = 0
        f = open("plotdata", "w")
        for line in lines:
            f.write(line + "\n")
        f.close()
        
    time.sleep(dt)
    count += 1
