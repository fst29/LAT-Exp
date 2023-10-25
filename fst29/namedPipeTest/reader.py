import os
import time
path= "/home/pi/fst29/namedPipeTest/test"
#os.mkfifo(path)

string = ""
while True:
    fifo = open(path, 'r')
    string = fifo.read()
    fifo.close()
    print(string)
    #time.sleep(0.5)