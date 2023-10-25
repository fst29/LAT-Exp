import os

path= "/home/pi/fst29/namedPipeTest/test"
#os.mkfifo(path)

string = ""
while True:
    string = input()
    print(string)
    fifo = open(path, 'w')
    fifo.write(string)
    fifo.close()
    #time.sleep(0.5)