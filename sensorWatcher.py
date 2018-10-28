#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import serial
import os
from datetime import datetime

ser = serial.Serial('/dev/ttyACM0', 9600)
lasthistorywrited = 0
while True:
    path = "/home/pi/AirPurifier/sensors.txt"
    pathhistory = "/home/pi/AirPurifier/history.txt"
    pathqueue = "/home/pi/AirPurifier/queue.txt"
    if os.path.isfile(pathqueue):
        q = open(pathqueue, 'r')
        ser.write("setmotorspeed %s" % q.read())
        os.remove(pathqueue)
    if len(ser.readline()) > 0:
        serialoutput = ser.readline()
        if serialoutput != "true" and serialoutput != "false":
            f=open(path, 'w')
            f.write(serialoutput)
            f.close()
            if time.time() - lasthistorywrited > 3600:
                print "writehistory"
                if not os.path.exists(pathhistory):
                    os.mknod(pathhistory)
                hr = open(pathhistory, 'r')
                content = hr.read()
                cbyl = content.split("\n")
                if len(cbyl) >= 10:
                    del cbyl[0]
                cbyl.append("%s:%s" % (int(time.time()), serialoutput))
                print 
                hw = open(pathhistory, 'w')
                for val in cbyl:
                    if val:
                        hw.write("%s\n" % val)
                hw.close()
                lasthistorywrited = time.time()
        else:
            print serialoutput
    time.sleep(0.3)