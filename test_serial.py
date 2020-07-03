#!/usr/bin/env python2

import socket
import sys
import signal
import serial
import time
import os
from threading import Thread

baudrate=2000000

def handler(signum, frame):
	print "RES: timeout "
	sys.stdout.flush()
	os._exit(1)

def test_recv(name,baudrate):
	ser = serial.Serial()
	ser.port = "/dev/ttyUSB1"
	ser.baudrate = baudrate
	ser.bytesize = serial.EIGHTBITS #number of bits per bytes
	ser.parity = serial.PARITY_NONE #set parity check: no parity
	ser.stopbits = serial.STOPBITS_ONE #number of stop bits
	#ser.timeout = None          #block read
	ser.timeout = 1            #non-block read
	#ser.timeout = 2              #timeout block read
	ser.xonxoff = False     #disable software flow control
	ser.rtscts = False     #disable hardware (RTS/CTS) flow control
	ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
	ser.writeTimeout = 2     #timeout for write
	ser.open()
	time.sleep(0.1);
	if ser.isOpen():
		print 'Recv: Serial ready'
		try:
			ser.flushInput()
			ser.flushOutput()
			c = 1
			while True:
				tm = ser.readline().strip()
				v = int(tm, 16)
				c = v + 1
				n = '%x' % c;
				ser.write( n + '\n' )
				print tm+" => "+n+" ("+str(len(n))+")"
			ser.close()
			sys.stdout.flush()
			os._exit(0)
		except Exception, e1:
			print "error communicating...: " + str(e1)
			sys.stdout.flush()
			os._exit(0)


def test_send(name,baudrate):
	ser = serial.Serial()
	ser.port = "/dev/ttyUSB0"
	ser.baudrate = baudrate
	ser.bytesize = serial.EIGHTBITS #number of bits per bytes
	ser.parity = serial.PARITY_NONE #set parity check: no parity
	ser.stopbits = serial.STOPBITS_TWO #number of stop bits
	#ser.timeout = None          #block read
	ser.timeout = 1            #non-block read
	#ser.timeout = 2              #timeout block read
	ser.xonxoff = False     #disable software flow control
	ser.rtscts = False     #disable hardware (RTS/CTS) flow control
	ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control
	ser.writeTimeout = 2     #timeout for write
	ser.open()
	time.sleep(0.2);
	if ser.isOpen():
		print 'Send: Uart ready'
		try:
			ser.flushInput()
			ser.flushOutput()
			c = 0
			ser.write('%x' % c+'\n')
			c = 1
			for x in range(1000):
				tm=ser.readline()
				v = int(tm, 16)
				if v != c:
					print "Send: Error!!! "+hex(v)+" "+hex(c);
					print "RES: ERROR ";
					sys.stdout.flush()
					os._exit(1)
				c = v + 1
				n = '%x' % c
				ser.write( n +'\n' )
				c = c + 1
			ser.close()
			print "RES: OK ";
			sys.stdout.flush()
			os._exit(0)
		except Exception, e1:
			print "error communicating...: " + str(e1)
			sys.stdout.flush()
			os._exit(0)

signal.signal(signal.SIGALRM, handler)
signal.alarm(10)

t = Thread(target=test_recv, args=("",baudrate))
ts = Thread(target=test_send, args=("",baudrate))
t.start()
ts.start()

while 1:
	pass

