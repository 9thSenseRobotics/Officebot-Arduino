#! /usr/bin/env python

import sys, tty, termios
import os
import re
import datetime
import popen2
import subprocess
import signal
import pygst
pygst.require("0.10")
import gst

pipeline = gst.Pipeline('SkypeStream')

source = gst.element_factory_make('v4l2src', 'video-source')
source.set_property('device', '/dev/video0')
videoscale = gst.element_factory_make('videoscale')
sink = gst.element_factory_make('v4l2sink', 'video-output')
sink.set_property('device', '/dev/video3')

pipeline.add(source, videoscale, sink)
gst.element_link_many(source, videoscale, sink)	
def GetchUnix(): 
	fd = sys.stdin.fileno() 
	old_settings = termios.tcgetattr(fd) 
	try: 
		tty.setraw(sys.stdin.fileno()) 
		ch = sys.stdin.read(1) 
	finally: 
		termios.tcsetattr(fd, termios.TCSADRAIN, old_settings) 
	return ch 

while True:
	c = GetchUnix()
	print c
	if (c == 's'): 
		print 'Switch to video0'
		source.set_property('device', '/dev/video0')
	elif (c == 'd'):
		print 'Switch to video1'
		source.set_property('device', '/dev/video1')	