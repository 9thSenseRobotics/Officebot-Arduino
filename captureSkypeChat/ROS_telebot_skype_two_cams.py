#! /usr/bin/env python
#
#	  Copyright (c) 2011, 9th Sense, Inc.
#	  All rights reserved.
#
# Shows Skype edited chat messages
# MODIFIED to be a CHAT PARSER
#   by Andrew Barry, (c) Copyright 2011, abarry@gmail.com
#   by Alaina Hardie, (c) Copyright 2011, alaina@9thsense.com
# Original:
#   (c) Copyright 2007, Vincent Oberle, vincent@oberle.org
#     This program is free software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
# 
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
# 
#     You should have received a copy of the GNU General Public License
#     along with this program.  If not, see <http://www.gnu.org/licenses/>.

#
# This file requires python-xlib:
#	sudo apt-get install python-xlib
#
import roslib; roslib.load_manifest('captureSkypeChat')  
roslib.load_manifest('turtlebot_node')
import rospy
from std_msgs.msg import String
import sys
import syslog
import pycurl
import cStringIO
import os
import re
import datetime
import time
from optparse import OptionParser
from turtlebot_node.msg import TurtlebotSensorState
from skype_api import *

# just  hang out until the internets are up
status = 0
syslog.syslog("Telo - Skype API script started")
while (status < 200):
	try:
		b = cStringIO.StringIO()
		c = pycurl.Curl()
		c.setopt(c.URL, 'http://www.google.com/')
		c.setopt(c.WRITEFUNCTION,b.write)
		c.perform()
		status = c.getinfo(pycurl.HTTP_CODE)
		print status
		c.close()
	except:
		continue
syslog.syslog ("Telo - I found Google")

# Now start Skype


skypeIsRunning = False

for line in os.popen ("ps ax"): 	# get the process list
	sl = line.split()		# split on whitespace
	pid = sl[0]
	processName = sl[4]
	if (processName.startswith ("skype")): # skype is already running
		print 'Skype is already running in process ' + str(pid)
		skypeIsRunning = True # Don't try to start it again
	if (processName.find(os.path.basename(__file__)) > -1): 	# this script is already running
		print 'The ' + __file__ + ' script is already running. Exiting.'
		exit						

if not skypeIsRunning:
	print "Starting Skype"
	os.system('skype &')
	print "Waiting 20 sec for Skype to start"
	time.sleep(20)

appname = 'telebot_command_parser'

# change these to add commands:
chat_string_table = ['list_dir', 'echo_hi']
chat_command_table = ['ls', 'echo "hi"']
chat_return_status_table = ['battery', 'Battery', 'help', 'Help']

help_driving = "To drive, use WASD: 'w' to move forward, 's' to move backwards, 'a' to turn left, and 'd' to turn right. You can send up to five of the same letter to repeat the motion. NOTE: the increase is not linear. In other words, 'ww' is more than the equivalent of two W's, and 'www' is much more than the equivalent of three W's."
# tables by chat id
edited_by = {}
edited_timestamp = {}
body = {}

callId = ''
lastCam = True
localUser = ''
remoteUser = ''
charge = 0
capacity = 0
pct = 0

pub = rospy.Publisher('SkypeChat', String)

# ROS publisher
#def SkypeListener():
#    while not rospy.is_shutdown():
#        rospy.sleep(1.0)
#if __name__ == '__main__':
#    try:
#        SkypeListener()
#    except rospy.ROSInterruptException: pass

# Callback for property change events
def edited_onchange(event, api):

	global callId
	global lastCam
	global localUser
	global remoteUser
	global charge
	global capacity
	global pct
	
	# check for a new call so we can grab its ID
	# Strings look like: Received: CALL 79 STATUS INPROGRESS
	#    Info: run_queue ['CALL 79 STATUS INPROGRESS']
	#print event
	
	r = re.search (r'CALL (\d+) (\w+) (.*)', event)
	if (r != None):
		# this is a call
		
		# check to make sure this is the beginning of the call
		thisId = r.group(1).strip()
		prop = r.group(2).strip()
		params = r.group(3).strip()
		
		#if (prop == 'STATUS' and params == 'INPROGRESS'):	
		#print 'Got call, ID = ' + str(thisId)
		callId = thisId
		

	r = re.search (r'CHATMESSAGE (\d+) (\w+) (.*)', event)
	r2 = re.search (r'CHAT \#(\w+)\/\$(\w+);(\w+) (\w+) (.*)', event)
	if not r and not r2: return   # event is not for us
	if r:
		id = r.group(1).strip()
		prop = r.group(2).strip()
		params = r.group(3).strip()
	elif r2:
		#print r2
		remoteUser = r2.group(1).strip()
		localUser = r2.group(2).strip()
		#print 'Found remote user ' + remoteUser + ', local user ' + localUser
		return
		
	if prop == 'EDITED_BY':
		# Comes only when changed
		edited_by[id] = params
	elif prop == 'EDITED_TIMESTAMP':
		edited_timestamp[id] = params
	elif prop == 'BODY':
		body[id] = params
	
	try: 
		ret = api.send_and_block('GET CHATMESSAGE ' + str(id) + ' BODY')
	except:
		print "An error occurred trying to get the message body"
		
	r = re.search (r'CHATMESSAGE ' + str(id) + ' BODY (.+)', ret)
	
	if (prop == 'STATUS' and params == 'RECEIVED'):
		messageBody = r.group(1).strip()

		if (messageBody.find('battery') >= 0):
			print 'Send battery status to ' + remoteUser + ' from ' + localUser
			try:
				chatCreateResult = api.send_and_block('CHAT CREATE ' + remoteUser)
				mylist = chatCreateResult.split()
				pct = round((float(charge) / float(capacity)) * 100)
			
				statusString = "Battery capacity is " +  str(pct) + '%'
				api.send_and_block('CHATMESSAGE ' + mylist[1] + ' ' + statusString)
			except NameError:
				print 'User must chat with us first'
		if (messageBody.find('Cam') >= 0 or messageBody.find('cam') >= 0):
			print 'SWITCH CAMERAS'
			print callId
			
			# create a symlink in /dev for the camera (our fake cam is on video9)
			if (lastCam == True):
				os.system('sudo ln -s -f /dev/video2 /dev/video9')
				lastCam = False
			else:
				os.system('sudo ln -s -f /dev/video1 /dev/video9')
				lastCam = True
			
			try:
				# stop and restart skype video
				ret = api.send_and_block('ALTER CALL ' + str(callId) + ' STOP_VIDEO_SEND')
				ret = api.send_and_block('ALTER CALL ' + str(callId) + ' START_VIDEO_SEND')
			except:
				print 'An error occurred trying to restart the video sending'
		
		rospy.loginfo(messageBody)
		pub.publish(messageBody)

def sensorStateCallback (TurtlebotSensorState):
	global charge
	global capacity
	#print 'in callback'
	charge = TurtlebotSensorState.charge
	capacity = TurtlebotSensorState.capacity
	#print charge
	

def sensorListener ():
	rospy.Subscriber('/turtlebot_node/sensor_state', TurtlebotSensorState, sensorStateCallback)
	print 'TurtleBot sensor subscriber defined'
	

if __name__ == "__main__":
	parser = OptionParser('%prog [options]')

	parser.add_option('-d', '--debug', action='store_true', dest='debug',
		default = False, help='Print debug messages')

	options, args = parser.parse_args()
	if len(args) > 0:
		parser.print_help()
		sys.exit(0)

	rospy.init_node('SkypeListener')
	
	os.system('sudo ln -s -f /dev/video1 /dev/video9')
	
	try:
		api = SkypeAPI(appname, options.debug)
	except StandardError:
		print 'Could not connect to Skype. Check if "' + appname + '" is authorized to connect to Skype (Options - Public API)'
		sys.exit(0)

	api.set_callback(edited_onchange, api)

	sensorListener () # track /turtlebot_node/sensor_state
	
	print 'Running...'
	try:
		while True:
			api.poll_events(1)
	except KeyboardInterrupt: # hides error message on control C
		print 'Exited'
