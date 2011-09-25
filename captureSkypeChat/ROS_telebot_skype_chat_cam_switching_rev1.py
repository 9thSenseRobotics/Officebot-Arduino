#! /usr/bin/env python
#
# Shows Skype edited chat messages
# MODIFIED to be a CHAT PARSER
#   by Andrew Barry, (c) Copyright 2011, abarry@gmail.com
#
# Original:
#   (c) Copyright 2007, Vincent Oberle, vincent@oberle.org
#
# This software may be used and distributed according to the terms
# of the GNU Public License, incorporated herein by reference.
#
# This file requires python-xlib:
#	sudo apt-get install python-xlib
#
import roslib; roslib.load_manifest('captureSkypeChat')
import rospy
from std_msgs.msg import String
import sys
import os
import re
import datetime
from optparse import OptionParser

from skype_api import *

appname = 'telebot_command_parser'

# change these to add commands:
chat_string_table = ['list_dir', 'echo_hi']
chat_command_table = ['ls', 'echo "hi"']


# tables by chat id
edited_by = {}
edited_timestamp = {}
body = {}

callId = ''
lastCam = True

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
	
	# check for a new call so we can grab it's ID
	# Strings look like: Received: CALL 79 STATUS INPROGRESS
	#    Info: run_queue ['CALL 79 STATUS INPROGRESS']
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
	if not r: return   # event is not for us
	id = r.group(1).strip()
	prop = r.group(2).strip()
	params = r.group(3).strip()
	
	if prop == 'EDITED_BY':
		# Comes only when changed
		edited_by[id] = params
	elif prop == 'EDITED_TIMESTAMP':
		edited_timestamp[id] = params
	elif prop == 'BODY':
		body[id] = params
	
	#print event
	#print id
	#print prop
	#print params

	ret = api.send_and_block('GET CHATMESSAGE ' + str(id) + ' BODY')
	r = re.search (r'CHATMESSAGE ' + str(id) + ' BODY (.+)', ret)
	
	if (prop == 'STATUS' and params == 'RECEIVED'):
		messageBody = r.group(1).strip()
		print messageBody

		if (messageBody.find('Cam') >= 0 or messageBody.find('cam') >= 0):
			print 'SWITCH CAMERAS'
			print callId
			
			# create a symlink in /dev for the camera (our fake cam is on video9)
			if (lastCam == True):
				os.system('sudo ln -s -f /dev/video0 /dev/video9')
				lastCam = False
			else:
				os.system('sudo ln -s -f /dev/video1 /dev/video9')
				lastCam = True
			
			# stop skype video
			ret = api.send_and_block('ALTER CALL ' + str(callId) + ' STOP_VIDEO_SEND')
			
			# restart skype video
			ret = api.send_and_block('ALTER CALL ' + str(callId) + ' START_VIDEO_SEND')
		
		# search the message body for something that matches in chat_string_table
		#for i in range(0, len(chat_string_table)):
		#	if chat_string_table[i] == messageBody:
		#		os.system(chat_command_table[i])
		
		# search the message body for something that matches in chat_string_table
		#for i in range(0, len(chat_string_table)):
		#	if chat_string_table[i] == messageBody:
		#		os.system(chat_command_table[i])
		#os.system("rostopic pub /SkypeChat std_msgs/String " + messageBody)
		rospy.loginfo(messageBody)
		pub.publish(messageBody)
		#os.system('rostopic pub /SkypeChat std_msgs/String "f"')
	

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

	print 'Running...'
	try:
		while True:
			api.poll_events(1)
	except KeyboardInterrupt: # hides error message on control C
		print 'Exited'
