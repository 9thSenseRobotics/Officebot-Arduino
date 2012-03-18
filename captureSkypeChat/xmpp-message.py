#!/usr/bin/env python
#
#	  Copyright (c) 2012, 9th Sense, Inc.
#	  All rights reserved.
#
#     Robot's XMPP client - publishes messages on /SkypeChat 
#     by Alaina Hardie
#
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

import logging, sys, string, time, math, os, re, datetime
import roslib; roslib.load_manifest('captureSkypeChat')
import rospy
from std_msgs.msg import String
from optparse import OptionParser

from sleekxmpp import ClientXMPP
from sleekxmpp.exceptions import IqError, IqTimeout

myRobotName = "litebot@9thsense.com"
myRobotPassword = "9thsense"

class EchoBot(ClientXMPP):

	def __init__(self, jid, password):
		ClientXMPP.__init__(self, jid, password)

		self.use_signals(signals=None)
		self.add_event_handler("session_start", self.session_start)
		self.add_event_handler("message", self.message)

		self.register_plugin('xep_0030') # Service Discovery
		self.register_plugin('xep_0199') # XMPP Ping
	
	def session_start(self, event):
		self.send_presence()

		# Most get_*/set_* methods from plugins use Iq stanzas, which
		# can generate IqError and IqTimeout exceptions
		try:
			self.get_roster()
		except IqError as err:
			logging.error('There was an error getting the roster')
			logging.error(err.iq['error']['condition'])
			self.disconnect()
		except IqTimeout:
			logging.error('Server is taking too long to respond')
			self.disconnect()

	def message(self, msg):
		global pub
		if msg['type'] in ('chat', 'normal'): 
			theString = msg['body']
			myItems = string.split(theString, '|', 2)
			print msg
			#myItems is an array of pipe-separated values from the XMPP server.
			# myItems[0] is the Jabber handle (e.g. "robot@9thsense.com") of the controller
			# myItems[1] is the "UNIXTIME.microseconds" that the message was sent
			# myItems[2] is the command that was sent
			# so a message to go forward would look something like this:
			#
			# rosie@9thsense.com|1331486994.728609|f
			#
			#mySentTime = float (myItems[1]) # convert time to float
			#myCurrentTime = microtime(True)
			#stringToSend = "I got this message (" + myItems[2].strip() + ") in " + repr(myCurrentTime - mySentTime) + " seconds"
			#stringToSend = "I got this message (" + myItems[2].strip() + ") "
			
			#rospy.loginfo (stringToSend)
			pub.publish(myItems[2].strip())
			#msg.reply(stringToSend).send()
			#self.send_message(mto=myItems[0], mbody=stringToSend) #send the message back to the jabber controller

if __name__ == '__main__':

 	rospy.init_node('XMPPListener', disable_signals=False)

	#logging.basicConfig(level=logging.ERROR, format='%(levelname)-8s %(message)s')
	logging.basicConfig(level=logging.DEBUG, format='%(levelname)-8s %(message)s')
 	pub = rospy.Publisher('SkypeChat', String)

	xmpp = EchoBot('litebot@9thsense.com', '9thsense')
	xmpp.connect()
	print "Connected, yo"
	try:
		xmpp.process(block=False)
	except rospy.ROSInterruptException: pass


