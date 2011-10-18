#!/usr/bin/env python

#	  Copyright (c) 2011, 9th Sense, Inc.
#	  All rights reserved.
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

import sys
import traceback
import re

import dbus
import dbus.service
#for event loop
import gobject
from dbus.mainloop.glib import DBusGMainLoop

#######################################################
#catching the events
class Callback_obj(dbus.service.Object):
   def __init__(self, bus, object_path):
       dbus.service.Object.__init__(self, bus, object_path, bus_name='com.Skype.API')

   @dbus.service.method(dbus_interface='com.Skype.API')
   def Notify(self, message_text):
       pass

######################################################

dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

#connect to the session
session_bus = dbus.SessionBus()

#connect to Skype
skype = session_bus.get_object('com.Skype.API', '/com/Skype')

#ok lets hit up skype now!
answer = skype.Invoke('NAME PythonManageCall')
if answer != 'OK':
   sys.exit('Could not bind to Skype client')
answer = skype.Invoke('PROTOCOL 5')
if (answer != 'PROTOCOL 5'):
   sys.exit('Could not agree on protocol!')

#tie up the events to the skype
skype_callback = Callback_obj(session_bus, '/com/Skype/Client')

print 'Alright lets go!\n'

answer = skype.Invoke('SEARCH ACTIVECALLS') #get calls going on right now!
print "Active calls ", answer
if(re.search(r'CALLS [0-9]+', answer)): # see if there was a call
   callNum = re.search(r'CALLS ([0-9]+)', answer).group(1)
   print 'Answering Call ', callNum
   skype.Invoke('SET CALL ' + callNum + ' STATUS INPROGRESS')
