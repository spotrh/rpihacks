# nxt.execute module -- Class to control LEGO Mindstorms NXT motors
# Copyright (C) 2006  Douglas P Lau
# Copyright (C) 2009  Marcus Wanner
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

'Use for execute control'

import time
import nxt
import rospy
class Execute():

    def __init__(self, brick, fname):
        self.brick = brick
        self.fname = fname
        #self.brick.play_tone(440, 800)

    def start_program(self):

        try:
            self.brick.stop_program()
        except nxt.error.DirProtError:
            pass
            self.brick.start_program(self.fname+".rxe")
            
        time.sleep(0.1)
    
    def stop_program(self):
        pass
        self.brick.stop_program()
