#!/usr/bin/python
# This code is based on Chris Crumpacker's example code, but with the bugs fixed.
# His code is GPLv3+. To keep things simple, this code is also GPLv3+.
# Copyright (C) 2013 Tom Callaway <spotrh@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from matrix_keypad.matrix_keypad_RPi_GPIO import keypad

from time import sleep
from sys import exit

# Initialize the keypad class. Using the **optional** variable to change it to a 4x4 keypad
kp = keypad(columnCount = 4)

# Setup variables
attempt = "0000"
passcode = "8675"
counter = 0

print 'Enter your four digit secret code into the numeric keypad!'

# Loop while waiting for a keypress
while True:
	# Loop to get a pressed digit
        digit = None
        while digit == None:
                digit = kp.getKey()

        # Print the result
        print "Digit Entered:       %s"%digit
        attempt = (attempt[1:] + str(digit))
        print "Attempt value:       %s"%attempt

        # Check for passcode match
        if (attempt == passcode):
                print "Your code was correct, goodbye."
                exit()
        else:
             	counter += 1
                print "Entered digit count: %s"%counter
                if (counter >= 4):
                        print "Incorrect code!"
                        sleep(3)
                        print "Try Again"
                        sleep(1)
                        counter = 0
	sleep(0.5)
