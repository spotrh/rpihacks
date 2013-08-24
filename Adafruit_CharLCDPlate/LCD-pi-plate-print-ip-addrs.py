#!/usr/bin/python
#
# Based off the LCDtest.py script (BSD) included with the Adafruit_CharLCDPlate module
# For full license information on LCDtest.py, see README.md.
# LCD-pi-plate-print-ip-addrs.py: Copyright (C) 2013 Tom Callaway <spotrh@gmail.com>
# Copyright (c) 2013 Tom Callaway
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the "Software"), to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
# to permit persons to whom the Software is furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of
# the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
# THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
# CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.

from time import sleep
from Adafruit_CharLCDPlate import Adafruit_CharLCDPlate
from netifaces import interfaces, ifaddresses, AF_INET

menu_msg = 'Pi IP Addrs\nUP:eth0 DN:wlan0'

def ipaddr_for_device(device):
    if device in interfaces():
       addresses = [i['addr'] for i in ifaddresses(device).setdefault(AF_INET, [{'addr':'No IP addr'}] )]
       return '%s:\n%s' % (device, ', '.join(addresses))
    else:
       return 'No %s device' % (device)

# Initialize the LCD plate.  Should auto-detect correct I2C bus.  If not,
# pass '0' for early 256 MB Model B boards or '1' for all later versions
lcd = Adafruit_CharLCDPlate()

# Clear display
lcd.clear()
lcd.backlight(lcd.ON)

# Init menu message
lcd.message(menu_msg)

# Change the message based on the button
btn = ((lcd.LEFT, menu_msg),
       (lcd.UP, ipaddr_for_device('eth0')),
       (lcd.DOWN, ipaddr_for_device('wlan0')),
       (lcd.RIGHT, menu_msg),
       (lcd.SELECT, menu_msg))

# Loop forever, change when any button is pressed.
# This is so that if the IP changes, we can simply repress the button for
# that device, and it will update.
while True:
    for b in btn:
        if lcd.buttonPressed(b[0]):
           lcd.clear()
           lcd.message(b[1])
