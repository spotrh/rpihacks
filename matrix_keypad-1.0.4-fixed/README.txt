Introduction
============

Python Library for Matrix Keypads. 
Written and tested on a Model B Raspberry Pi.
Supports both a 3x4 and 4x4 keypad included

:Project Page:  Project_Page_
:PyPI page:  PyPI_Page_

Author
======

:Author:	Chris Crumpacker
:Email:		chris@chriscrumpacker.com
:Web:		http://www.chriscrumpacker.com
:Blog:		http://crumpspot.blogspot.com

Prerequisites
=============

If the I2C Port expander MCP23017 or MCP23008 is being used, the Adafruit Python library for I2C and the MCP will need to be installed.

You can clone the whole library like so::

	git clone https://github.com/adafruit/Adafruit-Raspberry-Pi-Python-Code.git

or the two files needed can be pulled out, Adafruit_I2C.py_ & Adafruit_MCP230xx.py_.

Install
=======

Install via PIP has not been tested just yet but it should be as simple as::

	pip install matrix_keypad

Files Included
==============
::

    README.txt
    LICENSE.txt
    setup.py
    matrix_keypad/
        __init__.py
        matrix_keypad_RPi_GPIO.py
        matrix_keypad_MCP230xx.py
        matrix_keypad_demo.py
        matrix_keypad_demo2.py

Usage
=====
*See the demo scripts included to see this all in action.*

To call the library select which one you intend to use and use the correct line::

    from matrixKeypad_MCP230xx import keypad

or::

    from matrixKeypad_RPi_GPIO import keypad

Then name the library so it is easier to reference later::
	
    kp = keypad()

It is possible to just check to see if a digit is currently pressed.::

    checkKeypad = kp.getKey()
	
Or a simple function to call the keypad library and 
loop through it waiting for a digit press ::

    def digit():
        # Loop while waiting for a keypress
        digitPressed = None
        while digitPressed == None:
            digitPressed = kp.getKey()
        return digitPressed
	
Version
=======

:v0.1.0:

	Initial Scripts

:v1.0.0_:
    
	Initial package build
	
:v1.0.1_:
    
	Initial package build and push to PyPI
	
:v1.0.2_:
	
	Updating the matrix_keypad_demo2.py to demo selecting the 4x4 keypad

:v1.0.3_:
	
	Moved Version Log in README
	
	Updated README Links

:v1.0.4_:
	
	Updated References to include the PiLarm code as the inspiration for the "...demo2.py" code
	
References
==========

Column and Row scanning adapted from Bandono's matrixQPI_ which is wiringPi based.

matrix_keypad_demo2.py is based on some work that Jeff Highsmith had done in making his PiLarm_ that was featured on Make. 


.. --------------------------------------------------------------------------
.. Links

.. _Project_Page: http://crumpspot.blogspot.com/2013/08/python-matrix-keypad-package.html
.. _PyPI_Page: https://pypi.python.org/pypi/matrix_keypad
.. _Adafruit_I2C.py: https://github.com/adafruit/Adafruit-Raspberry-Pi-Python-Code/blob/master/Adafruit_I2C/Adafruit_I2C.py
.. _Adafruit_MCP230xx.py: https://github.com/adafruit/Adafruit-Raspberry-Pi-Python-Code/blob/master/Adafruit_MCP230xx/Adafruit_MCP230xx.py
.. _matrixQPI: https://github.com/bandono/matrixQPi?source=cc
.. _PiLarm: http://makezine.com/video/pilarm-how-to-build-a-raspberry-pi-room-alarm/
.. _PiLarm_Code: https://github.com/BabyWrassler/PiNopticon/blob/master/keypadd.py
.. _v1.0.1: https://pypi.python.org/pypi/matrix_keypad/1.0.1
.. _v1.0.2: https://pypi.python.org/pypi/matrix_keypad/1.0.2
.. _v1.0.3: https://pypi.python.org/pypi/matrix_keypad/1.0.3
.. _v1.0.4: https://pypi.python.org/pypi/matrix_keypad/1.0.4