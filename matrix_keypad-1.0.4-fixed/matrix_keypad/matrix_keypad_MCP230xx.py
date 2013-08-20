#!/usr/bin/python
 
from Adafruit_MCP230xx import Adafruit_MCP230XX
 
class keypad(Adafruit_MCP230XX):
    def __init__(self, address=0x21, num_gpios=8, columnCount=3):
         
        self.mcp2 = Adafruit_MCP230XX(address, num_gpios)
        # Constants
        self.INPUT   = 0
        self.OUTPUT  = 1
        self.HIGH    = 1
        self.LOW     = 0

        if columnCount is 3:
            # CONSTANTS   
            self.KEYPAD = [
            [1,2,3],
            [4,5,6],
            [7,8,9],
            ["*",0,"#"]
            ]

	    self.ROW         = [6,5,4,3]
            self.COLUMN      = [2,1,0]
        elif columnCount is 4:
            self.KEYPAD = [
            [1,2,3,"A"],
            [4,5,6,"B"],
            [7,8,9,"C"],
            ["*",0,"#","D"]
            ]

	    self.ROW         = [7,6,5,4]
            self.COLUMN      = [3,2,1,0]
        else:
	    return

    def getKey(self):
         
        # Set all columns as output low
        for j in range(len(self.COLUMN)):
            self.mcp2.config(self.COLUMN[j], self.mcp2.OUTPUT)
            self.mcp2.output(self.COLUMN[j], self.LOW)
         
        # Set all rows as input
        for i in range(len(self.ROW)):
            self.mcp2.config(self.ROW[i], self.mcp2.INPUT)
            self.mcp2.pullup(self.ROW[i], True)
         
        # Scan rows for pushed key/button
        # valid rowVal" should be between 0 and 3 when a key is pressed. Pre-setting it to -1
        rowVal = -1
        for i in range(len(self.ROW)):
            tmpRead = self.mcp2.input(self.ROW[i])
            if tmpRead == 0:
                rowVal = i
                 
        # if rowVal is still "return" then no button was pressed and we can exit
        if rowVal == -1:
            self.exit()
            return
         
        # Convert columns to input
        for j in range(len(self.COLUMN)):
            self.mcp2.config(self.COLUMN[j], self.mcp2.INPUT)
         
        # Switch the i-th row found from scan to output
        self.mcp2.config(self.ROW[rowVal], self.mcp2.OUTPUT)
        self.mcp2.output(self.ROW[rowVal], self.HIGH)
         
        # Scan columns for still-pushed key/button
        colVal = -1
        for j in range(len(self.COLUMN)):
            tmpRead = self.mcp2.input(self.COLUMN[j])
            if tmpRead == 1:
                colVal=j
         
        if colVal == -1:
            self.exit()
            return
               
        # Return the value of the key pressed
        self.exit()   
        return self.KEYPAD[rowVal][colVal]
             
    def exit(self):
        # Reinitialize all rows and columns as input before exiting
        for i in range(len(self.ROW)):
                self.mcp2.config(self.ROW[i], self.INPUT) 
        for j in range(len(self.COLUMN)):
                self.mcp2.config(self.COLUMN[j], self.INPUT)
         
if __name__ == '__main__':
    # Initialize the keypad class
    kp = keypad()
     
    # Loop while waiting for a keypress
    r = None
    while r == None:
        r = kp.getKey()
         
    # Print the result
    print r 
