#!/usr/bin/env python

import serial

ser_port = "/dev/ttyO4" #that's is not a zero
command = "\xce"
reply_size = 19

def main():
    """Runs a quick test of the serial port on the BeagleBone"""
    
    ser = serial.Serial(ser_port, baudrate=115200, timeout=2)
    
    ser.flush();
    
    #send the 0xCE command every timeout+1 seconds
    running =1
    while(running):
        try:
            ser.write(command)
            data = ser.read(reply_size)
            if (data):
                print "Received: " + str(data)
            else:
                print "Timeout or no data"
        except KeyboardInterrupt:
            running = 0
    
    print "Exitting..."
    
    ser.close()
    
if __name__ == '__main__':
    import sys
    argc = len(sys.argv)
    if argc>1:
        port = sys.argv[1]
        ser_port = port
    print "Using port: " + ser_port
    main()

    