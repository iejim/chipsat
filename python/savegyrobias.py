#!/usr/bin/env python
""" savegyrobias.py

    Tells the IMU to capture the gyro bias and saves the value into the
    non-volatile memory so it is read after powering up the IMU.
    
    This program should be run if there is noticeably drift on the gyro
    readings (specially yaw). It must be run with the platform stationary
    (as in not on the air bearing) and runs for about 20 seconds.
    
    Ivan Jimenez
    
    """
import serial
import struct
ser_port = "/dev/ttyO4" #that's is not a zero
captureGB = "\xCD"
captureReply = 19
transferGB = "\xD0"
transferReply = 9
confirm = '\xC1\x29'

captureWait = 15000 #15s
saveBias = '\x00\x02'
    
def calculateChecksum(data, length):
    buffer = struct.unpack('B'*(length-2),data[0:length-2])
    chksum = struct.unpack('BB',data[length-2:])
    temp = (chksum[0] << 8)  + chksum[1]
    sum = 0
    for n in buffer:
        sum = sum + n
    
    return (sum == temp )
    
def sendCaptureGB(ser):
    msg = captureGB+confirm+hex(captureWait)[2:]
    ser.write(msg)
    #wait
    print "Waiting for capture..."
    data = ser.read(captureReply)
    #confirm
    print "Done waiting"
    if not data[0] == captureGB:
        return False
    
    if not calculateChecksum(data, len(data)):
        return False
    
    GBx = struct.unpack('>f',data[1:5])
    GBy = struct.unpack('>f',data[5:9])
    GBz = struct.unpack('>f',data[9:13])
    
    print "Calculated Gyro Bias:"
    print "GBx = %f"%GBx
    print "GBy = %f"%GBy
    print "GBz = %f"%GBz
    
    return True
    
def sendTransferGB(ser):
    msg = transferGB+confirm+saveBias
    ser.write(msg)
    #wait
    print "Sent transfer"
    data = ser.read(transferReply)
    #confirm
    if not data[0] == transferGB:
        return False
    
    if not calculateChecksum(data, len(data)):
        return False
    
    recvd = data[1:3]
    
    if recvd == saveBias:
        print "Done"
        
    return True
   
    
def main():

    ser = serial.Serial(ser_port, baudrate=115200, timeout=captureWait/1000+2)
    
    ser.flush();
    
    #send the 0xCE command every timeout+1 seconds
    try:
        c = sendCaptureGB(ser)
        if not c:
            print "A problem happened confirming the CaptureGB command"
        t = sendTransferGB(ser)
        if not t:
            print "A problem happened confirming the TransferGB command"
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
