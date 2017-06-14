#!/usr/bin/env python
import sys, serial, argparse
import rospy
import time
from cms50dplus.msg import PulseOx
import std_msgs.msg

class DataPoint(object):
    def __init__(self, data):
        #Data packet is 9 bytes: a 5-byte spo2, and a 4-byte pulse
        self.status = data[3] & 0x0f
        self.pulseRate = data[7] & 0x7f
        self.waveform = data[6]
        self.SpO2 = data[3] & 0x7f
        
    def __str__(self):
        return ", ".join(["Status = {0}",
                          " Pulse Rate = {1} bpm",
                          " SpO2 = {2}%"
                          " Pulse waveform = {3}"]).format(self.status,
                                                          self.pulseRate,
                                                          self.SpO2,
                                                          self.waveform)

class CMS50Dplus(object):
    COMMAND_A2 = 0xA2
    COMMAND_A7 = 0xA7
    COMMAND_A8 = 0xA8
    COMMAND_A9 = 0xA9
    COMMAND_AA = 0xAA #only requires 4 0x80 bytes afterwards
    COMMAND_A0 = 0xA0
    COMMAND_A1 = 0xA1
    COMMAND_B0 = 0xB0
    COMMAND_AC = 0xAC
    COMMAND_B3 = 0xB3
    
    def __init__(self, port):
        self.port = port
        self.conn = None

    def isConnected(self):
        return type(self.conn) is serial.Serial and self.conn.isOpen()

    def connect(self):
        if self.conn is None:
            self.conn = serial.Serial(port = self.port,
                                      baudrate = 115200,
                                      parity = serial.PARITY_NONE,
                                      stopbits = serial.STOPBITS_ONE,
                                      bytesize = serial.EIGHTBITS,
                                      timeout = 5,
                                      xonxoff = 1)
        elif not self.isConnected():
            self.conn.open()

    def disconnect(self):
        if self.isConnected():
            self.conn.close()

    def getBytes(self, nBytes):
        bytes = [0]*nBytes
        index = 0
        while index < nBytes:
            singleByte = self.getByte()
            if singleByte is not None:
                bytes[index] = self.getByte()
                index += 1
            else:
                break;
        return bytes
    
    def getByte(self):
        char = self.conn.read()
        if len(char) == 0:
            return None
        else:
            return ord(char)
    
    def sendBytes(self, values):
        print 'Send:', ' '.join(hex(x) for x in values)
        return self.conn.write(''.join([chr(value & 0xff) for value in values]))

    # Waits until the specified byte is seen or a timeout occurs
    def expectByte(self, value):
        while True:
            byte = self.getByte()
            if byte is None:
                return False
            elif byte == value:
                return True

    def sendInit(self):
        packet = [0]*9

        packet = [0x7d, 0x81, self.COMMAND_A7, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80]
        self.sendBytes(packet)
        packet = [0x7d, 0x81, self.COMMAND_A2, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80]
        self.sendBytes(packet)
        packet = [0x7d, 0x81, self.COMMAND_A0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80]
        self.sendBytes(packet)
        packet = [0x7d, 0x81, self.COMMAND_B0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80]
        self.sendBytes(packet)
        packet = [0x7d, 0x81, self.COMMAND_AC, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80]
        self.sendBytes(packet)
        packet = [0x7d, 0x81, self.COMMAND_B3, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80]
        self.sendBytes(packet)
        
        packet = [0x7d, 0x81, self.COMMAND_A8, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80]
        self.sendBytes(packet)

        packet = [0x7d, 0x81, self.COMMAND_AA, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80]
        self.sendBytes(packet)

        packet = [0x7d, 0x81, self.COMMAND_A9, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80]
        self.sendBytes(packet)
        packet = [0x7d, 0x81, self.COMMAND_A1, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80]
        self.sendBytes(packet)


        #Should be in streaming mode at this point


    def stopStreaming(self):
        packet = [0x7d, 0x81, self.COMMAND_A7, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80]
        self.sendBytes(packet)

        packet = [0x7d, 0x81, self.COMMAND_A2, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80]
        self.sendBytes(packet)

    def syncToStream(self):
        #Read bytes until we get an 0xe{0,1,2} followed by an 0xff
        print 'Syncing to stream'
        self.conn.flushInput()
        while True:
            buf = self.getByte()
            if buf == 0xff:
                return
            else:
                print 'Discarding ', hex(buf)
                continue
            
    def getStreamPacket(self):
        #Buffers come in two sizes: a 5-byte SpO2 and a 4-byte pulse
        buf = self.getBytes(9)

        #print 'Recv:', ' '.join(hex(x) for x in buf)
        return DataPoint(buf)
        
    def getLiveData(self):
        print 'Getting live data'
    
        self.connect()
        self.sendInit()
        self.syncToStream()
        self.syncToStream()
        
        index = 0
        while index < 150:
            print self.getStreamPacket()
            index += 1
        self.stopStreaming()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="cms50dplus.py ROS node")
    parser.add_argument("serialport", help="The device's virtual serial port.")

    args = parser.parse_args()

    print 'Running live'
  
    try:

        dev = CMS50Dplus(args.serialport)
        dev.connect()
        dev.sendInit()
        dev.syncToStream()
        dev.syncToStream()

        rospy.init_node('pulseox_node', anonymous=True)
        pub = rospy.Publisher('pulseox', PulseOx, queue_size=10)
        while not rospy.is_shutdown():
            packet = dev.getStreamPacket()
            msg = PulseOx()
            msg.header = std_msgs.msg.Header()
            msg.header.stamp = rospy.Time.now()
            msg.status = packet.status
            msg.heartrate = packet.pulseRate
            msg.spo2 = packet.SpO2
            msg.waveform = packet.waveform
            pub.publish(msg)
            
        dev.stopStreaming()
        
    except rospy.ROSInterruptException:
        pass


