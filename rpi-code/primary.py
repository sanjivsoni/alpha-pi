import smbus
import time
import threading
import os
import sys

ARDUINO_ADDRESS = 0x04
ARDUINO_DATA_BYTES = 6

def highByte (number) : return number >> 8
def lowByte (number) : return number & 0x00FF
def getWord (lowByte, highByte) : return ((highByte << 8) | lowByte)

try:
    # 1 indicates /dev/i2c-1
    rpiBus = smbus.SMBus(1)
except Exception as e:
    print "Exception in Bus:" + str(e)
    shutdown()
    raise

def main():
    try:
        while(True):
            drive('forward', 255, True)
            time.sleep(0.01)
            print 'forward'
    except Exception as e:
        print "Exception in main" + str(e)
        shutdown()
        raise


def drive(command, speed = 127, dataLog = True):
    if command == 'forward':
        writeMotorSpeed(speed, speed)

def writeMotorSpeed(speedLeft, speedRight):
    try:
	# 0 - Register to write to
        rpiBus.write_block_data(ARDUINO_ADDRESS, 0, [highByte(speedLeft), lowByte(speedLeft), highByte(speedRight), lowByte(speedRight)])
    except IOError:
        writeMotorSpeed(speedLeft, speedRight)
    except Exception as e:
        print "Exception in writeMotorSpeed " + str(e)
        shutdown() 

def shutdown():
    print "Shutting Down"
    writeMotorSpeed(0, 0)
    sys.exit(0)

main()
