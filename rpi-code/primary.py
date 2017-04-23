import csv
import os
import smbus
import time
import threading
import serial
import sys

# configuration variables
ARDUINO_ADDRESS             = 0x04  # i2c address for arduino
ARDUINO_DATA_COUNT          = 5    # no of sensors on arduino
SENSOR_TILE_DATA_COUNT      = 24
DATA_READ_INTERVAL          = 100    # milliseconds
AUTOPILOT_UPDATE_INTERVAL   = 100    # milliseconds
MAX_SPEED                   = 255

arduinoBus = smbus.SMBus(1)
try:
    sensorTile = serial.Serial('/dev/ttyACM0', 9600)
except:
    sensorTile = serial.Serial('/dev/ttyACM1', 9600)

sensorData = [0] * (1 + ARDUINO_DATA_COUNT + SENSOR_TILE_DATA_COUNT)
sensorDataReady = False
dataReadFlag = False
dataLogFlag = False
autopilotFlag = False
initialtime = 0

def highByte (number) : return number >> 8
def lowByte (number) : return number & 0x00FF
def getWord (lowByte, highByte) : return ((highByte << 8) | lowByte)

def main():
    try:
        global dataReadFlag
        global sensorDataReady
        global sensorData
        global initialtime

        initialtime = time.time()

        try:
            dataReadThread = threading.Thread(target=readSensorData)
            dataReadThread.setDaemon(True)
            dataReadThread.start()
        except Exception as e:
            print "Exception in dataReadThread " + str(e)
            shutdown()

        dataReadFlag = False

        drive('autopilot-sonar', 155, False)
        while True:
            time.sleep(0.01)

    except KeyboardInterrupt:
        shutdown()

    except Exception as e:
        print "Exception in main " + str(e)
        shutdown()
        raise

def getFileName():
    number = 0
    while True:
        fileNamePath = '../../data/live-data/record_%d.csv' % number
        if not os.path.exists(fileNamePath):
            return fileNamePath
        else:
            number += 1

def getTimestamp():
    return time.time() - initialtime

def arduinoDataHandler():
    global sensorData

    try:
        rawData = arduinoBus.read_i2c_block_data(ARDUINO_ADDRESS, 0)

        if (len(rawData) != 32):
            readSensorData()

        for sensorIndex in range(1, ARDUINO_DATA_COUNT + 1):
            sensorData[sensorIndex] = getWord(rawData[2*(sensorIndex-1) + 1], rawData[2*(sensorIndex-1) + 0])
            if sensorData[sensorIndex] > 1023:
                arduinoDataHandler()

    except IOError:
        arduinoDataHandler()

def sensorTileDataHandler():
    global sensorData

    try:
        sensorTile.flushInput()

        while True:
            rawData = sensorTile.readline().split(', ')
            if len(rawData) == 24:
                for sensorIndex in range(0, SENSOR_TILE_DATA_COUNT):
                    if sensorIndex < 11:
                        sensorData[(1 + ARDUINO_DATA_COUNT) + sensorIndex] = int(rawData[sensorIndex])
                    else:
                        sensorData[(1 + ARDUINO_DATA_COUNT) + sensorIndex] = float(rawData[sensorIndex])
                break

            else:
                continue

    except Exception:
        print "Exception in sensorTileDataHandler " + str(Exception)
        shutdown()

def writeMotorSpeeds(speedLeft, speedRight):
    try:
        arduinoBus.write_block_data(ARDUINO_ADDRESS, 0, [highByte(speedLeft), lowByte(speedLeft), highByte(speedRight), lowByte(speedRight)])
    except IOError:
        writeMotorSpeeds(speedLeft, speedRight)
    except Exception as e:
        print "Exception " + str(e)
        shutdown()

def readSensorData():
    global sensorData
    global sensorDataReady
    global dataReadFlag
    global dataLogFlag

    nextDataReadTime = getTimestamp()

    '''
    with open(getFileName(), 'wb') as rawfile:
        csvfile = csv.writer(rawfile, delimiter=',') 
    '''
    while True:
        if dataReadFlag:
            currentTime = getTimestamp()
            if currentTime >= nextDataReadTime:
                sensorDataReady = False

                nextDataReadTime += DATA_READ_INTERVAL/1000.0
                
                sensorData[0] = currentTime
                arduinoDataHandler()
                sensorTileDataHandler()
                print sensorData

                sensorDataReady = True

                #if dataLogFlag:
                #   csvfile.writerow(sensorData)

        else:
            time.sleep(0.01)

def checkObstacle(obstacleArray=[]):
    global sensorData
    
    obstacleFlag = False
    obstacleSum = 0
    for sensorIndex in range(1, 4):
        obstacleArray.append(sensorData[sensorIndex]) 
        if sensorData[sensorIndex] > 0 and sensorData[sensorIndex] < 10:
            obstacleSum += (sensorIndex - 2)
            obstacleFlag = True


    if obstacleFlag:
        return obstacleSum
    else:
        return 100

def getSensorData():
    global sensorDataReady
    global sensorData

    while not sensorDataReady:
        pass

    return sensorData

def drive(command, speed=127, dataLog=True):
    global dataReadFlag
    global dataLogFlag
    global autopilotFlag
    
    dataReadFlag = True
    dataLogFlag = dataLog
    autopilotFlag = False

    if command == 'forward':
        writeMotorSpeeds(speed, speed)

    if command == 'backward':
        writeMotorSpeeds(-speed, -speed)

    if command == 'left':
        writeMotorSpeeds(0, speed)

    if command == 'right':
        writeMotorSpeeds(speed, 0)

    if command == 'stop':
        writeMotorSpeeds(0, 0)

    if command == 'halt':
        writeMotorSpeeds(0, 0)
        dataReadFlag = False

    if command == 'autopilot-sonar':
        try:
            autopilotThread.join()
        
        except NameError:
            autopilotFlag = True

            autopilotThread = threading.Thread(target=autopilot, args=('sonar', speed))
            autopilotThread.setDaemon(True)
            autopilotThread.start()

    if command == 'autopilot-sonar-yaw':
        try:
            autopilotThread.join()
        
        except NameError:
            autopilotFlag = True

            autopilotThread = threading.Thread(target=autopilot, args=('sonar-yaw', speed))
            autopilotThread.setDaemon(True)
            autopilotThread.start()

def autopilot(type='sonar', speed=255):
    global sensorData
    global sensorDataReady
    global autopilotFlag

    nextAutopilotUpdateTime = getTimestamp()

    while True:
        if autopilotFlag:
            currentTime = getTimestamp()
            if currentTime >= nextAutopilotUpdateTime:
                nextAutopilotUpdateTime += AUTOPILOT_UPDATE_INTERVAL/1000.0
                print 'here'

                while not sensorDataReady:
                    pass
                
                if type == 'sonar':
                    writeMotorSpeeds(speed, speed)

                    '''
                    obstacle = checkObstacle()

                    if obstacle == 100:     # no obstacle
                        writeMotorSpeeds(speed, speed)
                    elif obstacle < 0:      # obstacle towards left
                        writeMotorSpeeds(speed, -speed)
                        time.sleep(.500)
                    elif obstacle >= 0:     # obstacle towards right or in front
                        writeMotorSpeeds(-speed, speed)
                        time.sleep(.500)
                    '''
        else:
            return

def shutdown():
    print "Shutting Down"
    writeMotorSpeeds(0, 0)
    sys.exit(0)

main()
