import math
from multiprocessing import Queue
import csv
import os
import smbus
import time
import threading
import serial
import sys
import PID

# configuration variables
ARDUINO_ADDRESS             = 0x04  # i2c address for arduino
ARDUINO_DATA_COUNT          = 5    # no of sensors on arduino
SENSOR_TILE_DATA_COUNT      = 24
DATA_READ_INTERVAL          = 50    # milliseconds
PID_UPDATE_INTERVAL         = 50
YAW_P                       = 1.5
YAW_I                       = 0.0
YAW_D                       = 0.0
YAW_INDEX                   = 23
US_INDEX                    = 3
MAX_SPEED                   = 255
TURN_ANGLE                  = 45
OBSTACLE_DISTANCE           = 10
MAX_DISTANCE_DIFF           = 25
VERBOSE_DATA_REPORTING      = True
DATA_SOURCE                 = 'sonar'       # sonar or encoders

arduinoBus = smbus.SMBus(1)
try:
    sensorTile = serial.Serial('/dev/ttyACM0', 9600)
except:
    sensorTile = serial.Serial('/dev/ttyACM1', 9600)

sensorData = [0] * (1 + ARDUINO_DATA_COUNT + SENSOR_TILE_DATA_COUNT + 2)
sensorDataReady = False
dataReadFlag = False
dataLogFlag = False
autopilotFlag = False
initialtime = 0
sensorDataQueue = Queue()

prevX = prevY = None
turningFlag = False
turnFlag = False
prevUS = None

def highByte (number) : return number >> 8
def lowByte (number) : return number & 0x00FF
def getWord (lowByte, highByte): 
    word = ((highByte << 8) | lowByte)
    if word > 32767:
        word -= 65536
    return word

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

        drive('stop', 255, False)
        while True:
            time.sleep(0.01)

    except KeyboardInterrupt:
        shutdown()

    except Exception as e:
        print "Exception in main " + str(e)
        shutdown()
        raise

def getFileName():
    directory = '../../data/live-data'
    if not os.path.exists(directory):
        os.makedirs(directory)
    number = 0
    while True:
        fileNamePath = directory + ('/record_%d.csv' % number)
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

    except:
        sensorTileDataHandler()

def processFromSonar():
    global sensorData
    global prevX
    global prevY
    global prevUS
    global turningFlag
    global turnFlag

    if turnFlag is True and turningFlag is False:
        turningFlag = True
        sensorData[-2] = prevX
        sensorData[-1] = prevY

        print "turn = %s, turning = %s"%(turnFlag,turningFlag)
        return

    elif turnFlag is False and turningFlag is True:
        turningFlag = False
        sensorData[-4] = prevX
        sensorData[-3] = prevY
        prevUS = float(sensorData[US_INDEX])

        print "turn = %s, turning = %s"%(turnFlag,turningFlag)
        return
    
    if prevX is None and prevY is None:
        prevUS = float(sensorData[US_INDEX])
        if prevUS == 0:
            prevUS = None
            sensorData[-2] = 0
            sensorData[-1] = 0
            return
        sensorData[-2] = 0
        sensorData[-1] = 0
        prevX = prevY = 0
        return

    currentYaw = float(sensorData[YAW_INDEX])
    currentDistance = float(sensorData[US_INDEX])

    if currentDistance == 0:
        sensorData[-2] = prevX
        sensorData[-1] = prevY
        return
    
    distanceDiff = prevUS - currentDistance

    if abs(distanceDiff) > MAX_DISTANCE_DIFF:
        sensorData[-2] = prevX
        sensorData[-1] = prevY
        prevUS = currentDistance
        return

    localX = math.cos(math.radians(currentYaw))*distanceDiff
    localY = math.sin(math.radians(currentYaw))*distanceDiff

    tempX = prevX + localX
    tempY = prevY + localY

    #distance = math.hypot(tempX-prevX, tempY-prevY)
    #if distance > MAX_DISTANCE_DIFF :
    #    sensorData[-2] = prevX
    #    sensorData[-1] = prevY
    #    return

    prevX += localX
    prevY += localY

    sensorData[-2] = prevX
    sensorData[-1] = prevY
    prevUS = currentDistance

def processFromEncoders():
   pass 

def dataProcessor():
    if DATA_SOURCE == 'sonar':
        processFromSonar()
    elif DATA_SOURCE == 'encoders':
        processFromEncoders()

def readSensorData():
    global sensorData
    global sensorDataReady
    global dataReadFlag
    global dataLogFlag

    nextDataReadTime = getTimestamp()

    with open(getFileName(), 'wb') as rawfile:
        csvfile = csv.writer(rawfile, delimiter=',') 
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
                    
                    if VERBOSE_DATA_REPORTING:
                        dataProcessor()
                        sensorDataQueue.put(sensorData)

                    sensorDataReady = True

                    if dataLogFlag:
                        csvfile.writerow(sensorData)
                    time.sleep(0.01)

            else:
                time.sleep(0.01)

def writeMotorSpeeds(speedLeft, speedRight):
    try:
        arduinoBus.write_block_data(ARDUINO_ADDRESS, 0, [highByte(int(speedLeft)), lowByte(int(speedLeft)), highByte(int(speedRight)), lowByte(int(speedRight))])
    except IOError:
        writeMotorSpeeds(speedLeft, speedRight)
    except Exception as e:
        print "Exception " + str(e)
        shutdown()

def checkObstacle(sensorData, obstacleArray=[]):
    obstacleFlag = False
    obstacleSum = 0
    for sensorIndex in range(1, SONAR_NUM + 1):
        obstacleArray.append(sensorData[sensorIndex]) 
        if sensorData[sensorIndex] > 0 and sensorData[sensorIndex] < OBSTACLE_DISTANCE:
            obstacleSum += (sensorIndex - (SONAR_NUM +1)/2)
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
    global autopilotFlag
    global sensorDataQueue
    global turnFlag

    lock = threading.Lock()

    if type == 'sonar-yaw':
        robotPID = PID.PID(YAW_P, YAW_I, YAW_D)
        robotPID.setPoint = getSensorData()[YAW_INDEX]
        robotPID.setSampleTime(PID_UPDATE_INTERVAL)

    while True:
        if autopilotFlag:
            sensorData = getSensorData()

            if not VERBOSE_DATA_REPORTING:
                dataProcessor()
                sensorDataQueue.put(sensorData)

            if type == 'sonar':
                obstacleArray = []
                # Test
                writeMotorSpeeds(speed, speed)


                '''
                obstacle = checkObstacle(sensorData, obstacleArray)

                if obstacle == 100:     # no obstacle
                    writeMotorSpeeds(speed, speed)
                    time.sleep(0.01)
                elif obstacle < 0:      # obstacle towards left
                    turnFlag = True
                    writeMotorSpeeds(speed, -speed)
                    
                    lock.acquire()
                    dataProcessor()
                    lock.release()
                    
                    time.sleep(.700)
                    writeMotorSpeeds(0, 0)
                    turnFlag = False
                elif obstacle >= 0:     # obstacle towards right or in front
                    turnFlag = True
                    
                    lock.acquire()
                    dataProcessor()
                    lock.release()
                    
                    writeMotorSpeeds(-speed, speed)
                    time.sleep(.700)
                    writeMotorSpeeds(0, 0)
                    turnFlag = False
                '''

            elif type == 'sonar-yaw':
                obstacleArray = []
                obstacle = checkObstacle(sensorData, obstacleArray)

                if obstacleArray[2] == 0:
                    writeMotorSpeeds(0, 0)

                elif obstacle == 100:
                    feedback = sensorData[YAW_INDEX] - robotPID.setPoint

                    if feedback < -180.0:
                        feedback += 360
                    elif feedback > 180:
                        feedback -= 360

                    robotPID.update(feedback)
                    pidOutput = int(robotPID.output)

                    if pidOutput < 0:
                        writeMotorSpeeds(MAX_SPEED + pidOutput, MAX_SPEED)      # turn left if PID output is -ve
                    else:
                        writeMotorSpeeds(MAX_SPEED, MAX_SPEED - pidOutput)      # turn right if PID output is +ve

                elif obstacle < 0:
                    robotPID.setPoint += TURN_ANGLE
                    if robotPID.setPoint > 180:
                        robotPID.setPoint -= 360

                    writeMotorSpeeds(speed, -speed)
                    while (robotPID.setPoint - getSensorData()[YAW_INDEX])  > 10:
                        pass
                    else:
                        writeMotorSpeeds(0, 0)

                else:
                    robotPID.setPoint -= TURN_ANGLE
                    if robotPID.setPoint < -180:
                        robotPID.setPoint += 360

                    writeMotorSpeeds(-speed, speed)
                    while getSensorData()[YAW_INDEX] - robotPID.setPoint > 10:
                        pass
                    else:
                        writeMotorSpeeds(0, 0)

        else:
            writeMotorSpeeds(0, 0)
            print "Autopilot Stopped"
            return

def shutdown():
    print "Shutting Down"
    writeMotorSpeeds(0, 0)
    sys.exit(0)

main()

