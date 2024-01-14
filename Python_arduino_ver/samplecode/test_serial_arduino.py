import time
import serial
import math

loopcount = 0
test_speed = 0
test_steer = 5

ser = serial.Serial("COM9", 115200, timeout = 1)
while (loopcount <1000):
    op = str(100+100*math.sin(loopcount*0.1))+"," + str(20+15*math.sin(loopcount*0.05))+",b,0,test_message &"
    ser.write(op.encode())
    print("R: ", loopcount)

    if (test_speed<255):
        test_speed = test_speed + 0.1

    time.sleep(0.01)
    loopcount = loopcount +1

ser.close()



