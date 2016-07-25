import glob
import serial
import time

def getPort():
  ports = glob.glob("/dev/tty.usbmodem*")
  if len(ports)>0:
    return ports[0]
  return "None"



def getPositionString(position):
    x = (position[0]+40.0)*255.0/80.0
    y = position[1] * 255.0 / 40.0
    z = (position[2] + 35.0) * 255.0 / 80.0
    x = int(max(min(255.0, x),0.0))
    y = int(max(min(255.0, y), 0.0))
    z = int(max(min(255.0, z), 0.0))
    return "x"+chr(x)+"y"+chr(y)+"z"+chr(z)

def getSerialConnect():
    ser = serial.Serial(getPort(), 57600)
    print("connecting")
    ready=str(ser.read(5))
    print(ready)
    return ser

