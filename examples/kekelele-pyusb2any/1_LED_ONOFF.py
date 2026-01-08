from usb2anyapi import *
import time

#LED ON OFF

#open the error print in console
openError()

#open u2a
handle = u2aOpen("")
ret = 0

#get the serial number of the usb2any
ret, szSerialNumber = GetSerialNumberFromHandle(handle)
print("serial: %s"%(szSerialNumber.decode()))


for i in range(0,10):
    time.sleep(1)
    #set the LED ON
    u2aLED_WriteControl(handle, LED.LED_ON)
    print("ON")
    time.sleep(1)
    #set the LED off
    u2aLED_WriteControl(handle, LED.LED_OFF)
    print("OFF")


u2aClose(handle)