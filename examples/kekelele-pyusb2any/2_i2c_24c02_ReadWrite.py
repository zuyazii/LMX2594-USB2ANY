from usb2anyapi import *
import time

#24c02 
I2C_24C02_ID = 0x50

openError()
handle = u2aOpen("")
ret = 0

ret, szSerialNumber = GetSerialNumberFromHandle(handle)
print("serial %s"%(szSerialNumber.decode()))

#set the 5V0 for 24c02
u2aPower_WriteControl(handle, Power_3V3.Power_3V3_OFF, Power_5V0.Power_5V0_ON)

#enable the 5V0 door
u2aPower_Enable(handle, Enable_3V3.Power_3V3_Disable, Enable_5V0.Power_5V0_Enable, Enable_ADJ.Power_ADJ_Disable)

time.sleep(1)

#set the i2c
u2aI2C_Control(handle, I2C_Speed.I2C_100kHz, I2C_AddressLength.I2C_7Bits, I2C_PullUps.I2C_PullUps_OFF)

for i in range(0, 32):
    #write for each page
    u2aI2C_InternalWrite(handle, I2C_24C02_ID, i*8, 1, 8, bytes([a for a in range(i*8, i*8+8)]))

for i in range(0, 32):
    #read each page
    ret, data = u2aI2C_InternalRead(handle, I2C_24C02_ID, i*8, 1, 8)
    print("%d:%s"%(i,bytesToHexString(data)))

u2aClose(handle)