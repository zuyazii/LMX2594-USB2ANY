from usb2anyapi import *
import time

#24c02 
I2C_24C02_ID = 0x50

#print(u2aFindControllers())
openError()
handle = u2aOpen("")
ret = 0

ret, szSerialNumber = GetSerialNumberFromHandle(handle)
print("serial %s"%(szSerialNumber.decode()))

u2aPower_WriteControl(handle, Power_3V3.Power_3V3_OFF, Power_5V0.Power_5V0_ON)

u2aPower_Enable(handle, Enable_3V3.Power_3V3_Disable, Enable_5V0.Power_5V0_Enable, Enable_ADJ.Power_ADJ_Disable)

time.sleep(1)

u2aI2C_Control(handle, I2C_Speed.I2C_100kHz, I2C_AddressLength.I2C_7Bits, I2C_PullUps.I2C_PullUps_OFF)

#write one byte from 0x50
u2aI2C_RegisterWrite(handle, I2C_24C02_ID, 0x50, 0x01)

#read one byte from 0x50
data = u2aI2C_RegisterRead(handle, I2C_24C02_ID, 0x50)
print(data)

#write 2 bytes in address 0x00
u2aI2C_MultiRegisterWrite(handle, I2C_24C02_ID, 0x00, 2, bytes([1,2,3,4,5,7]))

#read 40 bytes from address 0x00
ret, data = u2aI2C_MultiRegisterRead(handle, I2C_24C02_ID, 0x00, 40)
print("data:%s"%(bytesToHexString(data)))

#write bytes [0x28, 2] to device
u2aI2C_RawWrite(handle, I2C_24C02_ID, 2, bytes([0x28,2]))

#read the 2bytes data from address 0x28
ret, data = u2aI2C_MultiRegisterRead(handle, I2C_24C02_ID, 0x28, 2)
print("data:%s"%(bytesToHexString(data)))

#write 1 byte and read 2 bytes
ret, data = u2aI2C_BlockWriteBlockRead(handle, I2C_24C02_ID, 1, bytes([0x1]), 2)
print("data:%s"%(bytesToHexString(data)))

#only read fonction
# ret, data = u2aI2C_RawRead(handle, I2C_24C02_ID, 2)
# print("data:%s"%(bytesToHexString(data)))

# ret, data = u2aI2C_RawRead(handle, I2C_24C02_ID, 2)
# print("data:%s"%(bytesToHexString(data)))

for i in range(0, 32):
    ret, data = u2aI2C_InternalRead(handle, I2C_24C02_ID, i*8, 1, 8)
    print("%d:%s"%(i,bytesToHexString(data)))

u2aClose(handle)