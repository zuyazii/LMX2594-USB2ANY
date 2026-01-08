#control of USB2any need 32 bit environment
#download 32 bit python and run the code 

from ctypes import *
import ctypes as ct
import time as t
#spi settings 
SPI_ClockPhase = ct.c_bool(True)
SPI_ClockPolarity = ct.c_bool(False)
SPI_BitDirection = ct.c_bool(True) # MSb first (1)
SPI_CharacterLength = ct.c_bool(False) # 8 bits (0)
SPI_CSType = ct.c_int(1) #(1) after every packet and (2) after every word
SPI_CSPolarity = ct.c_bool(True) #(active low)
DividerHigh =ct.c_byte(0)# high and low for 8000khz data rate 
DividerLow =ct.c_byte(3)



# Load DLL into memory.
path = "E:\\ram\\test\\USB2ANY.dll"
u2aDll = ct.WinDLL (path)

#find no of devices 
no_of_devices = u2aDll.u2aFindControllers()
print(no_of_devices)

#find the serial number of the device 
SerialNumber = ct.c_buffer(255)
s =u2aDll.u2aGetSerialNumber(0, ct.byref(SerialNumber))
print(s)

#open the device and make an handle (K)
K =u2aDll.u2aOpen (ct.byref(SerialNumber))
print(K)
print(type(K))

#sending the SPI settings 
s=u2aDll.u2aSPI_Control (K,SPI_ClockPhase,SPI_ClockPolarity,SPI_BitDirection,
                SPI_CharacterLength,SPI_CSType,SPI_CSPolarity,DividerHigh,
                DividerLow)
print(s)

#writing the spi command
#(first 1 hex for writing 0x0 or 0X1 and for reading 0x8 or 0x9)
#(second  hex for address)
#(last 4 hex for data)

#for 3.3 v enabling the circuit
s=u2aDll.u2aPower_Enable (K,1,0,0)
#t.sleep(5)

print("power")
print(s)

#dummy_write_to set the registers
m = (c_uint8*3)()
m[0]=0x00
m[1]=0x00
m[2] =0x00
s=u2aDll.u2aSPI_WriteAndRead (K,ct.c_byte(3),byref(m)) 
print(s)
print(hex(m[0]))
print(hex(m[1]))
print(hex(m[2]))
#t.sleep(1)

#set the DEV-PWDWN to 1 in the SPICONFIG Register
m = (c_uint8*3)()
m[0]=0x03
m[1]=0x0a
m[2] =0x04
s=u2aDll.u2aSPI_WriteAndRead (K,ct.c_byte(3),byref(m)) 
print(s)
print(hex(m[0]))
print(hex(m[1]))
print(hex(m[2]))
#t.sleep(1)

#set the DAC0 to 8000 
o = (c_uint8*3)()
o[0]=0x10
o[1]=0x80
o[2] =0x00
s=u2aDll.u2aSPI_WriteAndRead (K,ct.c_byte(3),byref(o)) 
print(s)
print(hex(o[0]))
print(hex(o[1]))
print(hex(o[2]))
#t.sleep(1)

#switching on the DAC0 
p = (c_uint8*3)()
p[0]=0x09
p[1]=0xff
p[2] =0xfe
s=u2aDll.u2aSPI_WriteAndRead (K,ct.c_byte(3),byref(p)) 
print(s)
print(hex(p[0]))
print(hex(p[1]))
print(hex(p[2]))

#t.sleep(1)

#close the device with handle (K)
s =u2aDll.u2aClose (K)
print(s)
