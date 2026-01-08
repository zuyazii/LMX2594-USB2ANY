from usb2anyapi import *
import time

#read flash demo
#M25D16

openError()
handle = u2aOpen("")
ret = 0

ret, szSerialNumber = GetSerialNumberFromHandle(handle)
print("serial %s"%(szSerialNumber.decode()))

u2aPower_WriteControl(handle, Power_3V3.Power_3V3_ON, Power_5V0.Power_5V0_OFF)

u2aPower_Enable(handle, Enable_3V3.Power_3V3_Enable, Enable_5V0.Power_5V0_Disable, Enable_ADJ.Power_ADJ_Disable)

time.sleep(1)

u2aSPI_Control(handle,
    SPI_ClockPhase.SPI_Capture_On_Leading_Edge,
    SPI_ClockPolarity.SPI_Inactive_State_Low,
    SPI_BitDirection.SPI_MSB_First, 
    SPI_CharacterLength.SPI__8_Bit, 
    SPI_ChipSelectType.SPI_With_Every_Packet, 
    SPI_ChipSelectPolarity.SPI_Active_Low, 
    SPI_BitRate.SPI_BitRate_100K)


time.sleep(1)
#read the Manufacturing ID and memery type for 
ret,data = u2aSPI_WriteAndRead(handle, 4, bytes([0x9F,0x00,0x00,0x00]))
print("data:%s"%(bytesToHexString(data)))

u2aPower_WriteControl(handle, Power_3V3.Power_3V3_OFF, Power_5V0.Power_5V0_OFF)

u2aClose(handle)