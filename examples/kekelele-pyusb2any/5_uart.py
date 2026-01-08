from usb2anyapi import *
import time
import sys
#uart interface




openError()

if __name__ == '__main__':

    if sys.argv[0] == '':
        print("Input File name!")
        exit()

    path = sys.argv[1]
    with open(path, mode='rb') as file:
        data = file.read()
    
    handle = u2aOpen("")
    ret = 0

    ret, szSerialNumber = GetSerialNumberFromHandle(handle)
    print("serial %s"%(szSerialNumber.decode()))

    u2aUART_Control(handle, UART_BaudRate.UART_115200_bps, UART_Parity.UART_None, UART_BitDirection.UART_LSB_First, UART_CharacterLength.UART_8_Bit, UART_StopBits.UART_One_Stop)


    time.sleep(1)

    u2aUART_SetMode(handle, UART_Mode.UART_Normal)

    time.sleep(1)

    #write one byte
    print(len(data))
    u2aUART_Write(handle, len(data), data)

    time.sleep(1)

    u2aClose(handle)

