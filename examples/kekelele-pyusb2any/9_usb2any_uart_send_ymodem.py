from usb2anyapi import *
import time
import sys, os
#uart interface
from ymodem import YMODEM



openError()

if sys.argv[0] == '':
    print("Input File name!")
    exit()


handle = u2aOpen("")
ret = 0

ret, szSerialNumber = GetSerialNumberFromHandle(handle)
print("serial %s"%(szSerialNumber.decode()))
u2aUART_Control(handle, UART_BaudRate.UART_115200_bps, UART_Parity.UART_None, UART_BitDirection.UART_LSB_First, UART_CharacterLength.UART_8_Bit, UART_StopBits.UART_One_Stop)
time.sleep(1)

u2aUART_SetMode(handle, UART_Mode.UART_Normal)

time.sleep(1)

def getc(size, timeout=1):
    global handle
    t1 = time.time()
    t2 = t1
    while (t2 - t1) < 60:
        t2 = time.time()
        ret, data = u2aUART_Read(handle, size)
        if(ret>0):
            print( "getc:%s %d"% (''.join(['%02X ' % b for b in data]), size) )
            break
    return data or None

def putc(data, timeout=60):
    global handle
    length = len(data)
    #the buf size is 54
    if length < 55:
        u2aUART_Write(handle, len(data), data)
    else:
        x = length//54
        y = length%54
        for i in range(0, x*54, 54):
            u2aUART_Write(handle, 54, data[i:i+54])
            time.sleep(0.02)
        u2aUART_Write(handle, y, data[x*54:])
    print( "send:%s %d"% (''.join(['%02X ' % b for b in data]), length) )
    return len(data)  # note that this ignores the timeout

modem = YMODEM(getc, putc, mode = 'ymodem128')

file_path = sys.argv[1]
file_stream = open(file_path, mode='rb')
file_name = os.path.basename(file_path)
file_size = os.path.getsize(file_path)
modem.send(file_stream, file_name, file_size)
# while True:
#     data = getc(1)
#     if(data==None):
#         pass
#     else:
#         print("%02X"%( data[0]))
# putc(bytes([0x9F,0x00,0x00,0x00]))
#write one byte


time.sleep(1)

u2aClose(handle)

