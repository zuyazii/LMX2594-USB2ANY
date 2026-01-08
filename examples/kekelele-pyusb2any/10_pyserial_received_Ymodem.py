import serial #导入模块
import os
from ymodem import YMODEM
import time
#端口，GNU / Linux上的/ dev / ttyUSB0 等 或 Windows上的 COM3 等
portx="COM3"
#波特率，标准值之一：50,75,110,134,150,200,300,600,1200,1800,2400,4800,9600,19200,38400,57600,115200
bps=115200
#超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
timex=5
# 打开串口，并得到串口对象
ser=serial.Serial(portx,bps)
ser.bytesize = 8
ser.parity = serial.PARITY_NONE
ser.stopbits = 1
ser.timeout = 0.5

print("串口详情参数：", ser)

def getc(size, timeout=1):
    
    data = ser.read(size)
    #if(len(data)>0):
        #print( "getc:%s %d"% (''.join(['%02X ' % b for b in data]), len(data) ))
    return data or None

def putc(data, timeout=1):
    #print( "send:%s"% (''.join(['%02X ' % b for b in data])) )
    return ser.write(data)  # note that this ignores the timeout
modem = YMODEM(getc, putc, mode = 'ymodem128')

def save(filename, stream):
    file_path = os.path.abspath(filename)
    try:
        f = open(file_path, 'wb')
        f.write(stream)
        f.close()
    except IOError as e:
        f.close()
        raise Exception("Failed to open file!")
    

try:
    while True:
        t1 = time.time()
        file_stream, file_name, file_size = modem.recv()
        t2 = time.time()
        save(file_name, file_stream[:file_size])
        print("write to file! %d"%(t2-t1))
except Exception as e:
    ser.close()#关闭串口
    raise
# # while True:
# #     data = getc(1)
# #     if(data==None):
# #         pass
# #     else:
# #         print("%02X"%( data[0]))
# putc(bytes([0x9F,0x00,0x00,0x00]))



