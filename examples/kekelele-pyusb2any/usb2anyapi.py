import ctypes as ct
from enum import Enum
from enum import IntEnum
from io import open_code

#API Version V2.8.2
OPEN_ERROR = False
handle = 0
try:
    dll = ct.windll.LoadLibrary('USB2ANY.dll')
except Exception as e:
    print(e)
    exit(1)

ErrorCodes = {
    0:"No error",
    -1:"Receiver overflowed",
    -2:"Receive buffer is empty ",
    -3:"Transmit buffer is full ",
    -4:"Transmit is stalled ",
    -5:"Transmit failed ",
    -6:"Failed to open communications port ",
    -7:"Communications port is not open ",
    -8:"Communications port is open ",
    -9:"Receive timeout ",
    -10:"Communications port read error ",
    -11:"Communications port write error",
    -12:"Communications device not found ",
    -13:"Communications CRC failed ",
    -20:"Invalid port",
    -21:"Address is out of accepted range ",
    -22:"Invalid function code ",
    -23:"Invalid packet size ",
    -24:"Invalid handle ",
    -25:"Operation failed ",
    -26:"Parameter is out of range ",
    -27:"Packet is out of sequence ",
    -28:"Invalid packet header ",
    -29:"Function not implemented ",
    -30:"Too much data ",
    -31:"Invalid device ",
    -32:"Unsupported firmware version ",
    -33:"Buffer is too small",
    -34:"No data available",
    -35:"Resource conflict",
    -36:"EVM is required for external power",
    -37:"Command is busy",
    -38:"Adjustable power supply failure",
    -39:"Interface or mode is not enabled",
    -40:"I2C initialization failed",
    -41:"I2C read error ",
    -42:"I2C write error ",
    -43:"I2C busy (transfer is pending) ",
    -44:"Address not acknowledged (NAK) ",
    -45:"Data not acknowledged (NAK) ",
    -46:"Read timeout ",
    -47:"Read data timeout ",
    -48:"Timeout waiting for read complete ",
    -49:"Write timeout ",
    -50:"Write data timeout ",
    -51:"Timeout waiting for write complete ",
    -52:"I2C not in Master mode ",
    -53:"I2C arbitration lost ",
    -54:"I2C pullups require the 3.3V EXT power to be on ",
    -60:"SPI initialization failed",
    -61:"SPI write/read error",
    -70:"Data write error",
    -71:"Data read error",
    -72:"Operation timeout",
    -73:"Data CRC failed"
}
def openError():
    global OPEN_ERROR 
    OPEN_ERROR = True
def printret(ret):
    if ret < 0 and OPEN_ERROR:
        print("Error Code=%d, %s"%(ret, ErrorCodes[ret]))

# USB2ANY_SDK_API int __stdcall u2aFindControllers();
# #if BUILD_FIRMWARE_LOADER_MODE
# USB2ANY_SDK_API int __stdcall u2aFindControllersBSL(FCB_MODE mode=FCB_LOADER_DIALOG);
# #else
# USB2ANY_SDK_API int __stdcall u2aFindControllersBSL(BOOL bEnumOnly);
# #endif
# USB2ANY_SDK_API int __stdcall u2aGetSerialNumber(int index, char *SerialNumber);
# USB2ANY_SDK_API U2A_HANDLE __stdcall u2aOpen(char *SerialNumber);
# USB2ANY_SDK_API char * __stdcall u2aGetStatusText(int code, char *buffer, int bufsize); // DEPRECATED! Use u2aStatus_GetText
# USB2ANY_SDK_API char * __stdcall u2aStatus_GetText(int code, char *buffer, int bufsize);
# USB2ANY_SDK_API int __stdcall u2aSetReceiveTimeout(int milliseconds);
# USB2ANY_SDK_API BOOL __stdcall u2aEnableDeviceDetect(BOOL enable, U2A_CONNECT_CALLBACK pfnCallback);
# USB2ANY_SDK_API BOOL __stdcall u2aEnableAPIProfiling(BOOL enable);
# USB2ANY_SDK_API void __stdcall u2aProfileTimestamp(LPSTR szMessage);
# USB2ANY_SDK_API BOOL __stdcall u2aEnableDeviceDetectMsg(BOOL enable, HWND hwnd, int msgno);
# USB2ANY_SDK_API int __stdcall u2aIsUSB2ANYConnected();
# USB2ANY_SDK_API int __stdcall u2aStatus_IsUSB2ANYConnected();
# USB2ANY_SDK_API int __stdcall u2aSetResponseMode(int mode);
# USB2ANY_SDK_API int __stdcall u2aEnableDebugLogging(BOOL enable);
# USB2ANY_SDK_API int __stdcall u2aEnablePacketLogging(BOOL enable);
# USB2ANY_SDK_API BOOL __stdcall u2aSuppressDebugLogging(BOOL state);
# USB2ANY_SDK_API BOOL __stdcall u2aSuppressSplash(BOOL suppress);
# USB2ANY_SDK_API BOOL __stdcall u2aSuppressFirmwareCheck(BOOL suppress);
# USB2ANY_SDK_API BOOL __stdcall u2aSuppressPopups(BOOL suppress);
# USB2ANY_SDK_API void __stdcall u2aLogComment(LPSTR szMessage);
# USB2ANY_SDK_API LPSTR __stdcall u2aGetSDKPath(LPSTR szPath);
# USB2ANY_SDK_API LPSTR __stdcall u2aGetDebugLogPath(LPSTR szBuffer, int len);

def u2aFindControllers():
    ret = dll.u2aFindControllers()
    printret(ret)
    return ret

def u2aOpen(SerialNumber):
    dll.u2aOpen.argtypes = [ct.c_char_p]
    ret = dll.u2aOpen(SerialNumber.encode('UTF-8'))
    printret(ret)
    return ret

# //
# // Controller functions
# //
# USB2ANY_SDK_API int __stdcall u2aClose(U2A_HANDLE handle);
# USB2ANY_SDK_API int __stdcall u2aReadResponse(U2A_HANDLE handle, BYTE *pBuffer, DWORD dwBufferSize);
# USB2ANY_SDK_API int __stdcall u2aGetSerialNumberFromHandle(U2A_HANDLE handle, LPSTR szSerialNumber, int length);
# USB2ANY_SDK_API int __stdcall GetSerialNumberFromHandle(U2A_HANDLE handle, LPSTR szSerialNumber, int length);   // deprecated!!!
# USB2ANY_SDK_API int __stdcall u2aStatus_GetControllerType(U2A_HANDLE handle);
# USB2ANY_SDK_API int __stdcall u2aSetAsyncIOCallback(U2A_HANDLE handle, void *Callback, int nFunctionID);

def u2aClose(handle):
    dll.u2aClose.argtypes = [ct.c_long]
    ret = dll.u2aClose(handle)
    printret(ret)
    return ret

def GetSerialNumberFromHandle(handle):
    dll.GetSerialNumberFromHandle.argtypes = [ct.c_long, ct.c_char_p, ct.c_int]
    szSerialNumber = ct.create_string_buffer(80)
    ret = dll.GetSerialNumberFromHandle(handle,szSerialNumber,80)
    printret(ret)
    return ret, szSerialNumber.value

# //
# // I2C functions
# //
# USB2ANY_SDK_API int __stdcall u2aI2C_Control(U2A_HANDLE handle, I2C_Speed Speed, I2C_AddressLength AddressLength, I2C_PullUps PullUps);
# USB2ANY_SDK_API int __stdcall u2aI2C_RegisterRead(U2A_HANDLE handle, UInt16 I2C_Address, Byte RegisterAddress);
# USB2ANY_SDK_API int __stdcall u2aI2C_RegisterWrite(U2A_HANDLE handle, UInt16 I2C_Address, Byte RegisterAddress, Byte Value);
# USB2ANY_SDK_API int __stdcall u2aI2C_MultiRegisterRead(U2A_HANDLE handle, UInt16 I2C_Address, Byte StartingRegisterAddress, Byte nBytes, Byte Data[]);
# USB2ANY_SDK_API int __stdcall u2aI2C_MultiRegisterWrite(U2A_HANDLE handle, UInt16 I2C_Address, Byte StartingRegisterAddress, Byte nBytes, Byte Data[]);
# USB2ANY_SDK_API int __stdcall u2aI2C_InternalRead(U2A_HANDLE handle, UInt16 I2C_Slave_Address, UInt16 InternalAddress, Byte nIntBytes, UInt16 nBytes, Byte Data[]);
# USB2ANY_SDK_API int __stdcall u2aI2C_InternalWrite(U2A_HANDLE handle, UInt16 I2C_Slave_Address, UInt16 InternalAddress, Byte nIntBytes, UInt16 nBytes, Byte Data[]);
# USB2ANY_SDK_API int __stdcall u2aI2C_RawRead(U2A_HANDLE handle, UInt16 I2C_Address, Byte nBytes, Byte Data[]);
# USB2ANY_SDK_API int __stdcall u2aI2C_RawWrite(U2A_HANDLE handle, UInt16 I2C_Address, Byte nBytes, Byte Data[]);
# USB2ANY_SDK_API int __stdcall u2aI2C_BlockWriteBlockRead(U2A_HANDLE handle, UInt16 I2C_Address, 
#         Byte nWriteBytes, Byte *WriteData, Byte nReadBytes, Byte *ReadData);

# typedef enum
# {
#     I2C_100kHz = 0,
#     I2C_400kHz = 1,
#     I2C_10kHz  = 2,
#     I2C_800kHz = 3
# } I2C_Speed;

# typedef enum
# {
#     I2C_7Bits = 0,
#     I2C_10Bits = 1
# } I2C_AddressLength;

# typedef enum
# {
#     I2C_PullUps_OFF = 0,
#     I2C_PullUps_ON = 1
# } I2C_PullUps;

class I2C_Speed(IntEnum):
    I2C_100kHz = 0
    I2C_400kHz = 1
    I2C_10kHz  = 2
    I2C_800kHz = 3
class I2C_AddressLength(IntEnum):
    I2C_7Bits = 0
    I2C_10Bits = 1
class I2C_PullUps(IntEnum):
    I2C_PullUps_OFF = 0
    I2C_PullUps_ON = 1

def u2aI2C_Control(handle, Speed, AddressLength, PullUps):
    dll.u2aI2C_Control.argtypes = [ct.c_long, ct.c_int, ct.c_int, ct.c_int]
    ret = dll.u2aI2C_Control(handle, Speed, AddressLength, PullUps)
    printret(ret)
    return ret

def u2aI2C_RegisterRead(handle, I2C_Address, RegisterAddress):
    dll.u2aI2C_RegisterRead.argtypes = [ct.c_long, ct.c_uint16, ct.c_ubyte]
    ret = dll.u2aI2C_RegisterRead(handle, I2C_Address, RegisterAddress)
    printret(ret)
    return ret

def u2aI2C_RegisterWrite(handle, I2C_Address, RegisterAddress, Value):
    dll.u2aI2C_RegisterWrite.argtypes = [ct.c_long, ct.c_uint16, ct.c_ubyte, ct.c_ubyte]
    ret = dll.u2aI2C_RegisterWrite(handle, I2C_Address, RegisterAddress, Value)
    printret(ret)
    return ret

def u2aI2C_MultiRegisterRead(handle, I2C_Address, StartingRegisterAddress, nBytes):
    dll.u2aI2C_MultiRegisterRead.argtypes = [ct.c_long, ct.c_uint16, ct.c_ubyte, ct.c_ubyte, ct.c_char_p]
    Data = ct.create_string_buffer(nBytes)
    ret = dll.u2aI2C_MultiRegisterRead(handle, I2C_Address, StartingRegisterAddress, nBytes, Data)
    printret(ret)
    return ret, Data.raw

def u2aI2C_MultiRegisterWrite(handle, I2C_Address, StartingRegisterAddress, nBytes, Data):
    dll.u2aI2C_MultiRegisterWrite.argtypes = [ct.c_long, ct.c_uint16, ct.c_ubyte, ct.c_ubyte, ct.c_char_p]
    tempData = ct.create_string_buffer(nBytes)
    tempData.raw = Data[:nBytes]
    ret = dll.u2aI2C_MultiRegisterWrite(handle, I2C_Address, StartingRegisterAddress, nBytes, tempData)
    printret(ret)
    return ret

def u2aI2C_InternalRead(handle, I2C_Slave_Address, InternalAddress, nIntBytes, nBytes):
    dll.u2aI2C_InternalRead.argtypes = [ct.c_long, ct.c_uint16, ct.c_uint16, ct.c_ubyte, ct.c_uint16, ct.c_char_p]
    Data = ct.create_string_buffer(nBytes)
    ret = dll.u2aI2C_InternalRead(handle, I2C_Slave_Address, InternalAddress, nIntBytes, nBytes, Data)
    printret(ret)
    return ret, Data.raw

def u2aI2C_InternalWrite(handle, I2C_Slave_Address, InternalAddress, nIntBytes, nBytes, Data):
    dll.u2aI2C_InternalWrite.argtypes = [ct.c_long, ct.c_uint16, ct.c_uint16, ct.c_ubyte, ct.c_uint16, ct.c_char_p]
    tempData = ct.create_string_buffer(nBytes)
    tempData.raw = Data[:nBytes]
    ret = dll.u2aI2C_InternalWrite(handle, I2C_Slave_Address, InternalAddress, nIntBytes, nBytes, tempData)
    printret(ret)
    return ret

def u2aI2C_RawRead(handle, I2C_Address, nBytes):
    dll.u2aI2C_RawRead.argtypes = [ct.c_long, ct.c_uint16, ct.c_ubyte, ct.c_char_p]
    Data = ct.create_string_buffer(nBytes)
    ret = dll.u2aI2C_RawRead(handle, I2C_Address, nBytes, Data)
    printret(ret)
    return ret, Data.raw

def u2aI2C_RawWrite(handle, I2C_Address, nBytes, Data):
    dll.u2aI2C_RawWrite.argtypes = [ct.c_long, ct.c_uint16, ct.c_ubyte, ct.c_char_p]
    tempData = ct.create_string_buffer(nBytes)
    tempData.raw = Data[:nBytes]
    ret = dll.u2aI2C_RawWrite(handle, I2C_Address, nBytes, tempData)
    printret(ret)
    return ret

def u2aI2C_BlockWriteBlockRead(handle, I2C_Address, nWriteBytes, WriteData, nReadBytes):
    dll.u2aI2C_BlockWriteBlockRead.argtypes = [ct.c_long, ct.c_uint16, ct.c_ubyte, ct.c_char_p, ct.c_ubyte, ct.c_char_p]
    tempData = ct.create_string_buffer(nWriteBytes)
    tempData.raw = WriteData[:nWriteBytes]
    ReadData = ct.create_string_buffer(nReadBytes)
    ret = dll.u2aI2C_BlockWriteBlockRead(handle, I2C_Address, nWriteBytes, tempData, nReadBytes, ReadData)
    printret(ret)
    return ret, ReadData.raw

# //
# // SPI functions
# //
# USB2ANY_SDK_API int __stdcall u2aSPI_Control(U2A_HANDLE handle, 
#         SPI_ClockPhase _SPI_ClockPhase,
#         SPI_ClockPolarity _SPI_ClockPolarity,
#         SPI_BitDirection _SPI_BitDirection,
#         SPI_CharacterLength _SPI_CharacterLength,
#         SPI_ChipSelectType _SPI_LatchType,
#         SPI_LatchPolarity _SPI_LatchPolarity,
#         Byte _DividerHigh,
#         Byte _DividerLow);

# USB2ANY_SDK_API int __stdcall u2aSPI_WriteAndRead(U2A_HANDLE handle, Byte nBytes, Byte Data[]);
# USB2ANY_SDK_API int __stdcall u2aSPI_WriteAndReadEx(U2A_HANDLE handle, Byte nCS, Byte nBytes, Byte Data[]);

# typedef enum 
# {
#     SPI_Capture_On_Trailing_Edge = 0,
#     SPI_Change_On_First_Edge = 0,       // deprecated
#     SPI_Capture_On_Leading_Edge = 1,
#     SPI_Change_On_Following_Edge = 1,   // deprecated
# } SPI_ClockPhase;
# typedef enum 
# {
#     SPI_Inactive_State_Low = 0,
#     SPI_Inactive_State_High = 1,
# } SPI_ClockPolarity;

# typedef enum 
# {
#     SPI_LSB_First = 0,
#     SPI_MSB_First = 1,
# } SPI_BitDirection;

# typedef enum 
# {
#     SPI__8_Bit = 0,
#     SPI__7_Bit = 1,
# } SPI_CharacterLength;

# typedef enum 
# {
#     SPI_With_Every_Byte = 0,
#     SPI_With_Every_Packet = 1,
#     SPI_With_Every_Word = 2,
#     SPI_No_CS = 3,
#     SPI_MOSI_High_When_Idle = 4,
#     SPI_Pulse_After_Packet = 255,
# } SPI_ChipSelectType;

# typedef enum 
# {
#     SPI_CS_Active_Low = 0,
#     SPI_Low_To_High = 0,        // deprecated
#     SPI_CS_Active_High = 1,
#     SPI_High_To_Low = 1,        // deprecated
# } SPI_ChipSelectPolarity;
# typedef enum 
# {
#     SPI_With_Every_Byte = 0,
#     SPI_With_Every_Packet = 1,
#     SPI_With_Every_Word = 2,
#     SPI_No_CS = 3,
#     SPI_MOSI_High_When_Idle = 4,
#     SPI_Pulse_After_Packet = 255,
# } SPI_ChipSelectType;
class SPI_ClockPhase(IntEnum):
    SPI_Capture_On_Trailing_Edge = 0
    SPI_Change_On_First_Edge = 0
    SPI_Capture_On_Leading_Edge = 1
    SPI_Change_On_Following_Edge = 1
class SPI_ClockPolarity(IntEnum):
    SPI_Inactive_State_Low = 0
    SPI_Inactive_State_High = 1
class SPI_BitDirection(IntEnum):
    SPI_LSB_First = 0
    SPI_MSB_First = 1
class SPI_CharacterLength(IntEnum):
    SPI__8_Bit = 0
    SPI__7_Bit = 1
class SPI_ChipSelectType(IntEnum):
    SPI_With_Every_Byte = 0
    SPI_With_Every_Packet = 1
    SPI_With_Every_Word = 2
    SPI_No_CS = 3
    SPI_MOSI_High_When_Idle = 4
    SPI_Pulse_After_Packet = 255
class SPI_ChipSelectPolarity(IntEnum):
    SPI_Active_High = 0
    SPI_Active_Low = 1
class SPI_BitRate(IntEnum):
    SPI_BitRate_10K = 10
    SPI_BitRate_25K = 25
    SPI_BitRate_50K = 50
    SPI_BitRate_100K = 100
    SPI_BitRate_125K = 125
    SPI_BitRate_200K = 200
    SPI_BitRate_250K = 250
    SPI_BitRate_400K = 400
    SPI_BitRate_500K = 500
    SPI_BitRate_800K = 800
    SPI_BitRate_1000K = 1000
    SPI_BitRate_2000K = 2000
    SPI_BitRate_4000K = 4000
    SPI_BitRate_8000K = 8000
def u2aSPI_Control(handle, _SPI_ClockPhase, _SPI_ClockPolarity, _SPI_BitDirection, _SPI_CharacterLength, _SPI_CSType, _SPI_CSPolarity, _SPI_BitRate):
    # Bit Rate (kbps)   Divider     _DividerHigh    _DividerLow
    # 10                2400        9               96
    # 25                960         3               192
    # 50                480         1               224
    # 100               240         0               240
    # 125               192         0               192
    # 200               120         0               120
    # 250               96          0               96
    # 400               60          0               60
    # 500               48          0               48
    # 800               30          0               30
    # 1000              24          0               24
    # 2000              12          0               12
    # 4000              6           0               6
    # 8000              3           0               3
    divider = 24000 // _SPI_BitRate
    _DividerHigh = (divider >> 8) & 0xFF
    _DividerLow = divider & 0xFF
    dll.u2aSPI_Control.argtypes = [ct.c_long, ct.c_int, ct.c_int, ct.c_int, ct.c_int, ct.c_int, ct.c_int, ct.c_ubyte, ct.c_ubyte]
    ret = dll.u2aSPI_Control(handle, _SPI_ClockPhase, _SPI_ClockPolarity, _SPI_BitDirection, _SPI_CharacterLength, _SPI_CSType, _SPI_CSPolarity, _DividerHigh, _DividerLow)
    printret(ret)
    return ret
def u2aSPI_WriteAndRead(handle, nBytes, Data):
    dll.u2aSPI_WriteAndRead.argtypes = [ct.c_long, ct.c_ubyte, ct.c_char_p]
    tempData = ct.create_string_buffer(nBytes)
    tempData.raw = Data[:nBytes]
    ret = dll.u2aSPI_WriteAndRead(handle, nBytes, tempData)
    printret(ret)
    return ret, tempData.raw
def u2aSPI_WriteAndReadEx(handle, nSS, nBytes, Data):
    dll.u2aSPI_WriteAndReadEx.argtypes = [ct.c_long, ct.c_ubyte, ct.c_ubyte, ct.c_char_p]
    tempData = ct.create_string_buffer(nBytes)
    tempData.raw = Data[:nBytes]
    ret = dll.u2aSPI_WriteAndReadEx(handle, nSS, nBytes, tempData)
    printret(ret)
    return ret, tempData.raw

# //
# // UART functions
# //
# USB2ANY_SDK_API int __stdcall u2aUART_Control(U2A_HANDLE handle, 
#         UINT _UART_BaudRate,
#         UINT _UART_Parity,
#         UINT _UART_BitDirection,
#         UINT _UART_CharacterLength,
#         UINT _UART_StopBits);

# USB2ANY_SDK_API int __stdcall u2aUART_Write(U2A_HANDLE handle, Byte nBytes, Byte Data[]);
# USB2ANY_SDK_API int __stdcall u2aUART_Read(U2A_HANDLE handle, Byte nBytes, Byte Data[]);
# USB2ANY_SDK_API int __stdcall u2aUART_DisableReceiver(U2A_HANDLE handle);
# USB2ANY_SDK_API int __stdcall u2aUART_GetRxCount(U2A_HANDLE handle);
# USB2ANY_SDK_API int __stdcall u2aUART_SetMode(U2A_HANDLE handle, UINT mode);
# typedef enum 
# {
#     UART_9600_bps   = 0,
#     UART_19200_bps  = 1,
#     UART_38400_bps  = 2,
#     UART_57600_bps  = 3,
#     UART_115200_bps = 4,
#     UART_230400_bps = 5,
#     UART_300_bps    = 6,
#     UART_320_bps    = 7,
#     UART_600_bps    = 8,
#     UART_1200_bps   = 9,
#     UART_2400_bps   = 10,
#     UART_4800_bps   = 11,
#     UART_Disable    = 99,
# } UART_BaudRate;

# typedef enum 
# {
#     UART_None = 0,
#     UART_Even = 1,
#     UART_Odd = 2,
# } UART_Parity;

# typedef enum 
# {
#     UART_LSB_First = 0,
#     UART_MSB_First = 1,
# } UART_BitDirection;

# typedef enum 
# {
#     UART_8_Bit = 0,
#     UART_7_Bit = 1,
# } UART_CharacterLength;

# typedef enum 
# {
#     UART_One_Stop = 0,
#     UART_Two_Stop = 1,
# } UART_StopBits;

# typedef enum 
# {
#     UART_Normal = 0,
#     UART_ReceiverOff = 1,
#     UART_RecvAfterXmit = 2,
# } UART_Mode;

class UART_BaudRate(IntEnum):
    UART_9600_bps   = 0
    UART_19200_bps  = 1
    UART_38400_bps  = 2
    UART_57600_bps  = 3
    UART_115200_bps = 4
    UART_230400_bps = 5
    UART_300_bps    = 6
    UART_320_bps    = 7
    UART_600_bps    = 8
    UART_1200_bps   = 9
    UART_2400_bps   = 10
    UART_4800_bps   = 11
    UART_Disable    = 99

class UART_Parity(IntEnum):
    UART_None = 0
    UART_Even = 1
    UART_Odd = 2

class UART_BitDirection(IntEnum):
    UART_LSB_First = 0
    UART_MSB_First = 1

class UART_CharacterLength(IntEnum):
    UART_8_Bit = 0
    UART_7_Bit = 1

class UART_StopBits(IntEnum):
    UART_One_Stop = 0
    UART_Two_Stop = 1

class UART_Mode(IntEnum):
    UART_Normal = 0
    UART_ReceiverOff = 1
    UART_RecvAfterXmit = 2

def u2aUART_Control(handle, _UART_BaudRate, _UART_Parity, _UART_BitDirection, _UART_CharacterLength, _UART_StopBits):
    dll.u2aUART_Control.argtypes = [ct.c_long, ct.c_uint, ct.c_uint, ct.c_uint, ct.c_uint, ct.c_uint]
    ret = dll.u2aUART_Control(handle, _UART_BaudRate, _UART_Parity, _UART_BitDirection, _UART_CharacterLength, _UART_StopBits)
    printret(ret)
    return ret

def u2aUART_Write(handle, nBytes, Data):
    dll.u2aUART_Write.argtypes = [ct.c_long, ct.c_ubyte, ct.c_char_p]
    tempData = ct.create_string_buffer(nBytes)
    tempData.raw = Data[:nBytes]
    ret = dll.u2aUART_Write(handle, nBytes, tempData)
    printret(ret)
    return ret

def u2aUART_Read(handle, nBytes):
    dll.u2aUART_Read.argtypes = [ct.c_long, ct.c_ubyte, ct.c_char_p]
    Data = ct.create_string_buffer(nBytes)
    ret = dll.u2aUART_Read(handle, nBytes, Data)
    printret(ret)
    if(ret > 0):
        read = Data.raw
    else:
        read = bytes()
    return ret, bytes(read)

def u2aUART_DisableReceiver(handle):
    dll.u2aUART_DisableReceiver.argtypes = [ct.c_long]
    ret = dll.u2aUART_DisableReceiver(handle)
    printret(ret)
    return ret

def u2aUART_GetRxCount(handle):
    dll.u2aUART_GetRxCount.argtypes = [ct.c_long]
    ret = dll.u2aUART_GetRxCount(handle)
    printret(ret)
    return ret

def u2aUART_SetMode(handle, mode):
    dll.u2aUART_SetMode.argtypes = [ct.c_long, ct.c_uint]
    ret = dll.u2aUART_Read(handle, mode)
    printret(ret)
    return ret

# //
# // Miscellaneous functions
# //
# USB2ANY_SDK_API int __stdcall u2aFirmwareVersion_Read(U2A_HANDLE handle, BYTE *szVersion, int bufsize);
# USB2ANY_SDK_API int __stdcall u2aLED_WriteControl(U2A_HANDLE handle, LED _LED);
# USB2ANY_SDK_API int __stdcall u2aLED_SetState(U2A_HANDLE handle, int LEDState, int BlinkCode);
# USB2ANY_SDK_API int __stdcall u2aClock_Control (U2A_HANDLE handle, ClockDivider1 _ClockDivider1, ClockDivider2 _ClockDivider2);
# USB2ANY_SDK_API int __stdcall u2aGetErrorList(U2A_HANDLE handle, int error_list[], int list_size);
# USB2ANY_SDK_API int __stdcall u2aDigital_Capture(U2A_HANDLE handle, UInt32 frequency, UInt16 timeframe);
# USB2ANY_SDK_API int __stdcall u2aDigital_GetData(U2A_HANDLE handle, UInt16 nBytes, BYTE *buffer);
# USB2ANY_SDK_API int __stdcall u2aDigital_GetStatus(U2A_HANDLE handle, BYTE *buffer);
# USB2ANY_SDK_API int __stdcall u2aPegasus_Test(U2A_HANDLE handle, BYTE address, BYTE value);

# typedef enum 
# {
#     LED_OFF = 0,
#     LED_ON = 1,
#     LED_TOGGLE = 2
# } LED;
class LED(IntEnum):
    LED_OFF = 0
    LED_ON = 1
    LED_TOGGLE = 2

def u2aLED_WriteControl(handle, _LED):
    dll.u2aLED_WriteControl.argtypes = [ct.c_long, ct.c_int]
    ret = dll.u2aLED_WriteControl(handle, _LED)
    printret(ret)
    return ret

# //
# // Power functions
# //
# USB2ANY_SDK_API int __stdcall u2aPower_Notify(U2A_HANDLE handle, BOOL enable, void *callback);
# USB2ANY_SDK_API int __stdcall u2aPower_SetVoltageRef(U2A_HANDLE handle, UInt16 Millivolts);
# USB2ANY_SDK_API int __stdcall u2aPower_Enable(U2A_HANDLE handle, int Enable3V3, int Enable5v0, int EnableAdj);
# USB2ANY_SDK_API int __stdcall u2aPower_WriteControl(U2A_HANDLE handle, Power_3V3 _Power_3V3, Power_5V0 _Power_5V0);
# USB2ANY_SDK_API int __stdcall u2aPower_ReadStatus(U2A_HANDLE handle);

# typedef enum 
# {
#     Power_3V3_OFF = 0,
#     Power_3V3_ON = 1
# } Power_3V3;

# typedef enum 
# {
#     Power_5V0_OFF = 0,
#     Power_5V0_ON = 1
# } Power_5V0;
class Power_3V3(IntEnum):
    Power_3V3_OFF = 0
    Power_3V3_ON = 1

class Enable_3V3(IntEnum):
    Power_3V3_Disable = 0
    Power_3V3_Enable = 1 
    Power_3V3_Ignore = 2

class Power_5V0(IntEnum):
    Power_5V0_OFF = 0
    Power_5V0_ON = 1

class Enable_5V0(IntEnum):
    Power_5V0_Disable = 0
    Power_5V0_Enable = 1 
    Power_5V0_Ignore = 2

class Enable_ADJ(IntEnum):
    Power_ADJ_Disable = 0
    Power_ADJ_Enable = 1 
    Power_ADJ_Ignore = 2

def u2aPower_Enable(handle, Enable3V3, Enable5v0, EnableAdj):
    dll.u2aPower_Enable.argtypes = [ct.c_long, ct.c_int, ct.c_int, ct.c_int]
    ret = dll.u2aPower_Enable(handle, Enable3V3, Enable5v0, EnableAdj)
    printret(ret)
    return ret

def u2aPower_WriteControl(handle, _Power_3V3, _Power_5V0):
    dll.u2aPower_WriteControl.argtypes = [ct.c_long, ct.c_int, ct.c_int]
    ret = dll.u2aPower_WriteControl(handle, _Power_3V3, _Power_5V0)
    printret(ret)
    return ret
    

#========================================================================================================+
def bytesToHexString(bs):
    # hex_str = ''
    # for item in bs:
    #     hex_str += str(hex(item))[2:].zfill(2).upper() + " "
    # return hex_str
    return ''.join(['%02X ' % b for b in bs])

