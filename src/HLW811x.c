/**
 **********************************************************************************
 * @file   HLW811x.c
 * @author Hossein.M (https://github.com/Hossein-M98)
 * @brief  HLW8110 and HLW8112 driver
 **********************************************************************************
 *
 * Copyright (c) 2024 Mahda Embedded System (MIT License)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 **********************************************************************************
 */

/* Includes ---------------------------------------------------------------------*/
#include "HLW811x.h"
#include "HLW811x_regs.h"


/* Private Constants ------------------------------------------------------------*/
/**
 * @brief  Commands
 */
#define HLW811X_COMMAND_ADDRESS       0xEA
#define HLW811X_COMMAND_WRITE_ENABLE  0xE5
#define HLW811X_COMMAND_WRITE_CLOSE   0xDC
#define HLW811X_COMMAND_CHANNELA      0x5A
#define HLW811X_COMMAND_CHANNELB      0xA5
#define HLW811X_COMMAND_RESET         0x96



/**
 ==================================================================================
                       ##### Private Functions #####
 ==================================================================================
 */

#if (HLW811X_CONFIG_SUPPORT_SPI)
static inline int8_t
HLW811x_WriteRegSPI(HLW811x_Handler_t *Handler,
                    uint8_t Address, uint8_t Size, uint8_t *Data)
{
  uint8_t Buffer[5] = {0};
  int8_t Result = 0;

  if (Address == 0xEA) // Special address for commands
    Buffer[0] = Address;
  else
    Buffer[0] = Address | 0x80;

  if (Size > 4)
    return -1;
  for (uint8_t i = 0; i < Size; i++)
    Buffer[i + 1] = Data[i];

  Handler->Platform.SPI.SetLevelSCSN(0);
  Result = Handler->Platform.SPI.SendReceive(Buffer, (uint8_t *)0, Size + 1);
  Handler->Platform.SPI.SetLevelSCSN(1);

  return Result;
}

static inline int8_t
HLW811x_ReadRegSPI(HLW811x_Handler_t *Handler,
                   uint8_t Address, uint8_t Size, uint8_t *Data)
{
  uint8_t BufferTx[5] = {0xFF};
  uint8_t BufferRx[5] = {0};
  int8_t Result = 0;

  if (Address == 0xEA) // Special address for commands
    return -1;
  else
    BufferTx[0] = Address & 0x7F;

  if (Size > 4)
    return -1;

  Handler->Platform.SPI.SetLevelSCSN(0);
  Result = Handler->Platform.SPI.SendReceive(BufferTx, BufferRx, Size + 1);
  Handler->Platform.SPI.SetLevelSCSN(1);

  for (uint8_t i = 0; i < Size; i++)
    Data[i] = BufferRx[i + 1];

  return Result;
}
#endif

#if (HLW811X_CONFIG_SUPPORT_UART)
static inline int8_t
HLW811x_WriteRegUART(HLW811x_Handler_t *Handler,
                     uint8_t Address, uint8_t Size, uint8_t *Data)
{
  uint8_t Buffer[7] = {0};
  uint8_t Checksum = 0;
  int8_t Result = 0;

  Buffer[0] = 0xA5;
  Checksum += Buffer[0];

  if (Address == 0xEA) // Special address for commands
    Buffer[1] = Address;
  else
    Buffer[1] = Address | 0x80;

  Checksum += Buffer[1];

  if (Size > 4)
    return -1;
  for (uint8_t i = 0; i < Size; i++)
  {
    Buffer[i + 2] = Data[i];
    Checksum += Buffer[i + 2];
  }

  Checksum = ~Checksum;
  Buffer[Size + 2] = Checksum;
  Result = Handler->Platform.UART.Send(Buffer, Size + 3);

  return Result;
}

static inline int8_t
HLW811x_ReadRegUART(HLW811x_Handler_t *Handler,
                    uint8_t Address, uint8_t Size, uint8_t *Data)
{
  uint8_t Buffer[5] = {0};
  uint8_t Checksum = 0;
  int8_t Result = 0;

  Buffer[0] = 0xA5;
  Checksum += Buffer[0];

  if (Address == 0xEA) // Special address for commands
    return -1;
  else
    Buffer[1] = Address & 0x7F;
  Checksum += Buffer[1];

  if (Size > 4)
    return -1;

  Result = Handler->Platform.UART.Send(Buffer, 2);
  if (Result < 0)
    return Result;

  Result = Handler->Platform.UART.Receive(Buffer, Size + 1);
  if (Result < 0)
    return Result;

  for (uint8_t i = 0; i < Size; i++)
  {
    Data[i] = Buffer[i];
    Checksum += Buffer[i];
  }

  Checksum = ~Checksum;
  if (Checksum != Buffer[Size])
    return -100;

  return Result;
}
#endif

static int8_t
HLW811x_WriteReg(HLW811x_Handler_t *Handler,
                 uint8_t Address, uint8_t Size, uint8_t *Data)
{
#if (HLW811X_CONFIG_SUPPORT_SPI && HLW811X_CONFIG_SUPPORT_UART)
  if (Handler->Platform.Communication == HLW811X_COMMUNICATION_SPI)
    return HLW811x_WriteRegSPI(Handler, Address, Size, Data);
  else
    return HLW811x_WriteRegUART(Handler, Address, Size, Data);
#elif (HLW811X_CONFIG_SUPPORT_SPI)
  return HLW811x_WriteRegSPI(Handler, Address, Size, Data);
#elif (HLW811X_CONFIG_SUPPORT_UART)
  return HLW811x_WriteRegUART(Handler, Address, Size, Data);
#endif

  return -1;
}

static int8_t
HLW811x_ReadReg(HLW811x_Handler_t *Handler,
                uint8_t Address, uint8_t Size, uint8_t *Data)
{
#if (HLW811X_CONFIG_SUPPORT_SPI && HLW811X_CONFIG_SUPPORT_UART)
  if (Handler->Platform.Communication == HLW811X_COMMUNICATION_SPI)
    return HLW811x_ReadRegSPI(Handler, Address, Size, Data);
  else
    return HLW811x_ReadRegUART(Handler, Address, Size, Data);
#elif (HLW811X_CONFIG_SUPPORT_SPI)
  return HLW811x_ReadRegSPI(Handler, Address, Size, Data);
#elif (HLW811X_CONFIG_SUPPORT_UART)
  return HLW811x_ReadRegUART(Handler, Address, Size, Data);
#endif

  return -1;
}

static int8_t
HLW811x_Command(HLW811x_Handler_t *Handler, uint8_t Command)
{
  uint8_t Buffer = Command;
  return HLW811x_WriteReg(Handler, HLW811X_COMMAND_ADDRESS, 1, &Buffer);
}

static inline int8_t
HLW811x_CommandEnableWriteOperation(HLW811x_Handler_t *Handler)
{
  return HLW811x_Command(Handler, HLW811X_COMMAND_WRITE_ENABLE);
}

static inline int8_t
HLW811x_CommandCloseWriteOperation(HLW811x_Handler_t *Handler)
{
  return HLW811x_Command(Handler, HLW811X_COMMAND_WRITE_CLOSE);
}

static inline int8_t
HLW811x_CommandChannel(HLW811x_Handler_t *Handler, uint8_t Channel)
{
  if (Channel == 0)
    return HLW811x_Command(Handler, HLW811X_COMMAND_CHANNELA);
  return HLW811x_Command(Handler, HLW811X_COMMAND_CHANNELB);
}

static inline int8_t
HLW811x_CommandReset(HLW811x_Handler_t *Handler)
{
  return HLW811x_Command(Handler, HLW811X_COMMAND_RESET);
}



/**
 ==================================================================================
                            ##### Public Functions #####
 ==================================================================================
 */

/**
 * @brief  Initializer function
 * @param  Handler: Pointer to handler
 * @param  Device: Device type
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 *         - HLW811X_INVALID_PARAM: One of parameters is invalid.
 */
HLW811x_Result_t
HLW811x_Init(HLW811x_Handler_t *Handler, HLW811x_Device_t Device)
{
  if (!Handler)
    return HLW811X_INVALID_PARAM;

  if (Device != HLW811X_DEVICE_HLW8110 && Device != HLW811X_DEVICE_HLW8112)
    return HLW811X_INVALID_PARAM;
  Handler->Device = Device;

#if (HLW811X_CONFIG_SUPPORT_SPI && HLW811X_CONFIG_SUPPORT_UART)
  switch (Handler->Platform.Communication)
  {
  case HLW811X_COMMUNICATION_SPI:
    if (Device == HLW811X_DEVICE_HLW8110)
    {
      return HLW811X_INVALID_PARAM;
    }
    else
    {
      if (!Handler->Platform.SPI.SendReceive || !Handler->Platform.SPI.SetLevelSCSN)
        return HLW811X_INVALID_PARAM;
    }
    break;

  case HLW811X_COMMUNICATION_UART:
    if (!Handler->Platform.UART.Send || !Handler->Platform.UART.Receive)
      return HLW811X_INVALID_PARAM;
    break;

  default:
    return HLW811X_INVALID_PARAM;
    break;
  }
#elif (HLW811X_CONFIG_SUPPORT_SPI)
  if (Device == HLW811X_DEVICE_HLW8110)
    return HLW811X_INVALID_PARAM;
  else if (!Handler->Platform.SPI.SendReceive || !Handler->Platform.SPI.SetLevelSCSN)
    return HLW811X_INVALID_PARAM;
#elif (HLW811X_CONFIG_SUPPORT_UART)
  if (!Handler->Platform.UART.Send || !Handler->Platform.UART.Receive)
    return HLW811X_INVALID_PARAM;
#endif

  if (Handler->Platform.Init)
  {
    if (Handler->Platform.Init() < 0)
      return HLW811X_FAIL;
  }

  if (HLW811x_CommandReset(Handler) < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Deinitialize function
 * @param  Handler: Pointer to handler
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_DeInit(HLW811x_Handler_t *Handler)
{
  if (!Handler)
    return HLW811X_INVALID_PARAM;

  if (Handler->Platform.DeInit)
    return ((Handler->Platform.DeInit() >= 0) ? HLW811X_OK : HLW811X_FAIL);

  return HLW811X_OK;
}


/**
 * @brief  Low level read register function
 * @param  Handler: Pointer to handler
 * @param  RegAddr: Register address
 * @param  Data: Pointer to buffer to store data
 * @param  Len: Register data length in bytes
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_ReadRegLL(HLW811x_Handler_t *Handler,
                  uint8_t RegAddr,
                  uint8_t *Data,
                  uint8_t Len)
{
  if (!Handler || !Data)
    return HLW811X_INVALID_PARAM;

  if (Len == 0)
    return HLW811X_INVALID_PARAM;

  return ((HLW811x_ReadReg(Handler, RegAddr, Len, Data) >= 0) ? HLW811X_OK : HLW811X_FAIL);
}


/**
 * @brief  Low level write register function
 * @param  Handler: Pointer to handler
 * @param  RegAddr: Register address
 * @param  Data: Pointer to data to write
 * @param  Len: Register data length in bytes
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_WriteRegLL(HLW811x_Handler_t *Handler,
                   uint8_t RegAddr,
                   uint8_t *Data,
                   uint8_t Len)
{
  int8_t Result = 0;

  if (!Handler || !Data)
    return HLW811X_INVALID_PARAM;

  if (Len == 0)
    return HLW811X_INVALID_PARAM;

  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg(Handler, RegAddr, Len, Data);
  if (Result < 0)
    return HLW811X_FAIL;
  
  return HLW811x_CommandCloseWriteOperation(Handler);
}
