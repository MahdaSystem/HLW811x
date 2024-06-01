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
 * @brief  CoefReg coefficients
 */
#define HLW811X_VREF                  1.25f

/**
 * @brief  PGA coefficients
 */
static const uint8_t HLW811X_PGA_TABLE[5] = {0x00, 0x01, 0x02, 0x03, 0x04};



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
HLW811x_WriteReg16(HLW811x_Handler_t *Handler,
                   uint8_t Address, uint16_t Data)
{
  uint8_t Buffer[2] = {0};
  
  Buffer[0] = (Data >> 8) & 0xFF;
  Buffer[1] = Data & 0xFF;

  return HLW811x_WriteReg(Handler, Address, 2, Buffer);
}

// static int8_t
// HLW811x_WriteReg24(HLW811x_Handler_t *Handler,
//                    uint8_t Address, uint32_t Data)
// {
//   uint8_t Buffer[3] = {0};
  
//   Buffer[0] = (Data >> 16) & 0xFF;
//   Buffer[1] = (Data >> 8) & 0xFF;
//   Buffer[2] = Data & 0xFF;

//   return HLW811x_WriteReg(Handler, Address, 3, Buffer);
// }

// static int8_t
// HLW811x_WriteReg32(HLW811x_Handler_t *Handler,
//                    uint8_t Address, uint32_t Data)
// {
//   uint8_t Buffer[4] = {0};
  
//   Buffer[0] = (Data >> 24) & 0xFF;
//   Buffer[1] = (Data >> 16) & 0xFF;
//   Buffer[2] = (Data >> 8) & 0xFF;
//   Buffer[3] = Data & 0xFF;

//   return HLW811x_WriteReg(Handler, Address, 4, Buffer);;
// }

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
HLW811x_ReadReg16(HLW811x_Handler_t *Handler,
                  uint8_t Address, uint16_t *Data)
{
  uint8_t Buffer[2] = {0};
  int8_t Result = 0;
  
  Result = HLW811x_ReadReg(Handler, Address, 2, Buffer);
  if (Result < 0)
    return Result;

  *Data = ((uint16_t)Buffer[0] << 8) | ((uint16_t)Buffer[1]);
  return Result;
}

static int8_t
HLW811x_ReadReg24(HLW811x_Handler_t *Handler,
                  uint8_t Address, uint32_t *Data)
{
  uint8_t Buffer[3] = {0};
  int8_t Result = 0;
  
  Result = HLW811x_ReadReg(Handler, Address, 3, Buffer);
  if (Result < 0)
    return Result;

  *Data = ((uint32_t)Buffer[0] << 16) | ((uint32_t)Buffer[1] << 8) |
          ((uint32_t)Buffer[2]);
  return Result;
}

static int8_t
HLW811x_ReadReg32(HLW811x_Handler_t *Handler,
                  uint8_t Address, uint32_t *Data)
{
  uint8_t Buffer[4] = {0};
  int8_t Result = 0;
  
  Result = HLW811x_ReadReg(Handler, Address, 4, Buffer);
  if (Result < 0)
    return Result;

  *Data = ((uint32_t)Buffer[0] << 24) | ((uint32_t)Buffer[1] << 16) |
          ((uint32_t)Buffer[2] << 8)  | ((uint32_t)Buffer[3]);
  return Result;
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

// static int32_t
// HLW811x_24BitTo32Bit(uint8_t *Data)
// {
//   int32_t Res = 0;

//   // convert three bytes of 2s complement into 24 bit signed integer
//   if (Data[0] & 0x80)
//     Res = 0xFF;
//   Res = (Res << 8) | Data[0];
//   Res = (Res << 8) | Data[1];
//   Res = (Res << 8) | Data[2];

//   // Res is now a 32-bit signed integer
//   return Res;
// }



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

  if (!Handler->Platform.DelayMs)
    return HLW811X_INVALID_PARAM;

  if (Handler->Platform.Init)
  {
    if (Handler->Platform.Init() < 0)
      return HLW811X_FAIL;
  }

  if (HLW811x_CommandReset(Handler) < 0)
    return HLW811X_FAIL;

  Handler->Platform.DelayMs(10);

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
  
  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Begin function to initialize the device and set the default values
 * @note   This function must be called after initializing platform dependent and
 *         HLW811x_Init functions.
 * 
 * @note   This function will read the coefficients from the device. It is
 *         mandatory to call this function before using other functions such as
 *         HLW811x_GetRmsU, HLW811x_GetRmsIx and etc.
 * 
 * @param  Handler: Pointer to handler
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_Begin(HLW811x_Handler_t *Handler)
{
  int8_t Result = 0;
  uint16_t Reg16 = 0;
  uint16_t Checksum = 0;

  Handler->ResCoef.KU = 1.0f;
  Handler->ResCoef.KIA = 1.0f;

  Handler->PGA.U = HLW811X_PGA_1;
  Handler->PGA.IA = HLW811X_PGA_16;
  Handler->PGA.IB = HLW811X_PGA_1;

  Handler->CLKI = 3579545;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_HFConst, &Reg16);
  if (Result < 0)
    return HLW811X_FAIL;
  Handler->HFconst = Reg16;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_RmsIAC, &Reg16);
  if (Result < 0)
    return HLW811X_FAIL;
  Handler->CoefReg.RmsIAC = Reg16;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_RmsIBC, &Reg16);
  if (Result < 0)
    return HLW811X_FAIL;
  Handler->CoefReg.RmsIBC = Reg16;
  
  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_RmsUC, &Reg16);
  if (Result < 0)
    return HLW811X_FAIL;
  Handler->CoefReg.RmsUC = Reg16;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_PowerPAC, &Reg16);
  if (Result < 0)
    return HLW811X_FAIL;
  Handler->CoefReg.PowerPAC = Reg16;
  
  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_PowerPBC, &Reg16);
  if (Result < 0)
    return HLW811X_FAIL;
  Handler->CoefReg.PowerPBC = Reg16;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_PowerSC, &Reg16);
  if (Result < 0)
    return HLW811X_FAIL;
  Handler->CoefReg.PowerSC = Reg16;
  
  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_EnergyAC, &Reg16);
  if (Result < 0)
    return HLW811X_FAIL;
  Handler->CoefReg.EnergyAC = Reg16;
  
  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_EnergyBC, &Reg16);
  if (Result < 0)
    return HLW811X_FAIL;
  Handler->CoefReg.EnergyBC = Reg16;

  Checksum = 0xFFFF +
             Handler->CoefReg.RmsIAC +
             Handler->CoefReg.RmsIBC +
             Handler->CoefReg.RmsUC +
             Handler->CoefReg.PowerPAC +
             Handler->CoefReg.PowerPBC +
             Handler->CoefReg.PowerSC +
             Handler->CoefReg.EnergyAC +
             Handler->CoefReg.EnergyBC;
  Checksum = ~Checksum;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_Coeff_chksum, &Reg16);
  if (Result < 0 || Checksum != Reg16)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Set the ratio of the resistors for current channel A
 * @note   This ration mentioned in the datasheet as K1
 * @param  Handler: Pointer to handler
 * @param  KIA: Ratio of the resistors for current channel A
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 */
HLW811x_Result_t
HLW811x_SetResRatioIA(HLW811x_Handler_t *Handler, float KIA)
{
  Handler->ResCoef.KIA = KIA;
  return HLW811X_OK;
}


/**
 * @brief  Set the ratio of the resistors for voltage channel
 * @note   This ration mentioned in the datasheet as K2
 * @param  Handler: Pointer to handler
 * @param  KU: Ratio of the resistors for voltage channel
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 */
HLW811x_Result_t
HLW811x_SetResRatioU(HLW811x_Handler_t *Handler, float KU)
{
  Handler->ResCoef.KU = KU;
  return HLW811X_OK;
}


/**
 * @brief  Set the oscillator frequency
 * @param  Handler: Pointer to handler
 * @param  Freq: Frequency in Hz
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 */
HLW811x_Result_t
HLW811x_SetCLKFreq(HLW811x_Handler_t *Handler, uint32_t Freq)
{
  Handler->CLKI = Freq;
  return HLW811X_OK;
}


/**
 * @brief  Enable/Disable the channels
 * @param  Handler: Pointer to handler
 * @param  U: Voltage channel
 * @param  IA: Current channel A
 * @param  IB: Current channel B
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_SetChannelOnOff(HLW811x_Handler_t *Handler,
                        HLW811x_EnDis_t U, HLW811x_EnDis_t IA, HLW811x_EnDis_t IB)
{
  int8_t Result = 0;
  uint16_t Reg = 0;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_SYSCON, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  if (U == HLW811X_ENDIS_ENABLE)
  {
    Reg |= (1 << HLW811X_REG_SYSCON_ADC3ON);
  }
  else if (U == HLW811X_ENDIS_DISABLE)
  {
    Reg &= ~(1 << HLW811X_REG_SYSCON_ADC3ON);
  }

  if (IA == HLW811X_ENDIS_ENABLE)
  {
    Reg |= (1 << HLW811X_REG_SYSCON_ADC1ON);
  }
  else if (IA == HLW811X_ENDIS_DISABLE)
  {
    Reg &= ~(1 << HLW811X_REG_SYSCON_ADC1ON);
  }

  if (IB == HLW811X_ENDIS_ENABLE)
  {
    Reg |= (1 << HLW811X_REG_SYSCON_ADC2ON);
  }
  else if (IB == HLW811X_ENDIS_DISABLE)
  {
    Reg &= ~(1 << HLW811X_REG_SYSCON_ADC2ON);
  }

  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg16(Handler, HLW811X_REG_ADDR_SYSCON, Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Set the PGA gain
 * @param  Handler: Pointer to handler
 * @param  U: PGA gain for voltage channel
 * @param  IA: PGA gain for current channel A
 * @param  IB: PGA gain for current channel B
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_SetPGA(HLW811x_Handler_t *Handler,
               HLW811x_PGA_t U, HLW811x_PGA_t IA, HLW811x_PGA_t IB)
{
  int8_t Result = 0;
  uint16_t Reg = 0;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_SYSCON, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  if (U < HLW811X_PGA_NONE)
  {
    Reg &= ~(0x07 << HLW811X_REG_SYSCON_PGAU);
    Reg |= (HLW811X_PGA_TABLE[U] << HLW811X_REG_SYSCON_PGAU);
  }

  if (IA < HLW811X_PGA_NONE)
  {
    Reg &= ~(0x07 << HLW811X_REG_SYSCON_PGAIA);
    Reg |= (HLW811X_PGA_TABLE[IA] << HLW811X_REG_SYSCON_PGAIA);
  }

  if (IB < HLW811X_PGA_NONE)
  {
    Reg &= ~(0x07 << HLW811X_REG_SYSCON_PGAIB);
    Reg |= (HLW811X_PGA_TABLE[IB] << HLW811X_REG_SYSCON_PGAIB);
  }

  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg16(Handler, HLW811X_REG_ADDR_SYSCON, Reg);
  if (Result < 0)
    return HLW811X_FAIL;
  
  Handler->PGA.U = U;
  Handler->PGA.IA = IA;
  Handler->PGA.IB = IB;

  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Set the active power calculation method
 * @param  Handler: Pointer to handler
 * @param  Method: Active power calculation method
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_SetActivePowCalcMethod(HLW811x_Handler_t *Handler,
                               HLW811x_ActivePowCalcMethod_t Method)
{
  int8_t Result = 0;
  uint16_t Reg = 0;

  if (Method > 3)
    return HLW811X_INVALID_PARAM;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_EMUCON, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  Reg &= ~(0x03 << HLW811X_REG_EMUCON_Pmode);
  Reg |= (Method << HLW811X_REG_EMUCON_Pmode);

  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg16(Handler, HLW811X_REG_ADDR_EMUCON, Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Set the RMS calculation mode
 * @param  Handler: Pointer to handler
 * @param  Mode: RMS calculation mode
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_SetRMSCalcMode(HLW811x_Handler_t *Handler, HLW811x_RMSCalcMode_t Mode)
{
  int8_t Result = 0;
  uint16_t Reg = 0;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_EMUCON, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  if (Mode == HLW811X_RMS_CALC_MODE_NORMAL)
    Reg &= ~(1 << HLW811X_REG_EMUCON_DC_MODE);
  else
    Reg |= (1 << HLW811X_REG_EMUCON_DC_MODE);
  

  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg16(Handler, HLW811X_REG_ADDR_EMUCON, Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Set the zero crossing detection mode
 * @param  Handler: Pointer to handler
 * @param  Mode: Zero crossing detection mode
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 *         - HLW811X_INVALID_PARAM: One of parameters is invalid.
 */
HLW811x_Result_t
HLW811x_SetZeroCrossing(HLW811x_Handler_t *Handler,
                        HLW811x_ZeroCrossingMode_t Mode)
{
  int8_t Result = 0;
  uint16_t Reg = 0;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_EMUCON, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  switch (Mode)
  {
  case HLW811X_ZERO_CROSSING_MODE_POSITIVE:
    Reg &= ~(1 << HLW811X_REG_EMUCON_ZXD0);
    Reg &= ~(1 << HLW811X_REG_EMUCON_ZXD1);
    break;

  case HLW811X_ZERO_CROSSING_MODE_NEGATIVE:
    Reg |= (1 << HLW811X_REG_EMUCON_ZXD0);
    Reg &= ~(1 << HLW811X_REG_EMUCON_ZXD1);
    break;

  case HLW811X_ZERO_CROSSING_MODE_BOTH:
    Reg &= ~(1 << HLW811X_REG_EMUCON_ZXD0);
    Reg |= (1 << HLW811X_REG_EMUCON_ZXD1);
    break;

  default:
    return HLW811X_INVALID_PARAM;
    break;
  }

  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg16(Handler, HLW811X_REG_ADDR_EMUCON, Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Set the high pass filter for digital signal
 * @param  Handler: Pointer to handler
 * @param  U: Voltage channel
 * @param  IA: Current channel A
 * @param  IB: Current channel B
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_SetDigitalHighPassFilter(HLW811x_Handler_t *Handler,
                                 HLW811x_EnDis_t U, HLW811x_EnDis_t IA, HLW811x_EnDis_t IB)
{
  int8_t Result = 0;
  uint16_t Reg = 0;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_EMUCON, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  if (U == HLW811X_ENDIS_ENABLE)
  {
    Reg &= ~(1 << HLW811X_REG_EMUCON_HPFUOFF);
  }
  else if (U == HLW811X_ENDIS_DISABLE)
  {
    Reg |= (1 << HLW811X_REG_EMUCON_HPFUOFF);
  }

  if (IA == HLW811X_ENDIS_ENABLE)
  {
    Reg &= ~(1 << HLW811X_REG_EMUCON_HPFIAOFF);
  }
  else if (IA == HLW811X_ENDIS_DISABLE)
  {
    Reg |= (1 << HLW811X_REG_EMUCON_HPFIAOFF);
  }

  if (IB == HLW811X_ENDIS_ENABLE)
  {
    Reg &= ~(1 << HLW811X_REG_EMUCON_HPFIBOFF);
  }
  else if (IB == HLW811X_ENDIS_DISABLE)
  {
    Reg |= (1 << HLW811X_REG_EMUCON_HPFIBOFF);
  }
  
  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg16(Handler, HLW811X_REG_ADDR_EMUCON, Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Set the PFB and PFA pulses
 * @param  Handler: Pointer to handler
 * @param  PFA: PFA pulse
 * @param  PFB: PFB pulse
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_SetPFPulse(HLW811x_Handler_t *Handler,
                   HLW811x_EnDis_t PFA, HLW811x_EnDis_t PFB)
{
  int8_t Result = 0;
  uint16_t Reg = 0;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_EMUCON, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  if (PFA == HLW811X_ENDIS_ENABLE)
  {
    Reg |= (1 << HLW811X_REG_EMUCON_PARUN);
  }
  else if (PFA == HLW811X_ENDIS_DISABLE)
  {
    Reg &= ~(1 << HLW811X_REG_EMUCON_PARUN);
  }

  if (PFB == HLW811X_ENDIS_ENABLE)
  {
    Reg |= (1 << HLW811X_REG_EMUCON_PBRUN);
  }
  else if (PFB == HLW811X_ENDIS_DISABLE)
  {
    Reg &= ~(1 << HLW811X_REG_EMUCON_PBRUN);
  }
  
  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg16(Handler, HLW811X_REG_ADDR_EMUCON, Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Set open drain for SDO pin
 * @param  Handler: Pointer to handler
 * @param  Enable: Enable/Disable open drain for SDO pin
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_SetSDOPinOpenDrain(HLW811x_Handler_t *Handler, HLW811x_EnDis_t Enable)
{
  int8_t Result = 0;
  uint16_t Reg = 0;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_EMUCON2, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  switch (Enable)
  {
  case HLW811X_ENDIS_ENABLE:
    Reg |= (1 << HLW811X_REG_EMUCON2_SDOCmos);
    break;

  case HLW811X_ENDIS_DISABLE:
    Reg &= ~(1 << HLW811X_REG_EMUCON2_SDOCmos);
    break;

  default:
    return HLW811X_INVALID_PARAM;
    break;
  }
  
  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg16(Handler, HLW811X_REG_ADDR_EMUCON2, Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Set the energy clearance
 * @param  Handler: Pointer to handler
 * @param  PA: Energy clearance for channel A
 * @param  PB: Energy clearance for channel B
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_SetEnergyClearance(HLW811x_Handler_t *Handler,
                           HLW811x_EnDis_t PA, HLW811x_EnDis_t PB)
{
  int8_t Result = 0;
  uint16_t Reg = 0;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_EMUCON2, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  if (PA == HLW811X_ENDIS_ENABLE)
  {
    Reg &= ~(1 << HLW811X_REG_EMUCON2_EPB_CA);
  }
  else if (PA == HLW811X_ENDIS_DISABLE)
  {
    Reg |= (1 << HLW811X_REG_EMUCON2_EPB_CA);
  }

  if (PB == HLW811X_ENDIS_ENABLE)
  {
    Reg &= ~(1 << HLW811X_REG_EMUCON2_EPB_CB);
  }
  else if (PB == HLW811X_ENDIS_DISABLE)
  {
    Reg |= (1 << HLW811X_REG_EMUCON2_EPB_CB);
  }
  
  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg16(Handler, HLW811X_REG_ADDR_EMUCON2, Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Set the data update frequency
 * @param  Handler: Pointer to handler
 * @param  Freq: Data update frequency
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 *         - HLW811X_INVALID_PARAM: One of parameters is invalid.
 */
HLW811x_Result_t
HLW811x_SetDataUpdateFreq(HLW811x_Handler_t *Handler,
                          HLW811x_DataUpdateFreq_t Freq)
{
  int8_t Result = 0;
  uint16_t Reg = 0;
  uint16_t Mask = 0;

  switch (Freq)
  {
  case HLW811X_DATA_UPDATE_FREQ_3_4HZ:
    Mask = 0;
    break;

  case HLW811X_DATA_UPDATE_FREQ_6_8HZ:
    Mask = (1 << HLW811X_REG_EMUCON2_DUPSEL);
    break;

  case HLW811X_DATA_UPDATE_FREQ_13_65HZ:
    Mask = (2 << HLW811X_REG_EMUCON2_DUPSEL);
    break;

  case HLW811X_DATA_UPDATE_FREQ_27_3HZ:
    Mask = (3 << HLW811X_REG_EMUCON2_DUPSEL);
    break;

  default:
    return HLW811X_INVALID_PARAM;
    break;
  }

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_EMUCON2, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  Reg &= ~(3 << HLW811X_REG_EMUCON2_DUPSEL);
  Reg |= Mask;
  
  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg16(Handler, HLW811X_REG_ADDR_EMUCON2, Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Set the power factor functionality
 * @param  Handler: Pointer to handler
 * @param  Enable: Enable/Disable power factor functionality
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 *         - HLW811X_INVALID_PARAM: One of parameters is invalid.
 */
HLW811x_Result_t
HLW811x_SetPowerFactorFunctionality(HLW811x_Handler_t *Handler,
                                    HLW811x_EnDis_t Enable)
{
  int8_t Result = 0;
  uint16_t Reg = 0;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_EMUCON2, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  switch (Enable)
  {
  case HLW811X_ENDIS_ENABLE:
    Reg |= (1 << HLW811X_REG_EMUCON2_PfactorEN);
    break;

  case HLW811X_ENDIS_DISABLE:
    Reg &= ~(1 << HLW811X_REG_EMUCON2_PfactorEN);
    break;

  default:
    return HLW811X_INVALID_PARAM;
    break;
  }
  
  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg16(Handler, HLW811X_REG_ADDR_EMUCON2, Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Set the waveform data
 * @param  Handler: Pointer to handler
 * @param  Enable: Enable/Disable waveform data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 *         - HLW811X_INVALID_PARAM: One of parameters is invalid.
 */
HLW811x_Result_t
HLW811x_SetWaveformData(HLW811x_Handler_t *Handler,
                        HLW811x_EnDis_t Enable)
{
  int8_t Result = 0;
  uint16_t Reg = 0;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_EMUCON2, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  switch (Enable)
  {
  case HLW811X_ENDIS_ENABLE:
    Reg |= (1 << HLW811X_REG_EMUCON2_WaveEN);
    break;

  case HLW811X_ENDIS_DISABLE:
    Reg &= ~(1 << HLW811X_REG_EMUCON2_WaveEN);
    break;

  default:
    return HLW811X_INVALID_PARAM;
    break;
  }
  
  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg16(Handler, HLW811X_REG_ADDR_EMUCON2, Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Set the voltage sag detection
 * @param  Handler: Pointer to handler
 * @param  Enable: Enable/Disable voltage sag detection
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 *         - HLW811X_INVALID_PARAM: One of parameters is invalid.
 */
HLW811x_Result_t
HLW811x_SetVoltageSagDetection(HLW811x_Handler_t *Handler,
                               HLW811x_EnDis_t Enable)
{
  int8_t Result = 0;
  uint16_t Reg = 0;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_EMUCON2, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  switch (Enable)
  {
  case HLW811X_ENDIS_ENABLE:
    Reg |= (1 << HLW811X_REG_EMUCON2_SAGEN);
    break;

  case HLW811X_ENDIS_DISABLE:
    Reg &= ~(1 << HLW811X_REG_EMUCON2_SAGEN);
    break;

  default:
    return HLW811X_INVALID_PARAM;
    break;
  }
  
  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg16(Handler, HLW811X_REG_ADDR_EMUCON2, Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Set the over voltage, current and load detection
 * @param  Handler: Pointer to handler
 * @param  Enable: Enable/Disable over voltage and current detection
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 *         - HLW811X_INVALID_PARAM: One of parameters is invalid.
 */
HLW811x_Result_t
HLW811x_SetOverVolCarDetection(HLW811x_Handler_t *Handler,
                               HLW811x_EnDis_t Enable)
{
  int8_t Result = 0;
  uint16_t Reg = 0;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_EMUCON2, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  switch (Enable)
  {
  case HLW811X_ENDIS_ENABLE:
    Reg |= (1 << HLW811X_REG_EMUCON2_OverEN);
    break;

  case HLW811X_ENDIS_DISABLE:
    Reg &= ~(1 << HLW811X_REG_EMUCON2_OverEN);
    break;

  default:
    return HLW811X_INVALID_PARAM;
    break;
  }
  
  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg16(Handler, HLW811X_REG_ADDR_EMUCON2, Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Set zero crossing detection
 * @param  Handler: Pointer to handler
 * @param  Enable: Enable/Disable zero crossing detection
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 *         - HLW811X_INVALID_PARAM: One of parameters is invalid.
 */
HLW811x_Result_t
HLW811x_SetZeroCrossingDetection(HLW811x_Handler_t *Handler,
                                 HLW811x_EnDis_t Enable)
{
  int8_t Result = 0;
  uint16_t Reg = 0;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_EMUCON2, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  switch (Enable)
  {
  case HLW811X_ENDIS_ENABLE:
    Reg |= (1 << HLW811X_REG_EMUCON2_ZxEN);
    break;

  case HLW811X_ENDIS_DISABLE:
    Reg &= ~(1 << HLW811X_REG_EMUCON2_ZxEN);
    break;

  default:
    return HLW811X_INVALID_PARAM;
    break;
  }
  
  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg16(Handler, HLW811X_REG_ADDR_EMUCON2, Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Set the peak detection
 * @param  Handler: Pointer to handler
 * @param  Enable: Enable/Disable peak detection
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 *         - HLW811X_INVALID_PARAM: One of parameters is invalid.
 */
HLW811x_Result_t
HLW811x_SetPeakDetection(HLW811x_Handler_t *Handler,
                         HLW811x_EnDis_t Enable)
{
  int8_t Result = 0;
  uint16_t Reg = 0;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_EMUCON2, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  switch (Enable)
  {
  case HLW811X_ENDIS_ENABLE:
    Reg |= (1 << HLW811X_REG_EMUCON2_PeakEN);
    break;

  case HLW811X_ENDIS_DISABLE:
    Reg &= ~(1 << HLW811X_REG_EMUCON2_PeakEN);
    break;

  default:
    return HLW811X_INVALID_PARAM;
    break;
  }
  
  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg16(Handler, HLW811X_REG_ADDR_EMUCON2, Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Set INT1 and INT2 pins functionality
 * @param  Handler: Pointer to handler
 * @param  INT1: INT1 pin functionality
 * @param  INT2: INT2 pin functionality
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 *         - HLW811X_INVALID_PARAM: One of parameters is invalid.
 */
HLW811x_Result_t
HLW811x_SetIntOutFunc(HLW811x_Handler_t *Handler,
                      HLW811x_IntOutFunc_t INT1, HLW811x_IntOutFunc_t INT2)
{
  int8_t Result = 0;
  uint16_t Reg = 0;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_INT, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  if (INT1 < HLW811X_INTOUT_FUNC_NO_CHANGE)
  {
    Reg &= ~(0x0F << HLW811X_REG_INT_P1sel);
    Reg |= ((INT1 & 0x0F) << HLW811X_REG_INT_P1sel);
  }

  if (INT2 < HLW811X_INTOUT_FUNC_NO_CHANGE)
  {
    Reg &= ~(0x0F << HLW811X_REG_INT_P2sel);
    Reg |= ((INT2 & 0x0F) << HLW811X_REG_INT_P2sel);
  }

  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg16(Handler, HLW811X_REG_ADDR_INT, Reg);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Set interrupt mask
 * @param  Handler: Pointer to handler
 * @param  Mask: Interrupt mask value
 * @note   Use the define HLW811X_REG_IE_XXX to set the mask value
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_SetIntMask(HLW811x_Handler_t *Handler, uint16_t Mask)
{
  int8_t Result = 0;

  Result = HLW811x_CommandEnableWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_WriteReg16(Handler, HLW811X_REG_ADDR_IE, Mask);
  if (Result < 0)
    return HLW811X_FAIL;

  Result = HLW811x_CommandCloseWriteOperation(Handler);
  if (Result < 0)
    return HLW811X_FAIL;

  return HLW811X_OK;
}


/**
 * @brief  Get interrupt status flag
 * @note   Use the define HLW811X_REG_IF_XXX to check the flag
 * @param  Handler: Pointer to handler
 * @param  Mask: Pointer to interrupt mask value
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetIntFlag(HLW811x_Handler_t *Handler, uint16_t *Mask)
{
  int8_t Result = 0;
  uint16_t Reg = 0;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_IF, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;
  
  if (Mask)
    *Mask = Reg;

  return HLW811X_OK;
}


/**
 * @brief  Get reset interrupt status flag
 * @note   Use the define HLW811X_REG_RIF_XXX to check the flag
 * @param  Handler: Pointer to handler
 * @param  Mask: Pointer to interrupt mask value
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetResIntFlag(HLW811x_Handler_t *Handler, uint16_t *Mask)
{
  int8_t Result = 0;
  uint16_t Reg = 0;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_RIF, &Reg);
  if (Result < 0)
    return HLW811X_FAIL;
  
  if (Mask)
    *Mask = Reg;

  return HLW811X_OK;
}


/**
 * @brief  Get the RMS value of the voltage in V
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to store the data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetRmsU(HLW811x_Handler_t *Handler, float *Data)
{
  int8_t Result = 0;
  uint32_t RawValue = 0;
  uint16_t CoefReg = 0;
  float ResCoef = 0;
  uint8_t PGA = 0;

  Result = HLW811x_ReadReg24(Handler, HLW811X_REG_ADDR_RmsU, &RawValue);
  if (Result < 0)
    return HLW811X_FAIL;

  CoefReg = Handler->CoefReg.RmsUC;
  ResCoef = Handler->ResCoef.KU;
  PGA = (1 << Handler->PGA.U);
  *Data = (float)RawValue * (CoefReg / 4194304.0f / ResCoef / 100 / PGA);

  return HLW811X_OK;
}


/**
 * @brief  Get the RMS value of the current channel A in A
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to store the data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetRmsIA(HLW811x_Handler_t *Handler, float *Data)
{
  int8_t Result = 0;
  uint32_t RawValue = 0;
  uint16_t CoefReg = 0;
  float ResCoef = 0;
  uint8_t PGA = 0;
  double DoubleBuffer = 0;

  Result = HLW811x_ReadReg24(Handler, HLW811X_REG_ADDR_RmsIA, &RawValue);
  if (Result < 0)
    return HLW811X_FAIL;

  CoefReg = Handler->CoefReg.RmsIAC;
  ResCoef = Handler->ResCoef.KIA;
  PGA = 16 >> Handler->PGA.IA;
  DoubleBuffer = (double)RawValue * (CoefReg / 8388608.0 / ResCoef / 1000 * PGA);
  *Data = (float)DoubleBuffer;

  return HLW811X_OK;
}


/**
 * @brief  Get the RMS value of the current channel B in A
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to store the data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetRmsIB(HLW811x_Handler_t *Handler, float *Data)
{
  int8_t Result = 0;
  uint32_t RawValue = 0;
  uint16_t CoefReg = 0;
  uint8_t PGA = 0;
  double DoubleBuffer = 0;

  Result = HLW811x_ReadReg24(Handler, HLW811X_REG_ADDR_RmsIB, &RawValue);
  if (Result < 0)
    return HLW811X_FAIL;

  CoefReg = Handler->CoefReg.RmsIBC;
  PGA = 16 >> Handler->PGA.IB;
  DoubleBuffer = (double)RawValue * (CoefReg / 8388608.0 / 1000 * PGA);
  *Data = (float)DoubleBuffer;

  return HLW811X_OK;
}


/**
 * @brief  Get active power of channel A in W
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to store the data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetPowerPA(HLW811x_Handler_t *Handler, float *Data)
{
  int8_t Result = 0;
  uint32_t RawValue = 0; // TODO: Check data type (I32 or U32)
  uint16_t CoefReg = 0;
  double ResCoef = 0;
  double PGA = 0;
  double DoubleBuffer = 0;

  Result = HLW811x_ReadReg32(Handler, HLW811X_REG_ADDR_PowerPA, &RawValue);
  if (Result < 0)
    return HLW811X_FAIL;

  CoefReg = Handler->CoefReg.PowerPAC;
  PGA = 16 >> (Handler->PGA.U + Handler->PGA.IA);
  ResCoef = Handler->ResCoef.KU * Handler->ResCoef.KIA;
  DoubleBuffer = (double)RawValue * (CoefReg / 2147483648.0 / ResCoef * PGA);
  *Data = (float)DoubleBuffer;

  return HLW811X_OK;
}


/**
 * @brief  Get active power of channel B in W
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to store the data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetPowerPB(HLW811x_Handler_t *Handler, float *Data)
{
  int8_t Result = 0;
  uint32_t RawValue = 0;
  uint16_t CoefReg = 0;
  double ResCoef = 0;
  uint16_t PGA = 0;
  double DoubleBuffer = 0;

  Result = HLW811x_ReadReg32(Handler, HLW811X_REG_ADDR_PowerPB, &RawValue);
  if (Result < 0)
    return HLW811X_FAIL;

  CoefReg = Handler->CoefReg.PowerPBC;
  PGA = (1 << Handler->PGA.U) * (1 << Handler->PGA.IB);
  ResCoef = Handler->ResCoef.KU * Handler->ResCoef.KIA;
  DoubleBuffer = (double)RawValue * (CoefReg / 2147483648.0 / ResCoef * PGA);
  *Data = (float)DoubleBuffer;

  return HLW811X_OK;
}


/**
 * @brief  Get the apparent power of selected channel in W
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to store the data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetPowerS(HLW811x_Handler_t *Handler, float *Data)
{
  int8_t Result = 0;
  uint32_t RawValue = 0;
  uint16_t CoefReg = 0;
  double ResCoef = 0;
  double PGA = 0;
  double DoubleBuffer = 0;

  Result = HLW811x_ReadReg32(Handler, HLW811X_REG_ADDR_PowerS, &RawValue);
  if (Result < 0)
    return HLW811X_FAIL;

  // TODO: Check the selected channel
  CoefReg = Handler->CoefReg.PowerSC;
  PGA = 16 >> (Handler->PGA.U + Handler->PGA.IA);
  ResCoef = Handler->ResCoef.KU * Handler->ResCoef.KIA;
  DoubleBuffer = (double)RawValue * (CoefReg / 2147483648.0 / ResCoef * PGA);
  *Data = (float)DoubleBuffer;

  return HLW811X_OK;
}


/**
 * @brief  Get Energy of channel A in KWH
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to store the data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetEnergyA(HLW811x_Handler_t *Handler, float *Data)
{
  int8_t Result = 0;
  uint32_t RawValue = 0;
  uint16_t CoefReg = 0;
  float ResCoef = 0;
  uint16_t PGA = 0;
  double DoubleBuffer = 0;

  Result = HLW811x_ReadReg24(Handler, HLW811X_REG_ADDR_Energy_PA, &RawValue);
  if (Result < 0)
    return HLW811X_FAIL;

  CoefReg = Handler->CoefReg.EnergyAC;
  PGA = (1 << Handler->PGA.U) * (1 << Handler->PGA.IA);
  ResCoef = Handler->ResCoef.KU * Handler->ResCoef.KIA;
  DoubleBuffer = (double)RawValue * (CoefReg / 536870912.0 / PGA / 4096 / ResCoef) * Handler->HFconst;
  *Data = (float)DoubleBuffer;

  return HLW811X_OK;
}


/**
 * @brief  Get Energy of channel B in KWH
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to store the data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetEnergyB(HLW811x_Handler_t *Handler, float *Data)
{
  int8_t Result = 0;
  uint32_t RawValue = 0;
  uint16_t CoefReg = 0;
  float ResCoef = 0;
  uint16_t PGA = 0;

  Result = HLW811x_ReadReg24(Handler, HLW811X_REG_ADDR_Energy_PB, &RawValue);
  if (Result < 0)
    return HLW811X_FAIL;

  // TODO: Fix resistor ratio
  CoefReg = Handler->CoefReg.EnergyBC;
  PGA = (1 << Handler->PGA.U) * (1 << Handler->PGA.IB);
  ResCoef = Handler->ResCoef.KU * Handler->ResCoef.KIA;
  *Data = (float)RawValue * (CoefReg / (double)536870912.0f / PGA / 4096 * Handler->HFconst);

  return HLW811X_OK;
}


/**
 * @brief  Get the frequency of the voltage channel in Hz
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to store the data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetFreqU(HLW811x_Handler_t *Handler, float *Data)
{
  int8_t Result = 0;
  uint16_t RawValue = 0;

  Result = HLW811x_ReadReg16(Handler, HLW811X_REG_ADDR_Ufreq, &RawValue);
  if (Result < 0)
    return HLW811X_FAIL;

  *Data = Handler->CLKI / 8.0 / RawValue;

  return HLW811X_OK;
}
