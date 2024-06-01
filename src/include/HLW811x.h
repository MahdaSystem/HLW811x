/**
 **********************************************************************************
 * @file   HLW811x.h
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

/* Define to prevent recursive inclusion ----------------------------------------*/
#ifndef	_HLW811X_H_
#define _HLW811X_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ---------------------------------------------------------------------*/
#include <stdint.h>
#include "HLW811x_config.h"

/* Configurations ---------------------------------------------------------------*/
#ifndef HLW811X_CONFIG_SUPPORT_SPI
  #define HLW811X_CONFIG_SUPPORT_SPI  1
#endif

#ifndef HLW811X_CONFIG_SUPPORT_UART
  #define HLW811X_CONFIG_SUPPORT_UART 1
#endif

#if (HLW811X_CONFIG_SUPPORT_SPI == 0 && HLW811X_CONFIG_SUPPORT_UART == 0)
  #error "HLW811x: SPI and UART can not be both disabled!"
#endif


/* Exported Data Types ----------------------------------------------------------*/
/**
 * @brief  Library functions result data type
 */
typedef enum HLW811x_Result_e
{
  HLW811X_OK              = 0,
  HLW811X_FAIL            = 1,
  HLW811X_INVALID_PARAM   = 2,
} HLW811x_Result_t;


/**
 * @brief  Device type
 */
typedef enum HLW811x_Device_e
{
  HLW811X_DEVICE_HLW8110 = 0,
  HLW811X_DEVICE_HLW8112 = 1,
} HLW811x_Device_t;


/**
 * @brief  Channel type
 */
typedef enum HLW811x_CurrentChannel_e
{
  HLW811X_CURRENT_CHANNEL_A = 0,
  HLW811X_CURRENT_CHANNEL_B = 1
} HLW811x_CurrentChannel_t;

/**
 * @brief  PGA gain
 */
typedef enum HLW811x_PGA_e
{
  HLW811X_PGA_1   = 0,
  HLW811X_PGA_2   = 1,
  HLW811X_PGA_4   = 2,
  HLW811X_PGA_8   = 3,
  HLW811X_PGA_16  = 4,
  HLW811X_PGA_NONE = 5
} HLW811x_PGA_t;


/**
 * @brief  Active power calculation method
 */
typedef enum HLW811x_ActivePowCalcMethod_e
{
  HLW811X_ACTIVE_POW_CALC_METHOD_POS_NEG_ALGEBRAIC = 0,
  HLW811X_ACTIVE_POW_CALC_METHOD_POS = 1,
  HLW811X_ACTIVE_POW_CALC_METHOD_POS_NEG_ABSOLUTE = 2
} HLW811x_ActivePowCalcMethod_t;


/**
 * @brief  RMS calculation mode
 * @note   In DC mode, it is mandatory to enable the Waveform
 *         Data.
 */
typedef enum HLW811x_RMSCalcMode_e
{
  HLW811X_RMS_CALC_MODE_NORMAL = 0,
  HLW811X_RMS_CALC_MODE_DC = 1
} HLW811x_RMSCalcMode_t;


/**
 * @brief  Zero crossing detection mode
 */
typedef enum HLW811x_ZeroCrossingMode_e
{
  HLW811X_ZERO_CROSSING_MODE_POSITIVE = 0,
  HLW811X_ZERO_CROSSING_MODE_NEGATIVE = 1,
  HLW811X_ZERO_CROSSING_MODE_BOTH = 2
} HLW811x_ZeroCrossingMode_t;


/**
 * @brief  Interrupt output functionality
 */
typedef enum HLW811x_IntOutFunc_e
{
  HLW811X_INTOUT_FUNC_PULSE_PFA = 0,
  HLW811X_INTOUT_FUNC_PULSE_PFB = 1,
  HLW811X_INTOUT_FUNC_LEAKAGE= 2,
  HLW811X_INTOUT_FUNC_IRQ = 3,
  HLW811X_INTOUT_FUNC_POWER_OVERLOAD = 4,
  HLW811X_INTOUT_FUNC_NEGATIVE_POWER_A = 5,
  HLW811X_INTOUT_FUNC_NEGATIVE_POWER_B = 6,
  HLW811X_INTOUT_FUNC_INSTAN_VALUE_UPDATE_INT = 7,
  HLW811X_INTOUT_FUNC_AVG_UPDATE_INT = 8,
  HLW811X_INTOUT_FUNC_VOLTAGE_ZERO_CROSSING = 9,
  HLW811X_INTOUT_FUNC_CURRENT_ZERO_CROSSING_A = 10,
  HLW811X_INTOUT_FUNC_CURRENT_ZERO_CROSSING_B = 11,
  HLW811X_INTOUT_FUNC_OVERVOLTAGE = 12,
  HLW811X_INTOUT_FUNC_UNDERVOLTAGE = 13,
  HLW811X_INTOUT_FUNC_OVERCURRENT_A = 14,
  HLW811X_INTOUT_FUNC_OVERCURRENT_B = 15,
  HLW811X_INTOUT_FUNC_NO_CHANGE = 16
} HLW811x_IntOutFunc_t;


/**
 * @brief  Data update frequency
 */
typedef enum HLW811x_DataUpdateFreq_e
{
  HLW811X_DATA_UPDATE_FREQ_3_4HZ = 0,
  HLW811X_DATA_UPDATE_FREQ_6_8HZ = 1,
  HLW811X_DATA_UPDATE_FREQ_13_65HZ = 2,
  HLW811X_DATA_UPDATE_FREQ_27_3HZ = 3
} HLW811x_DataUpdateFreq_t;


/**
 * @brief  Enable/Disable type
 */
typedef enum HLW811x_EnDis_e
{
  HLW811X_ENDIS_NOCHANGE = -1,
  HLW811X_ENDIS_DISABLE = 0,
  HLW811X_ENDIS_ENABLE = 1
} HLW811x_EnDis_t;


/**
 * @brief  Communication type
 */
typedef enum HLW811x_Communication_e
{
  HLW811X_COMMUNICATION_SPI  = 0,
  HLW811X_COMMUNICATION_UART = 1,
} HLW811x_Communication_t;


/**
 * @brief  Function type for Initialize/Deinitialize the platform dependent layer.
 * @retval 
 *         -  0: The operation was successful.
 *         - -1: The operation failed. 
 */
typedef int8_t (*HLW811x_Platform_InitDeinit_t)(void);

/**
 * @brief  Function type for Set/Reset GPIO pin.
 * @param  Level: 0 for reset, 1 for set
 * @retval 
 *         -  0: The operation was successful.
 *         - -1: Failed to set/reset GPIO pin.
 */
typedef int8_t (*HLW811x_Platform_SetLevelGPIO_t)(uint8_t Level);

/**
 * @brief  Function type for delay.
 * @param  Delay: Delay
 * @retval 
 *         -  0: The operation was successful.
 *         - -1: Failed
 */
typedef int8_t (*HLW811x_Platform_Delay_t)(uint8_t Delay);

#if (HLW811X_CONFIG_SUPPORT_SPI)
/**
 * @brief  Function type for Send/Receive data to/from the slave through SPI.
 * @param  SendData: Pointer to data to send
 * @param  ReceiveData: Pointer to data to receive
 * @param  Len: data len in Bytes
 * @note   If SendData is NULL, the function must receive data.
 * @note   If ReceiveData is NULL, the function must send data.
 * @retval 
 *         -  0: The operation was successful.
 *         - -1: Failed to send/receive.
 */
typedef int8_t (*HLW811x_Platform_SPI_SendReceive_t)(uint8_t *SendData,
                                                     uint8_t *ReceiveData,
                                                     uint8_t Len);
#endif

#if (HLW811X_CONFIG_SUPPORT_UART)
/**
 * @brief  Function type for Send/Receive data to/from the slave through UART.
 * @param  Data: Pointer to data to send/receive
 * @param  Len: data len in Bytes
 * @retval 
 *         -  0: The operation was successful.
 *         - -1: Failed to send/receive.
 */
typedef int8_t (*HLW811x_Platform_UART_SendReceive_t)(uint8_t *Data,
                                                      uint8_t Len);
#endif

/**
 * @brief  Platform dependent layer data type
 * @note   It is optional to initialize this functions:
 *         - Init
 *         - DeInit
 * @note   If using SPI, user must initialize this this functions before using library:
 *         - SendReceive
 *         - SetLevelSCSN
 * @note   If using UART, user must initialize this this functions before using library:
 *         - Send
 *         - Receive
 * @note   If success the functions must return 0 
 */
typedef struct HLW811x_Platform_s
{
#if (HLW811X_CONFIG_SUPPORT_SPI && HLW811X_CONFIG_SUPPORT_UART)
  // Communication type
  HLW811x_Communication_t Communication;
#endif

  // Initialize platform dependent layer
  HLW811x_Platform_InitDeinit_t Init;
  // De-initialize platform dependent layer
  HLW811x_Platform_InitDeinit_t DeInit;

  // Millisecond delay
  HLW811x_Platform_Delay_t DelayMs;

  // Platform dependent layer for SPI or UART
  union
  {
#if (HLW811X_CONFIG_SUPPORT_SPI)
    struct
    {
      // Send and Receive data
      HLW811x_Platform_SPI_SendReceive_t SendReceive;
      // Set/Reset SCSN pin
      HLW811x_Platform_SetLevelGPIO_t SetLevelSCSN;
    } SPI;
#endif
#if (HLW811X_CONFIG_SUPPORT_UART)
    struct
    {
      // Send data
      HLW811x_Platform_UART_SendReceive_t Send;
      // Receive data
      HLW811x_Platform_UART_SendReceive_t Receive;
    } UART;
#endif
  };
} HLW811x_Platform_t;


/**
 * @brief  Handler data type
 * @note   User must initialize this this functions before using library:
 *         - PlatformInit
 *         - PlatformDeInit
 *         - PlatformSend
 *         - PlatformReceive
 * @note   If success the functions must return 0 
 */
typedef struct HLW811x_Handler_s
{
  HLW811x_Device_t Device;

  // Coefficients
  struct
  {
    uint16_t RmsIAC;
    uint16_t RmsIBC;
    uint16_t RmsUC;
    uint16_t PowerPAC;
    uint16_t PowerPBC;
    uint16_t PowerSC;
    uint16_t EnergyAC;
    uint16_t EnergyBC;
  } CoefReg;

  struct
  {
    float KU;
    float KIA;
  } ResCoef;

  struct
  {
    HLW811x_PGA_t U;
    HLW811x_PGA_t IA;
    HLW811x_PGA_t IB;
  } PGA;

  
  uint16_t HFconst;
  uint32_t CLKI;
  HLW811x_CurrentChannel_t CurrentChannel;

  // Platform dependent layer
  HLW811x_Platform_t Platform;
} HLW811x_Handler_t;


/* Exported Macros --------------------------------------------------------------*/
#if (HLW811X_CONFIG_SUPPORT_SPI && HLW811X_CONFIG_SUPPORT_UART)
/**
 * @brief  Link platform dependent layer communication type
 * @param  HANDLER: Pointer to handler
 * @param  COM: Communication type
 *         - HLW811X_COMMUNICATION_SPI
 *         - HLW811X_COMMUNICATION_UART
 */
#define HLW811X_PLATFORM_SET_COMMUNICATION(HANDLER, COM) \
  (HANDLER)->Platform.Communication = COM
#else
#define HLW811X_PLATFORM_SET_COMMUNICATION(HANDLER, COM) \
  do {} while(0)
#endif

/**
 * @brief  Link platform dependent layer functions to handler
 * @param  HANDLER: Pointer to handler
 * @param  FUNC: Function name
 */
#define HLW811X_PLATFORM_LINK_INIT(HANDLER, FUNC) \
  (HANDLER)->Platform.Init = FUNC

/**
 * @brief  Link platform dependent layer functions to handler
 * @param  HANDLER: Pointer to handler
 * @param  FUNC: Function name
 */
#define HLW811X_PLATFORM_LINK_DEINIT(HANDLER, FUNC) \
  (HANDLER)->Platform.DeInit = FUNC

/**
 * @brief  Link platform dependent layer functions to handler
 * @param  HANDLER: Pointer to handler
 * @param  FUNC: Function name
 */
#define HLW811X_PLATFORM_LINK_DELAYMS(HANDLER, FUNC) \
  (HANDLER)->Platform.DelayMs = FUNC

#if (HLW811X_CONFIG_SUPPORT_SPI)
/**
 * @brief  Link platform dependent layer functions to handler
 * @param  HANDLER: Pointer to handler
 * @param  FUNC: Function name
 */
#define HLW811X_PLATFORM_LINK_SPI_SENDRECEIVE(HANDLER, FUNC) \
  (HANDLER)->Platform.SPI.SendReceive = FUNC

/**
 * @brief  Link platform dependent layer functions to handler
 * @param  HANDLER: Pointer to handler
 * @param  FUNC: Function name
 */
#define HLW811X_PLATFORM_LINK_SPI_SETLEVELSCSN(HANDLER, FUNC) \
  (HANDLER)->Platform.SPI.SetLevelSCSN = FUNC
#endif

#if (HLW811X_CONFIG_SUPPORT_UART)
/**
 * @brief  Link platform dependent layer functions to handler
 * @param  HANDLER: Pointer to handler
 * @param  FUNC: Function name
 */
#define HLW811X_PLATFORM_LINK_UART_SEND(HANDLER, FUNC) \
  (HANDLER)->Platform.UART.Send = FUNC

/**
 * @brief  Link platform dependent layer functions to handler
 * @param  HANDLER: Pointer to handler
 * @param  FUNC: Function name
 */
#define HLW811X_PLATFORM_LINK_UART_RECEIVE(HANDLER, FUNC) \
  (HANDLER)->Platform.UART.Receive = FUNC
#endif



/**
 ==================================================================================
                        ##### Initialization Functions #####                       
 ==================================================================================
 */

/**
 * @brief  Initializer function
 * @note   This function must be called after initializing platform dependent
 *         layer and before using other functions.
 * @note   This function will reset the device.
 * @param  Handler: Pointer to handler
 * @param  Device: Device type
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 *         - HLW811X_INVALID_PARAM: One of parameters is invalid.
 */
HLW811x_Result_t
HLW811x_Init(HLW811x_Handler_t *Handler, HLW811x_Device_t Device);


/**
 * @brief  Deinitialize function
 * @param  Handler: Pointer to handler
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_DeInit(HLW811x_Handler_t *Handler);



/**
 ==================================================================================
                          ##### Low Level Functions #####                          
 ==================================================================================
 */

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
                  uint8_t Len);


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
                   uint8_t Len);



/**
 ==================================================================================
                        ##### Configuration Functions #####                        
 ==================================================================================
 */

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
HLW811x_Begin(HLW811x_Handler_t *Handler);


/**
 * @brief  Set the ratio of the resistors for current channel A
 * @note   This ration mentioned in the datasheet as K1
 * @param  Handler: Pointer to handler
 * @param  KIA: Ratio of the resistors for current channel A
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 */
HLW811x_Result_t
HLW811x_SetResRatioIA(HLW811x_Handler_t *Handler, float KIA);


/**
 * @brief  Set the ratio of the resistors for voltage channel
 * @note   This ration mentioned in the datasheet as K2
 * @param  Handler: Pointer to handler
 * @param  KU: Ratio of the resistors for voltage channel
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 */
HLW811x_Result_t
HLW811x_SetResRatioU(HLW811x_Handler_t *Handler, float KU);


/**
 * @brief  Set the oscillator frequency
 * @param  Handler: Pointer to handler
 * @param  Freq: Frequency in Hz
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 */
HLW811x_Result_t
HLW811x_SetCLKFreq(HLW811x_Handler_t *Handler, uint32_t Freq);


/**
 * @brief  Set current channel for special measurements
 * @note   The selected channel will be used for special measurements such as
 *         apparent power, power factor, phase angle, instantaneous
 *         apparent power and active power overload
 * 
 * @param  Handler: Pointer to handler
 * @param  Channel: Current channel
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 *         - HLW811X_INVALID_PARAM: One of parameters is invalid.
 */
HLW811x_Result_t
HLW811x_SetSpecialMeasurementChannel(HLW811x_Handler_t *Handler,
                                     HLW811x_CurrentChannel_t Channel);


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
                        HLW811x_EnDis_t U, HLW811x_EnDis_t IA, HLW811x_EnDis_t IB);


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
               HLW811x_PGA_t U, HLW811x_PGA_t IA, HLW811x_PGA_t IB);


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
                               HLW811x_ActivePowCalcMethod_t Method);


/**
 * @brief  Set the RMS calculation mode
 * @param  Handler: Pointer to handler
 * @param  Mode: RMS calculation mode
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_SetRMSCalcMode(HLW811x_Handler_t *Handler, HLW811x_RMSCalcMode_t Mode);


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
                        HLW811x_ZeroCrossingMode_t Mode);


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
                                 HLW811x_EnDis_t U, HLW811x_EnDis_t IA, HLW811x_EnDis_t IB);


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
                   HLW811x_EnDis_t PFA, HLW811x_EnDis_t PFB);


/**
 * @brief  Set open drain for SDO pin
 * @param  Handler: Pointer to handler
 * @param  Enable: Enable/Disable open drain for SDO pin
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_SetSDOPinOpenDrain(HLW811x_Handler_t *Handler, HLW811x_EnDis_t Enable);


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
                           HLW811x_EnDis_t PA, HLW811x_EnDis_t PB);


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
                          HLW811x_DataUpdateFreq_t Freq);


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
                                    HLW811x_EnDis_t Enable);


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
                        HLW811x_EnDis_t Enable);


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
                               HLW811x_EnDis_t Enable);


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
                               HLW811x_EnDis_t Enable);


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
                                 HLW811x_EnDis_t Enable);


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
                         HLW811x_EnDis_t Enable);


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
                      HLW811x_IntOutFunc_t INT1, HLW811x_IntOutFunc_t INT2);


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
HLW811x_SetIntMask(HLW811x_Handler_t *Handler, uint16_t Mask);


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
HLW811x_GetIntFlag(HLW811x_Handler_t *Handler, uint16_t *Mask);


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
HLW811x_GetResIntFlag(HLW811x_Handler_t *Handler, uint16_t *Mask);



/**
 ==================================================================================
                             ##### Data Functions #####                            
 ==================================================================================
 */

/**
 * @brief  Get the RMS value of the voltage in V
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to store the data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetRmsU(HLW811x_Handler_t *Handler, float *Data);


/**
 * @brief  Get the RMS value of the current channel A in A
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to store the data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetRmsIA(HLW811x_Handler_t *Handler, float *Data);


/**
 * @brief  Get the RMS value of the current channel B in A
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to store the data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetRmsIB(HLW811x_Handler_t *Handler, float *Data);


/**
 * @brief  Get active power of channel A in W
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to store the data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetPowerPA(HLW811x_Handler_t *Handler, float *Data);


/**
 * @brief  Get active power of channel B in W
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to store the data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetPowerPB(HLW811x_Handler_t *Handler, float *Data);


/**
 * @brief  Get the apparent power of selected channel in W
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to store the data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetPowerS(HLW811x_Handler_t *Handler, float *Data);


/**
 * @brief  Get Energy of channel A in KWH
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to store the data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetEnergyA(HLW811x_Handler_t *Handler, float *Data);


/**
 * @brief  Get Energy of channel B in KWH
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to store the data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetEnergyB(HLW811x_Handler_t *Handler, float *Data);

/**
 * @brief  Get the frequency of the voltage channel in Hz
 * @param  Handler: Pointer to handler
 * @param  Data: Pointer to store the data
 * @retval HLW811x_Result_t
 *         - HLW811X_OK: Operation was successful.
 *         - HLW811X_FAIL: Failed to send or receive data.
 */
HLW811x_Result_t
HLW811x_GetFreqU(HLW811x_Handler_t *Handler, float *Data);



#ifdef __cplusplus
}
#endif

#endif //! _HLW811X_H_