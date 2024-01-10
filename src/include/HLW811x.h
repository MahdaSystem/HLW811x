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



#ifdef __cplusplus
}
#endif

#endif //! _HLW811X_H_