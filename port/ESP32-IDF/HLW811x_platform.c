/**
 **********************************************************************************
 * @file   HLW811x_platform.c
 * @author Hossein.M (https://github.com/Hossein-M98)
 * @brief  A sample Platform dependent layer for HLW811x Driver
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
#include "HLW811x_platform.h"
#include "freertos/FreeRTOS.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"
#if (HLW811X_CONFIG_SUPPORT_SPI)
#include "driver/spi_master.h"
#endif
#if (HLW811X_CONFIG_SUPPORT_UART)
#include "driver/uart.h"
#endif


/* Private Variables ------------------------------------------------------------*/
#if (HLW811X_CONFIG_SUPPORT_SPI)
static spi_device_handle_t spi_device_handle = {0};
#endif

#if (HLW811X_CONFIG_SUPPORT_UART)
static const uint16_t BaudRateTable[] =
{
  9600, 19200, 38400
};
#endif



/**
 ==================================================================================
                           ##### Private Functions #####                           
 ==================================================================================
 */

#if (HLW811X_CONFIG_SUPPORT_SPI)
static int8_t
HLW811x_Platform_SPI_Init(void)
{
  gpio_reset_pin(HLW811X_SPI_CSN_GPIO);
  gpio_set_direction(HLW811X_SPI_CSN_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(HLW811X_SPI_CSN_GPIO, 1);

  gpio_reset_pin(HLW811X_SPI_CLK_GPIO);
  gpio_set_direction(HLW811X_SPI_CLK_GPIO, GPIO_MODE_OUTPUT);

  gpio_reset_pin(HLW811X_SPI_MOSI_GPIO);
  gpio_set_direction(HLW811X_SPI_MOSI_GPIO, GPIO_MODE_OUTPUT);

  gpio_reset_pin(HLW811X_SPI_MISO_GPIO);

  const spi_bus_config_t spi_bus_config =
  {
    .mosi_io_num = HLW811X_SPI_MOSI_GPIO,
    .miso_io_num = HLW811X_SPI_MISO_GPIO,
    .sclk_io_num = HLW811X_SPI_CLK_GPIO,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 0,
    .flags = 0,
    .intr_flags = 0
  };
  if (spi_bus_initialize(HLW811X_SPI_NUM, &spi_bus_config, 0) != ESP_OK)
    return -1;

  const spi_device_interface_config_t spi_device_interface_config =
  {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .mode = 3,
    .duty_cycle_pos = 0,
    .cs_ena_pretrans = 0,
    .cs_ena_posttrans = 0,
    .clock_speed_hz = HLW811X_SPI_RATE,
    .input_delay_ns = 0,
    .spics_io_num = -1,
    .flags = 0,
    .queue_size = 1,
    .pre_cb = (void *)0,
    .post_cb = (void *)0
  };
  if (spi_bus_add_device(HLW811X_SPI_NUM, &spi_device_interface_config, &spi_device_handle) != ESP_OK)
    return -2;

  return 0;
}

static int8_t
HLW811x_Platform_SPI_DeInit(void)
{
  gpio_reset_pin(HLW811X_SPI_CLK_GPIO);
  gpio_reset_pin(HLW811X_SPI_MOSI_GPIO);
  gpio_reset_pin(HLW811X_SPI_MISO_GPIO);
  gpio_reset_pin(HLW811X_SPI_CSN_GPIO);
  return 0;
}

static int8_t
HLW811x_Platform_SPI_SendReceive(uint8_t *SendData,
                                 uint8_t *ReceiveData,
                                 uint8_t Len)
{
  spi_transaction_t spi_transaction = {0};
  uint8_t TxBuff[SOC_SPI_MAXIMUM_BUFFER_SIZE] = {0xFF};

  spi_transaction.flags = 0;
  spi_transaction.length = SOC_SPI_MAXIMUM_BUFFER_SIZE * 8;
  spi_transaction.rxlength = 0;

  while (Len > SOC_SPI_MAXIMUM_BUFFER_SIZE)
  {
    spi_transaction.tx_buffer = SendData ? SendData : TxBuff;
    spi_transaction.rx_buffer = ReceiveData;
    if (spi_device_polling_transmit(spi_device_handle, &spi_transaction) != ESP_OK)
      return -1;

    Len -= SOC_SPI_MAXIMUM_BUFFER_SIZE;
    if (ReceiveData)
      ReceiveData += SOC_SPI_MAXIMUM_BUFFER_SIZE;
    if (SendData)
      SendData += SOC_SPI_MAXIMUM_BUFFER_SIZE;
  }

  if (Len)
  {
    spi_transaction.length = Len * 8;
    spi_transaction.rxlength = 0;
    spi_transaction.tx_buffer = SendData ? SendData : TxBuff;
    spi_transaction.rx_buffer = ReceiveData;
    if (spi_device_polling_transmit(spi_device_handle, &spi_transaction) != ESP_OK)
      return -1;
  }

  return 0;
}

static int8_t
HLW811x_Platform_SPI_SetLevelSCSN(uint8_t Level)
{
  gpio_set_level(HLW811X_SPI_CSN_GPIO, Level);
  return 0;
}
#endif

#if (HLW811X_CONFIG_SUPPORT_UART)
static int8_t
HLW811x_Platform_UART_Init(void)
{
  esp_err_t err = ESP_OK;
  uart_config_t uart_config = {0};

  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.source_clk = UART_SCLK_APB;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.baud_rate = BaudRateTable[HLW811X_UART_BAUD];
  uart_config.parity = UART_PARITY_EVEN;
  uart_config.stop_bits = UART_STOP_BITS_1;

  // Configure UART parameters
  err = uart_param_config(HLW811X_UART_NUM, &uart_config);
  if (err != ESP_OK)
  {
    return -1;
  }

  // Set UART pins(TX, RX, RTS, CTS)
  err = uart_set_pin(HLW811X_UART_NUM,
                     HLW811X_UART_TX_GPIO, HLW811X_UART_RX_GPIO,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (err != ESP_OK)
  {
    return -2;
  }

  // Install UART driver using an event queue here
  err = uart_driver_install(HLW811X_UART_NUM, 256, 256,
                            0, NULL, 0);
  if (err != ESP_OK)
  {
    return -3;
  }

  return 0;
}

static int8_t
HLW811x_Platform_UART_DeInit(void)
{
  uart_driver_delete(HLW811X_UART_NUM);
  return 0;
}

#include <stdio.h>
static int8_t
HLW811x_Platform_UART_Send(uint8_t *Data, uint8_t Len)
{
  int Result = 0;

  Result = uart_write_bytes(HLW811X_UART_NUM, (const void *)Data, Len);
  if (Result == -1)
    return -1;
  else if (Result != Len)
    return -2;

  return 0;
}

static int8_t
HLW811x_Platform_UART_Receive(uint8_t *Data, uint8_t Len)
{
  int Result = 0;

  Result = uart_read_bytes(HLW811X_UART_NUM, (void *)Data, Len, 10 / portTICK_PERIOD_MS);
  if (Result == -1)
    return -1;
  else if (Result != Len)
    return -2;

  return 0;
}
#endif



/**
 ==================================================================================
                            ##### Public Functions #####                           
 ==================================================================================
 */
 
#if (HLW811X_CONFIG_SUPPORT_SPI)
/**
 * @brief  Initialize platform device to communicate HLW811x through SPI.
 * @param  Handler: Pointer to handler
 * @retval None
 */
void
HLW811x_Platform_Init_SPI(HLW811x_Handler_t *Handler)
{
  HLW811X_PLATFORM_LINK_INIT(Handler, HLW811x_Platform_SPI_Init);
  HLW811X_PLATFORM_LINK_DEINIT(Handler, HLW811x_Platform_SPI_DeInit);
  HLW811X_PLATFORM_LINK_SPI_SENDRECEIVE(Handler, HLW811x_Platform_SPI_SendReceive);
  HLW811X_PLATFORM_LINK_SPI_SETLEVELSCSN(Handler, HLW811x_Platform_SPI_SetLevelSCSN);
  HLW811X_PLATFORM_SET_COMMUNICATION(Handler, HLW811X_COMMUNICATION_SPI);
}
#endif

#if (HLW811X_CONFIG_SUPPORT_UART)
/**
 * @brief  Initialize platform device to communicate HLW811x through UART.
 * @param  Handler: Pointer to handler
 * @retval None
 */
void
HLW811x_Platform_Init_UART(HLW811x_Handler_t *Handler)
{
  HLW811X_PLATFORM_LINK_INIT(Handler, HLW811x_Platform_UART_Init);
  HLW811X_PLATFORM_LINK_DEINIT(Handler, HLW811x_Platform_UART_DeInit);
  HLW811X_PLATFORM_LINK_UART_SEND(Handler, HLW811x_Platform_UART_Send);
  HLW811X_PLATFORM_LINK_UART_RECEIVE(Handler, HLW811x_Platform_UART_Receive);
  HLW811X_PLATFORM_SET_COMMUNICATION(Handler, HLW811X_COMMUNICATION_UART);
}
#endif
