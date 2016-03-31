/******************************************************************************
 * Modbus Library
 * Copyright (C) 2015-2016 AEA s.r.l. Loccioni Group - Elctronic Design Dept.
 *
 * Authors:
 *  Matteo Civale <m.civale@loccioni.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 ******************************************************************************/

#ifndef __LOCCIONI_MODBUS_H
#define __LOCCIONI_MODBUS_H

#include "libohiboard.h"

/**
 * In this file you must define the MAP information:
 *     #define LOCCIONI_MODBUS_MAPSIZE NNN
 */
//#include "board.h"

#define LOCCIONI_MODBUS_LIBRARY_VERSION     "1.1"
#define LOCCIONI_MODBUS_LIBRARY_VERSION_M   1
#define LOCCIONI_MODBUS_LIBRARY_VERSION_m   1
#define LOCCIONI_MODBUS_LIBRARY_TIME        0

/* The dma transfer is supported only for k12d5 and k64f12 uController */
#define ENABLE_DMA_TRANSFER 1  //to improve velocity set to 1 and include DMA



#if ENABLE_DMA_TRANSFER
 #include "dma.h"
#endif




#define RX_BUFFER_LEN 300
#define LOCCIONI_MODBUS_MAPSIZE 110

typedef enum
{
    MODBUS_PHYSICALTYPE_RS232,
    MODBUS_PHYSICALTYPE_RS485
} Modbus_PhysicalType;

/**
 * Define receiver buffer type
 */
typedef union
{
    uint8_t raw[RX_BUFFER_LEN];

    struct
    {
	    uint8_t address;
	    uint8_t function;
	    uint8_t data[RX_BUFFER_LEN-2];
    }field;

} Modbus_RxBuffer;

/**
 * Define logical error type
 */
typedef enum
{
    MODBUS_LOGICERROR_NO_ERROR             = 0,
    MODBUS_LOGICERROR_ILLEGAL_FUNCTION     = 1,
    MODBUS_LOGICERROR_ILLEGAL_DATA_ADDRESS = 2,
    MODBUS_LOGICERROR_ILLEGAL_DATA_VALUE   = 3,
    MODBUS_LOGICERROR_NEG_ACK              = 7,
} Modbus_LogicError;

/**
 * Define transmission mode type
 */
typedef enum
{
    MODBUS_TRANSMISSIONMODE_RTU,
    MODBUS_TRANSMISSIONMODE_ASCII,
} Modbus_TransmissionMode;

/**
 * Define Modbus state type
 */
typedef enum
{
    MODBUS_STATE_IDLE,
    MODBUS_STATE_NEW_MESSAGE,
    MODBUS_STATE_IN_RECEPTION,
    MODBUS_STATE_PARITY_ERROR,
    MODBUS_STATE_SYNTAX_ERROR,
    MODBUS_STATE_CRC_ERROR
}Modbus_State;

/**
 * Define device type
 */
typedef enum
{
    MODBUS_DEVICETYPE_MASTER,
    MODBUS_DEVICETYPE_SLAVE,
}Modbus_DeviceType;

/**
 * Define serial configuration
 */
typedef enum
{
    MODBUS_SERIALCONFIG_1_8_1_ODD,  /**< 1 start, 8 data, Odd parity, 1 stop. */
} Modbus_SerialConfig;

/**
 * Define errors for modbus
 */
typedef enum
{
    MODBUS_ERRORS_NO_ERROR,
    MODBUS_ERRORS_NO_FREE_DEVICE,
    MODBUS_ERRORS_UART_OPEN,
} Modbus_Errors;

/**
 * Define all the parameters to configure Modbus.
 */
typedef struct _Modbus_Config
{
    Modbus_DeviceType type;               /**< Select the type of the device. */

    Modbus_TransmissionMode txMode;            /**< The type of transmission. */

    Ftm_DeviceHandle counter;   /**< Timer device handler for delay counting. */

    Uart_DeviceHandle com;                           /**< UART device handle. */
    Uart_RxPins rx;
    Uart_TxPins	tx;
    Gpio_Pins   de;
    Modbus_PhysicalType phy;            /**< Physical type for communication. */
    uint32_t baudrate;                /**< Baudrate for serial communication. */
    Modbus_SerialConfig comConfig;

    uint8_t id;                                  /**< Id of the current node. */


#if ENABLE_DMA_TRANSFER
    Dma_ChannelType dmaCh;
    Dma_DeviceHandle dma;
#endif
} Modbus_Config;

typedef struct _Modbus_Device
{
    Ftm_DeviceHandle counter;   /**< Timer device handler for delay counting. */

    Uart_DeviceHandle com;                           /**< UART device handle. */
    Gpio_Pins de;

    uint8_t id;                                  /**< Id of the current node. */

    Modbus_State state;
    Modbus_LogicError logicError;
    uint8_t errorParity;

    uint8_t position;

    Modbus_RxBuffer buffer;
    uint8_t length;
    uint8_t timeout;

    uint8_t status;
    uint16_t map[LOCCIONI_MODBUS_MAPSIZE];

#if ENABLE_DMA_TRANSFER
    Dma_ChannelType dmaChannel;
    dma_ConfigType dmaConfig;
    Dma_DeviceHandle dma;
#endif



} Modbus_Device;

Modbus_Errors Modbus_init (Modbus_Device *dev, Modbus_Config *config);
void Modbus_listener (Modbus_Device *dev);

uint16_t Modbus_get (Modbus_Device *dev, uint8_t position);
void Modbus_set (Modbus_Device *dev, uint8_t position, uint16_t value);

#endif /* __LOCCIONI_MODBUS_H */
