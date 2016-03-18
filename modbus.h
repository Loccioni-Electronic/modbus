/******************************************************************************
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

#define LOCCIONI_MODBUS_LIBRARY_VERSION     "1.1"
#define LOCCIONI_MODBUS_LIBRARY_VERSION_M   1
#define LOCCIONI_MODBUS_LIBRARY_VERSION_m   1
#define LOCCIONI_MODBUS_LIBRARY_TIME        0

//define costant
#define RX_BUFFER_LEN 30

static unsigned short modbus_16_tbl[];

typedef enum
{
    RS232,
	RS485
}PhysicalLayer_Type;

//define reciver buffer type
typedef union //RX_buffer
{
    uint8_t Raw[RX_BUFFER_LEN];

    struct
    {
	    uint8_t Address;
	    uint8_t Function;
	    uint8_t Data[RX_BUFFER_LEN-2];
    }get_Field;

} ModBus_RX_buffer_Type;

//define logical error type
typedef enum
{
    NO_ERROR			=0,
    ILLEGAL_FUNCTION	=1,
    ILLEGAL_DATA_ADDRESS=2,
    ILLEGAL_DATA_VALUE	=3,
    NEG_ACK				=7,
} ModBus_L_Error_Type;

//define trasmission mode type
typedef enum
{
    RTU_T_Mode,
    ASCII_T_TMode,
} ModBus_TM_Type;

//define Mod Bus state type
typedef enum
{
    IDLE,
    NEW_MESSAGE,
    IN_RECEPTION,
    PARITY_ERROR,
    SYNTAX_ERROR,
    CRCL_ERROR
}ModBus_State_Type;

//define device type
typedef enum
{
    MASTER_Device,
    SLAVE_Device,
}ModBus_Device_Type;

//define serial configuration
typedef enum
{
    S1_D8_Odd_Stop1,  //1 bit start,8 bit data, Odd parity, 1 stop bit
}ModBus_Serial_Config_Type;

//define
typedef struct _Modbus_Config_Type
{
    ModBus_Device_Type D_Type;

    ModBus_TM_Type T_Mode;

    uint32_t Baudrate;

    ModBus_Serial_Config_Type Serial_Config;

    Uart_DeviceHandle COM;

    Ftm_DeviceHandle Counter;

    PhysicalLayer_Type PLayer;



    Uart_RxPins RX;
    Uart_TxPins	TX;
    Gpio_Pins DE;

    uint8_t ID;

}Modbus_Config_Type;


typedef struct _Mod_Bus_handler
{
    Modbus_Config_Type *config;
    ModBus_State_Type state;
    uint8_t interface_is_set;
    uint8_t timeout_flag;
    uint8_t error_parity_flag;
    Ftm_DeviceHandle ftm_Handler;
    Uart_DeviceHandle uart_handler;
    Gpio_Pins DE;

    ModBus_RX_buffer_Type buffer;
    uint8_t pos;
    uint8_t length;
    uint8_t ID;
    ModBus_L_Error_Type Log_erroro;
}ModBus_handler;

extern ModBus_handler ModBus_interface;

System_Errors ModBus_inizialize(Modbus_Config_Type *Bus_config);
void ModBus_listener(void);
uint16_t CRC16_Check(uint8_t *head,uint8_t len);
void ModBus_analizeFrame(void);
void ModBus_sendLogicalError(ModBus_L_Error_Type error);

#endif /* __LOCCIONI_MODBUS_H */
