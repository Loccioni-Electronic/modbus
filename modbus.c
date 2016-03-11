/* Copyright (C) 2014-2015 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Matteo Civale <matteo.civale@gmail.com>
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
 ******************************************************************************/

/**
 * @file modbus.c
 * @author Matteo Civale <matteo.civale@gmail.com>
 * @brief modbus  implementation for KL25Z4 and FRDM-KL25Z.
 *
 */

//#include "MOD_BUS.h"
//#include "ftm.h"
//#include "uart.h"
#include "libohiboard.h"
#include "modbus.h"
#include "var_mapping.h"
#include "configuration.h"


#define SET_VAR16(X)    (*X<<8&0xFF00)|*(X+1)
#define U16_H(X)		(uint8_t) 0x00FF&(X>>8)
#define U16_L(X)		(uint8_t) 0x00FF&X
#define POL_G            0xA001//generator polynomial




void Int_function(void);
void Set_End_Message(void);

ModBus_handler ModBus_interface={
    .state=IDLE,
    .interface_is_set=0,
};


System_Errors ModBus_inizialize(Modbus_Config_Type *Bus_config)
{
    Uart_Config COM_config;
    System_Errors error;
    Ftm_Config FTM_config;

//set clock surce
	COM_config.clockSource  = UART_CLOCKSOURCE_BUS;

//set boud parity and data length
	switch (Bus_config->Serial_Config)
	{
    case S1_D8_Odd_Stop1:
	    COM_config.dataBits     = UART_DATABITS_EIGHT;
	    COM_config.parity       = UART_PARITY_NONE;
	    COM_config.baudrate     = Bus_config->Baudrate;
	    COM_config.stop         = UART_STOPBITS_ONE;

        #if defined(LIBOHIBOARD_KL25Z4)
	        COM_config.oversampling=16;
        #endif

	    break;
	}
//pin uart setting
    COM_config.rxPin=Bus_config->RX;
    COM_config.txPin=Bus_config->TX;

    if (Bus_config->PLayer==RS485)
    {
        error=Gpio_config (Bus_config->DE, GPIO_PINS_OUTPUT);
        if(error) return error;
        ModBus_interface.DE=Bus_config->DE;
        Gpio_set (ModBus_interface.DE);
    }


//open serial interface
#if defined (LIBOHIBOARD_K12D5)      || \
    defined (LIBOHIBOARD_K64F12)     || \
	defined (LIBOHIBOARD_FRDMK64F)

    COM_config.callbackRx=Int_function;
    error=Uart_open (Bus_config->COM, &COM_config);
#else
    error=Uart_open (Bus_config->COM, Int_function, &COM_config);
#endif

    if(error) return error;

    ModBus_interface.config=Bus_config;
    ModBus_interface.state=IDLE;
    ModBus_interface.pos=0;

    ModBus_interface.uart_handler=Bus_config->COM;


//initialize counter for temporization

//set counter configuration

    FTM_config.mode=FTM_MODE_FREE;
    FTM_config.timerFrequency=COM_config.baudrate/(11*3.5);
    FTM_config.initCounter=0;

#if defined (LIBOHIBOARD_K12D5)      || \
    defined (LIBOHIBOARD_K64F12)     || \
	defined (LIBOHIBOARD_FRDMK64F)

    FTM_config.fault[0].pin=FTM_FAULTPINS_STOP;
    FTM_config.triggerChannel=FTM_TRIGGER_NOCH;
#endif
    Ftm_init (Bus_config->Counter,Set_End_Message,&FTM_config);
    //Ftm_disableInterrupt (Bus_config->Counter);//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    ModBus_interface.ftm_Handler=Bus_config->Counter;
    ModBus_interface.ID=Bus_config->ID;

    return error;
}

void Set_End_Message(void)
{
	//READY_TOG();
	//TEST_PIN_HIGH();

	if(ModBus_interface.state==IN_RECEPTION)
    {
    ModBus_interface.timeout_flag=1;
    ModBus_interface.state=NEW_MESSAGE;
    ModBus_interface.length=ModBus_interface.pos;
    ModBus_interface.pos=0;
    //LED_GREEN_ON();
    }
	Ftm_disableInterrupt(ModBus_interface.ftm_Handler);
	//TEST_PIN_2_LOW();
	//READY_LOW();
	//TEST_PIN_LOW();
}

void Int_function()
{
    System_Errors error;
    uint8_t ID;
    //READY_HIGHT();
    //LED_BLUE_ON();
    //TEST_PIN_HIGH();
    //TEST_PIN_2_HIGH();
    //put the new recived byte in the buffer
    error=Uart_getChar (ModBus_interface.uart_handler, &ModBus_interface.buffer.Raw[ModBus_interface.pos]);
    if(error==ERRORS_UART_PARITY) ModBus_interface.error_parity_flag|=1;
    //update position pointer
    ModBus_interface.pos++;
    ModBus_interface.pos=ModBus_interface.pos%RX_BUFFER_LEN;
    //LED_RED_ON();
    ModBus_interface.state=IN_RECEPTION;
    //put to zero timeout flag
    ModBus_interface.timeout_flag=0;
    //start count peripheral
    Ftm_enableInterrupt(ModBus_interface.ftm_Handler);
    //TEST_PIN_LOW();
}

void ModBus_listener()
{
    uint16_t crc_code_rx;
    uint16_t crc_code_calc;
    uint8_t crc_flag;
    //Uart_putChar(UART3, '3');
    if(ModBus_interface.state==NEW_MESSAGE)
    {

        ModBus_interface.state=IDLE;
      //  Uart_putChar(UART3, 'M');//--------------------

        crc_code_rx=(ModBus_interface.buffer.Raw[ModBus_interface.length-2]<<8)|(ModBus_interface.buffer.Raw[ModBus_interface.length-1]);
        crc_code_calc=CRC16_Check(ModBus_interface.buffer.Raw,ModBus_interface.length-2);
        crc_flag=0;

        if(crc_code_calc==crc_code_rx)	crc_flag=1;
        //check CRC16 and parity error occured

        if((!ModBus_interface.error_parity_flag)&&(crc_flag))
        {//se non ci sono errori
        //LED_RED_ON();
        ModBus_analizeFrame();
        //TODO: analizza il paccheto
        }
        //Uart_sendData (UART0,ModBus_interface.buffer.Raw,ModBus_interface.length);
        //LED_RED_ON();

        ModBus_interface.error_parity_flag=0;
    }

	if(ModBus_interface.Log_erroro!=NO_ERROR)
	{
    ModBus_interface.buffer.get_Field.Function|=0x80;
    ModBus_interface.buffer.get_Field.Data[0]=ModBus_interface.Log_erroro;
    crc_code_calc=CRC16_Check(ModBus_interface.buffer.Raw,3);
    ModBus_interface.buffer.get_Field.Data[1]=U16_H(crc_code_calc);
    ModBus_interface.buffer.get_Field.Data[2]=U16_L(crc_code_calc);
    //Gpio_clear (ModBus_interface.DE);
    Uart_sendData (ModBus_interface.uart_handler,ModBus_interface.buffer.Raw,5);
    //Gpio_set (ModBus_interface.DE);
    ModBus_interface.Log_erroro=NO_ERROR;
    }

}

uint16_t CRC16_Check(uint8_t *head,uint8_t len){

	uint8_t comb_val;
	uint16_t crc = 0xFFFF;
	for(uint8_t n=0;n<len;++n){
			comb_val = (crc>>8) ^ head[n];
			crc = (crc<<8) ^ modbus_16_tbl[comb_val & 0x00FF];
	}
	return (uint16_t)crc;
}



void ModBus_analizeFrame(void)
{
    uint16_t mem_pos;
    uint16_t num_word;
    uint16_t crc_code;
    uint8_t num_byte;
    uint16_t i;
    uint16_t j;

    uint8_t app=ModBus_interface.buffer.get_Field.Address;

    if((app==0)||(app==ModBus_interface.ID))
    {
    switch (ModBus_interface.buffer.get_Field.Function)
    {
    case 3:
    case 4: //request 16 bit data register
        mem_pos=SET_VAR16(ModBus_interface.buffer.get_Field.Data);
        num_word=SET_VAR16(&ModBus_interface.buffer.get_Field.Data[2]);//location 2-3
        num_byte=num_word*2;
        if((mem_pos+num_word)>MAX_MAP_ADDRESS)
        {
        ModBus_sendLogicalError(ILLEGAL_DATA_ADDRESS);
        return;
        }

        //start to compile transmissing buffer
        ModBus_interface.buffer.get_Field.Data[0]=num_byte;
        j=1;//start pos

        for(i=mem_pos;i<mem_pos+num_word;i++)
        {
	    //LED_BLUE_ON();
	    ModBus_interface.buffer.get_Field.Data[j]=U16_H(*Map[i]);
	    ModBus_interface.buffer.get_Field.Data[j+1]=U16_L(*Map[i]);
	    j=j+2;
        }
        //calculate and put in the message the CRC message code
        crc_code=CRC16_Check(ModBus_interface.buffer.Raw,num_byte+3);//add two address and function byte and #byte
        ModBus_interface.buffer.get_Field.Data[j]=U16_H(crc_code);
        ModBus_interface.buffer.get_Field.Data[j+1]=U16_L(crc_code);
        ModBus_interface.length=num_byte+5; //2 crc16 byte + address+function+#byte
        Uart_sendData (ModBus_interface.uart_handler,ModBus_interface.buffer.Raw,ModBus_interface.length);
        break;

    case 5:

        break;

    case 6://set a single 16 bit var
        mem_pos=SET_VAR16(ModBus_interface.buffer.get_Field.Data);
        *Map[mem_pos]=SET_VAR16(&ModBus_interface.buffer.get_Field.Data[2]);
        //response with the same trasmitted message
        Uart_sendData (ModBus_interface.uart_handler,ModBus_interface.buffer.Raw,ModBus_interface.length);
        break;

    case 7://read status
        ModBus_interface.buffer.get_Field.Data[0]=STATUS;
        crc_code=CRC16_Check(ModBus_interface.buffer.Raw,3);//add two address and function byte
        ModBus_interface.buffer.get_Field.Data[0]=U16_H(crc_code);
        ModBus_interface.buffer.get_Field.Data[1]=U16_L(crc_code);
        ModBus_interface.length=5; //2 crc16 byte + address+function
        Uart_sendData (ModBus_interface.uart_handler,ModBus_interface.buffer.Raw,ModBus_interface.length);
        break;

    case 16://set a multiple 16 bit var

        mem_pos=SET_VAR16(&ModBus_interface.buffer.get_Field.Data[0]); //location 0-1
        num_word=SET_VAR16(&ModBus_interface.buffer.get_Field.Data[2]);//locatio 2-3
        num_byte=ModBus_interface.buffer.get_Field.Data[4];//locatio 4
        //check address error
        if((mem_pos+num_word)>MAX_MAP_ADDRESS)
        {
		ModBus_sendLogicalError(ILLEGAL_DATA_ADDRESS);
		return;
        }

        j=5;//start location

        for(i=mem_pos;i<mem_pos+num_word;i++)
        {
        *Map[i]=SET_VAR16(&ModBus_interface.buffer.get_Field.Data[j]);
        j=j+2;
        }

        ModBus_interface.buffer.get_Field.Data[2]=U16_H(num_word);
        ModBus_interface.buffer.get_Field.Data[3]=U16_L(num_word);
        crc_code=CRC16_Check(ModBus_interface.buffer.Raw,6);//add two address and function byte
        ModBus_interface.buffer.get_Field.Data[4]=U16_H(crc_code);
        ModBus_interface.buffer.get_Field.Data[5]=U16_L(crc_code);
        ModBus_interface.length=8; //2 crc16 byte + address+function
        Uart_sendData (ModBus_interface.uart_handler,ModBus_interface.buffer.Raw,ModBus_interface.length);

        break;

    case 17:
        break;

	}
	}
}

void ModBus_sendLogicalError(ModBus_L_Error_Type error)
{
	ModBus_interface.Log_erroro=error;
}

static unsigned short modbus_16_tbl[] =
{
0x0000, 0xC1C0, 0x81C1, 0x4001, 0x01C3, 0xC003, 0x8002, 0x41C2,
0x01C6, 0xC006, 0x8007, 0x41C7, 0x0005, 0xC1C5, 0x81C4, 0x4004,
0x01CC, 0xC00C, 0x800D, 0x41CD, 0x000F, 0xC1CF, 0x81CE, 0x400E,
0x000A, 0xC1CA, 0x81CB, 0x400B, 0x01C9, 0xC009, 0x8008, 0x41C8,
0x01D8, 0xC018, 0x8019, 0x41D9, 0x001B, 0xC1DB, 0x81DA, 0x401A,
0x001E, 0xC1DE, 0x81DF, 0x401F, 0x01DD, 0xC01D, 0x801C, 0x41DC,
0x0014, 0xC1D4, 0x81D5, 0x4015, 0x01D7, 0xC017, 0x8016, 0x41D6,
0x01D2, 0xC012, 0x8013, 0x41D3, 0x0011, 0xC1D1, 0x81D0, 0x4010,
0x01F0, 0xC030, 0x8031, 0x41F1, 0x0033, 0xC1F3, 0x81F2, 0x4032,
0x0036, 0xC1F6, 0x81F7, 0x4037, 0x01F5, 0xC035, 0x8034, 0x41F4,
0x003C, 0xC1FC, 0x81FD, 0x403D, 0x01FF, 0xC03F, 0x803E, 0x41FE,
0x01FA, 0xC03A, 0x803B, 0x41FB, 0x0039, 0xC1F9, 0x81F8, 0x4038,
0x0028, 0xC1E8, 0x81E9, 0x4029, 0x01EB, 0xC02B, 0x802A, 0x41EA,
0x01EE, 0xC02E, 0x802F, 0x41EF, 0x002D, 0xC1ED, 0x81EC, 0x402C,
0x01E4, 0xC024, 0x8025, 0x41E5, 0x0027, 0xC1E7, 0x81E6, 0x4026,
0x0022, 0xC1E2, 0x81E3, 0x4023, 0x01E1, 0xC021, 0x8020, 0x41E0,
0x01A0, 0xC060, 0x8061, 0x41A1, 0x0063, 0xC1A3, 0x81A2, 0x4062,
0x0066, 0xC1A6, 0x81A7, 0x4067, 0x01A5, 0xC065, 0x8064, 0x41A4,
0x006C, 0xC1AC, 0x81AD, 0x406D, 0x01AF, 0xC06F, 0x806E, 0x41AE,
0x01AA, 0xC06A, 0x806B, 0x41AB, 0x0069, 0xC1A9, 0x81A8, 0x4068,
0x0078, 0xC1B8, 0x81B9, 0x4079, 0x01BB, 0xC07B, 0x807A, 0x41BA,
0x01BE, 0xC07E, 0x807F, 0x41BF, 0x007D, 0xC1BD, 0x81BC, 0x407C,
0x01B4, 0xC074, 0x8075, 0x41B5, 0x0077, 0xC1B7, 0x81B6, 0x4076,
0x0072, 0xC1B2, 0x81B3, 0x4073, 0x01B1, 0xC071, 0x8070, 0x41B0,
0x0050, 0xC190, 0x8191, 0x4051, 0x0193, 0xC053, 0x8052, 0x4192,
0x0196, 0xC056, 0x8057, 0x4197, 0x0055, 0xC195, 0x8194, 0x4054,
0x019C, 0xC05C, 0x805D, 0x419D, 0x005F, 0xC19F, 0x819E, 0x405E,
0x005A, 0xC19A, 0x819B, 0x405B, 0x0199, 0xC059, 0x8058, 0x4198,
0x0188, 0xC048, 0x8049, 0x4189, 0x004B, 0xC18B, 0x818A, 0x404A,
0x004E, 0xC18E, 0x818F, 0x404F, 0x018D, 0xC04D, 0x804C, 0x418C,
0x0044, 0xC184, 0x8185, 0x4045, 0x0187, 0xC047, 0x8046, 0x4186,
0x0182, 0xC042, 0x8043, 0x4183, 0x0041, 0xC181, 0x8180, 0x4040
};
