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

/**
 * @file modbus.c
 * @author Matteo Civale <m.civale@loccioni.com>
 * @brief Modbus implementation
 */

#include "modbus.h"

static unsigned short Modbus_crc16_tbl[];

#define SET_VAR16(X)    (*X<<8&0xFF00)|*(X+1)
#define U16_H(X)		(uint8_t) 0x00FF&(X>>8)
#define U16_L(X)		(uint8_t) 0x00FF&X
#define POL_G           0xA001//generator polynomial

#define MODBUS_MAX_DEVICE     4


void Modbus_uartIsr0 (void);
void Modbus_uartIsr1 (void);
void Modbus_uartIsr2 (void);
void Modbus_uartIsr3 (void);

void Modbus_counterIsr0 (void);
void Modbus_counterIsr1 (void);
void Modbus_counterIsr2 (void);
void Modbus_counterIsr3 (void);

void Modbus_dmaStopIsr0 (void);
void Modbus_dmaStopIsr1 (void);
void Modbus_dmaStopIsr2 (void);
void Modbus_dmaStopIsr3 (void);

typedef struct Modbus_RegisteredDevice
{
    Modbus_Device *dev;
    void (* uartIsr)(void);
    void (* counterIsr)(void);
    bool enabled;
    void (* dmaStopIsr)(void);

} Modbus_RegisteredDevice;

static Modbus_RegisteredDevice Modbus_devs[] =
{
        {0,Modbus_uartIsr0,Modbus_counterIsr0,FALSE, Modbus_dmaStopIsr0},
        {0,Modbus_uartIsr1,Modbus_counterIsr1,FALSE, Modbus_dmaStopIsr1},
        {0,Modbus_uartIsr2,Modbus_counterIsr2,FALSE, Modbus_dmaStopIsr2},
        {0,Modbus_uartIsr3,Modbus_counterIsr3,FALSE, Modbus_dmaStopIsr3},
};

static uint16_t Modbus_crcCheck (uint8_t *head, uint8_t length)
{
    uint8_t combValue;
    uint16_t crc = 0xFFFF;

    for (uint8_t n = 0; n < length; ++n)
    {
        combValue = (crc>>8) ^ head[n];
        crc = (crc<<8) ^ Modbus_crc16_tbl[combValue & 0x00FF];
    }
    return (uint16_t)crc;
}

static void Modbus_sendLogicalError (Modbus_Device *dev, Modbus_LogicError error)
{
    dev->logicError = error;
}

static void Modbus_analizeFrame (Modbus_Device *dev)
{
    uint16_t memPosition;
    uint16_t numWord;
    uint16_t crcCode;
    uint8_t numByte;
    uint16_t i,j;

    uint8_t rxId = dev->buffer.field.address;

    if ((rxId == 0) || (rxId == dev->id))
    {
        switch (dev->buffer.field.function)
        {
        case 3:
        case 4: // request 16 bit data register
            memPosition = SET_VAR16(dev->buffer.field.data);
            numWord = SET_VAR16(&dev->buffer.field.data[2]);//location 2-3
            numByte = numWord * 2;
            if ((memPosition + numWord) > LOCCIONI_MODBUS_MAPSIZE)
            {
                Modbus_sendLogicalError(dev,MODBUS_LOGICERROR_ILLEGAL_DATA_ADDRESS);
                return;
            }

            // start to compile transmitting buffer
            dev->buffer.field.data[0] = numByte;
            j=1; // start pos

            for (i = memPosition; i < memPosition + numWord; i++)
            {
                dev->buffer.field.data[j]   = U16_H(dev->map[i]);
                dev->buffer.field.data[j+1] = U16_L(dev->map[i]);
                j += 2;
            }

            // calculate and put in the message the CRC message code
            // add two address and function byte and #byte
            crcCode = Modbus_crcCheck(dev->buffer.raw,numByte+3);
            dev->buffer.field.data[j] = U16_H(crcCode);
            dev->buffer.field.data[j+1] = U16_L(crcCode);
            dev->length = numByte + 5; // 2 crc16 byte + address+function+#byte
              // Uart_sendData(dev->com,dev->buffer.raw,dev->length);
            break;
        case 5:

            break;

        case 6: // set a single 16 bit var
            memPosition = SET_VAR16(dev->buffer.field.data);
            dev->map[memPosition] = SET_VAR16(&dev->buffer.field.data[2]);
            // response with the same trasmitted message
              // Uart_sendData(dev->com,dev->buffer.raw,dev->length);
            break;

        case 7: // read status
            dev->buffer.field.data[0] = dev->status;
            // add two address and function byte
            crcCode = Modbus_crcCheck(dev->buffer.raw,3);
            dev->buffer.field.data[1] = U16_H(crcCode);
            dev->buffer.field.data[2] = U16_L(crcCode);
            dev->length = 5; //2 crc16 byte + address+function
            //   Uart_sendData(dev->com,dev->buffer.raw,dev->length);
            break;

        case 16: //set a multiple 16 bit var

            memPosition = SET_VAR16(&dev->buffer.field.data[0]); // location 0-1
            numWord = SET_VAR16(&dev->buffer.field.data[2]);     // location 2-3
            numByte = dev->buffer.field.data[4];                 // location 4

            // check address error
            if ((memPosition+numWord) > LOCCIONI_MODBUS_MAPSIZE)
            {
                Modbus_sendLogicalError(dev,MODBUS_LOGICERROR_ILLEGAL_DATA_ADDRESS);
                return;
            }

            j = 5; // start location
            for (i = memPosition; i < memPosition+numWord; i++)
            {
                dev->map[i] = SET_VAR16(&dev->buffer.field.data[j]);
                j += 2;
            }

            dev->buffer.field.data[2] = U16_H(numWord);
            dev->buffer.field.data[3] = U16_L(numWord);

            // Add two address and function byte
            crcCode = Modbus_crcCheck(dev->buffer.raw,6);
            dev->buffer.field.data[4] = U16_H(crcCode);
            dev->buffer.field.data[5] = U16_L(crcCode);
            dev->length = 8; // 2 crc16 byte + address+function
               // Uart_sendData(dev->com,dev->buffer.raw,dev->length);
            break;

        case 17:
            break;
        }
#if !ENABLE_DMA_TRANSFER

        Uart_sendData(dev->com,dev->buffer.raw,dev->length);

#else
        /* Set and start DMA transfer */
        dev->dmaConfig.nOfCycle=dev->length;
        Dma_startChannel(dev->dma, &dev->dmaConfig);

#endif
    }
}

Modbus_Errors Modbus_init (Modbus_Device *dev, Modbus_Config *config)
{
    Uart_Config comConfig;
    Ftm_Config counterConfig;
    System_Errors error=0;
    uint8_t position = 0,i;
    bool isFreeDevice = FALSE;

    /* Search free modbus device */
    for (i = 0; i < MODBUS_MAX_DEVICE; ++i)
    {
        if (Modbus_devs[i].enabled == FALSE)
        {
            position = i;
            Modbus_devs[i].enabled = TRUE;
            Modbus_devs[i].dev = dev;
            isFreeDevice = TRUE;
            break;
        }
    }

    if (!isFreeDevice) return MODBUS_ERRORS_NO_FREE_DEVICE;

    /* set clock source */
	comConfig.clockSource  = UART_CLOCKSOURCE_BUS;

	/* set baudrate, parity and data length */
	switch (config->comConfig)
	{
    case MODBUS_SERIALCONFIG_1_8_1_ODD:
        comConfig.dataBits     = UART_DATABITS_EIGHT;
        comConfig.parity       = UART_PARITY_NONE;
        comConfig.baudrate     = config->baudrate;
        comConfig.stop         = UART_STOPBITS_ONE;

#if defined(LIBOHIBOARD_KL25Z4)
        comConfig.oversampling = 16;
#endif
	    break;
	}

	/* UART pin setting */
	comConfig.rxPin = config->rx;
	comConfig.txPin = config->tx;

    if (config->phy == MODBUS_PHYSICALTYPE_RS485)
    {
        error = Gpio_config (config->de, GPIO_PINS_OUTPUT);
        if(error) return error;
        dev->de = config->de;
        /* Transmission line free */
        Gpio_set (dev->de);
    }
	else
	{
    	dev->de = GPIO_PINS_NONE;
	}

    /* Open serial interface */
    comConfig.callbackRx = Modbus_devs[position].uartIsr;
    comConfig.callbackTx=0;

#if defined (LIBOHIBOARD_KL25Z4)     || \
	defined (LIBOHIBOARD_FRDMKL25Z)  || \
	defined (LIBOHIBOARD_K12D5)      || \
    defined (LIBOHIBOARD_K64F12)     || \
	defined (LIBOHIBOARD_FRDMK64F)   || \
	defined (LIBOHIBOARD_KV46F)      || \
	defined (LIBOHIBOARD_TRWKV46F)

    error = Uart_open (config->com, &comConfig);
#else
    error = Uart_open (config->com, 0, &comConfig);
#endif

    if (error != ERRORS_NO_ERROR) return MODBUS_ERRORS_UART_OPEN;

    dev->com = config->com;
    dev->state = MODBUS_STATE_IDLE;
    dev->position = 0;

    /* initialize counter for temporization */
    counterConfig.mode = FTM_MODE_FREE;
    counterConfig.timerFrequency = comConfig.baudrate / (11 * 3.5);
    counterConfig.initCounter = 0;

#if defined (LIBOHIBOARD_K64F12)     || \
	defined (LIBOHIBOARD_FRDMK64F)

    counterConfig.fault[0].pin = FTM_FAULTPINS_STOP;
    counterConfig.triggerChannel = FTM_TRIGGER_NOCH;
#endif

    Ftm_init(config->counter, Modbus_devs[position].counterIsr, &counterConfig);
    dev->counter = config->counter;

    dev->id = config->id;

#if ENABLE_DMA_TRANSFER

    dev->dmaConfig.channel             = config->dmaCh;

    dev->dmaConfig.sourceAddress       = (uint32_t)dev->buffer.raw;
    dev->dmaConfig.destinationAddress  = (uint32_t)Uart_getRxRegisterAddress(config->com);

    dev->dmaConfig.sourceOff           = 0x01;
    dev->dmaConfig.destinationOff      = 0x00;

    dev->dmaConfig.sSize               = DMA_DATASIZE_8BIT;
    dev->dmaConfig.dSize               = DMA_DATASIZE_8BIT;

    dev->dmaConfig.nByteforReq         = 1;

    dev->dmaConfig.nOfCycle            = 0;
    dev->dmaConfig.enableTimerTrig     = FALSE;

    dev->dmaConfig.lsAdjust            = 0;
    dev->dmaConfig.ldAdjust            = 0;

    dev->dmaConfig.pHandler            = config->com;

    /* Save dma device handler */
    dev->dma=config->dma;

    Dma_init(dev->dma, dev->dmaConfig.pHandler, DMA_REQ_UART_TRANSMIT, dev->dmaConfig.channel, Modbus_devs[position].dmaStopIsr);

#endif

    return MODBUS_ERRORS_NO_ERROR;
}

static void Modbus_uartIsr (Modbus_Device *dev)
{
    System_Errors error;
    uint8_t id;

    // put the new received byte in the buffer
    error = Uart_getChar (dev->com, &dev->buffer.raw[dev->position]);
    if (error == ERRORS_UART_PARITY) dev->errorParity |= 1;
    // update position pointer
    dev->position++;
    dev->position = dev->position % RX_BUFFER_LEN;

    dev->state = MODBUS_STATE_IN_RECEPTION;
    // put to zero timeout flag
    dev->timeout = 0;
    // start count peripheral
    Ftm_enableInterrupt(dev->counter);
}

void Modbus_uartIsr0 (void)
{
    Modbus_Device* dev = Modbus_devs[0].dev;

    if (dev != 0)
        Modbus_uartIsr(dev);
    else
        return; /* TODO: ERROR */
}

void Modbus_uartIsr1 (void)
{
    Modbus_Device* dev = Modbus_devs[1].dev;

    if (dev != 0)
        Modbus_uartIsr(dev);
    else
        return; /* TODO: ERROR */
}

void Modbus_uartIsr2 (void)
{
    Modbus_Device* dev = Modbus_devs[2].dev;

    if (dev != 0)
        Modbus_uartIsr(dev);
    else
        return; /* TODO: ERROR */
}

void Modbus_uartIsr3 (void)
{
    Modbus_Device* dev = Modbus_devs[3].dev;

    if (dev != 0)
        Modbus_uartIsr(dev);
    else
        return; /* TODO: ERROR */
}

static void Modbus_counterIsr (Modbus_Device *dev)
{
    if (dev->state == MODBUS_STATE_IN_RECEPTION)
    {
        dev->timeout = 1;
        dev->state = MODBUS_STATE_NEW_MESSAGE;
        dev->length = dev->position;
        dev->position = 0;
    }
    Ftm_disableInterrupt(dev->counter);
}

void Modbus_counterIsr0 (void)
{
    Modbus_Device* dev = Modbus_devs[0].dev;

    if (dev != 0)
        Modbus_counterIsr(dev);
    else
        return; /* TODO: ERROR */
}

void Modbus_counterIsr1 (void)
{
    Modbus_Device* dev = Modbus_devs[1].dev;

    if (dev != 0)
        Modbus_counterIsr(dev);
    else
        return; /* TODO: ERROR */
}

void Modbus_counterIsr2 (void)
{
    Modbus_Device* dev = Modbus_devs[2].dev;

    if (dev != 0)
        Modbus_counterIsr(dev);
    else
        return; /* TODO: ERROR */
}

void Modbus_counterIsr3 (void)
{
    Modbus_Device* dev = Modbus_devs[3].dev;

    if (dev != 0)
        Modbus_counterIsr(dev);
    else
        return; /* TODO: ERROR */
}


void Modbus_dmaStopIsr0(void)
{
#if ENABLE_DMA_TRANSFER
    Dma_disableChannel(Modbus_devs[0].dev->dma, Modbus_devs[0].dev->dmaConfig.channel);
#endif
}

void Modbus_dmaStopIsr1(void)
{
#if ENABLE_DMA_TRANSFER
    Dma_disableChannel(Modbus_devs[1].dev->dma, Modbus_devs[1].dev->dmaConfig.channel);
#endif
}

void Modbus_dmaStopIsr2(void)
{
#if ENABLE_DMA_TRANSFER
    Dma_disableChannel(Modbus_devs[2].dev->dma, Modbus_devs[2].dev->dmaConfig.channel);
#endif
}

void Modbus_dmaStopIsr3(void)
{
#if ENABLE_DMA_TRANSFER
    Dma_disableChannel(Modbus_devs[3].dev->dma, Modbus_devs[3].dev->dmaConfig.channel);
#endif
}

void Modbus_listener (Modbus_Device *dev)
{
    uint16_t crcCodeRx;
    uint16_t crcCodeCalc;
    bool crcFlag = FALSE;

    if (dev->state == MODBUS_STATE_NEW_MESSAGE)
    {
        dev->state = MODBUS_STATE_IDLE;

        crcCodeRx = (dev->buffer.raw[dev->length-2] << 8) |
                    (dev->buffer.raw[dev->length-1]);
        crcCodeCalc = Modbus_crcCheck(dev->buffer.raw,dev->length-2);

        if (crcCodeCalc == crcCodeRx)  crcFlag = 1;

        /* Check CRC16 and parity error */
        if ((!dev->errorParity) && (crcFlag))
        {
            /* If there isn't error */
            Modbus_analizeFrame(dev);
        }
        dev->errorParity = 0;
    }

    if (dev->logicError != MODBUS_LOGICERROR_NO_ERROR)
    {
        dev->buffer.field.function |= 0x80;
        dev->buffer.field.data[0] = dev->logicError;
        crcCodeCalc = Modbus_crcCheck(dev->buffer.raw,3);
        dev->buffer.field.data[1] = U16_H(crcCodeCalc);
        dev->buffer.field.data[2] = U16_L(crcCodeCalc);
        Uart_sendData(dev->com,dev->buffer.raw,5);
        dev->logicError = MODBUS_LOGICERROR_NO_ERROR;
    }
}

uint16_t Modbus_get (Modbus_Device *dev, uint16_t position)
{
    if (position > (LOCCIONI_MODBUS_MAPSIZE-1)) return 0;

    return dev->map[position];
}

void Modbus_set (Modbus_Device *dev, uint16_t position, uint16_t value)
{
    if (position > (LOCCIONI_MODBUS_MAPSIZE-1)) return;

    dev->map[position] = value;
}

Modbus_Errors Modbus_clearMemoryArea (Modbus_Device* dev, uint16_t start, uint16_t length)
{
  uint16_t i;
  uint16_t last = start + length;

  if (last > LOCCIONI_MODBUS_MAPSIZE - 1)
      return MODBUS_ERRORS_WRONG_LENGTH;

  for (i = start; i < last; i++)
  {
      Modbus_set(dev, i, 0);
  }
  return MODBUS_ERRORS_NO_ERROR;
}

static unsigned short Modbus_crc16_tbl[] =
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
