#include "ad5940.h"
#include <SPI.h>

#define SPI_CS_AD5940_Pin D7
#define AD5940_ResetPin   A1
#define AD5940_IntPin     A2

int csPin = SPI_CS_AD5940_Pin;
int resetPin = AD5940_ResetPin;
int intPin = AD5940_IntPin;

// -----------------------------------------------------------------------------
volatile static uint32_t ucInterrupted = 0;       /* Flag to indicate interrupt occurred */

uint32_t AD5940_GetMCUIntFlag(void)
{
    return ucInterrupted;
}

uint32_t AD5940_ClrMCUIntFlag(void)
{
    ucInterrupted = 0;
    return 1;
}

/* MCU related external line interrupt service routine */
//The interrupt handler handles the interrupt to the MCU
//when the AD5940 INTC pin generates an interrupt to alert the MCU that data is ready
void Ext_Int0_Handler()
{
    ucInterrupted = 1;
    /* This example just set the flag and deal with interrupt in AD5940Main function. It's your choice to choose how to process interrupt. */
}

// -----------------------------------------------------------------------------

void AD5940_Delay10us(uint32_t time)
{
    delayMicroseconds(time * 10);  // 10 mikrosecundumos késleltetés
}

void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer, unsigned char *pRecvBuff, unsigned long length)
{
    SPI.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));

    for (int i = 0; i < length; i++)
    {
        *pRecvBuff++ = SPI.transfer(*pSendBuffer++);  // Transferálás byte-onként
    }

    SPI.endTransaction();  // Tranzakció vége
}

void AD5940_CsClr(void)
{
    digitalWrite(SPI_CS_AD5940_Pin, LOW);
}

void AD5940_CsSet(void)
{
    digitalWrite(SPI_CS_AD5940_Pin, HIGH);
}

void AD5940_RstSet(void)
{
    digitalWrite(AD5940_ResetPin, HIGH);
}

void AD5940_RstClr(void)
{
    digitalWrite(AD5940_ResetPin, LOW);
}

uint32_t AD5940_MCUResourceInit(void *pCfg)
{
  /* Step1, initialize SPI peripheral and its GPIOs for CS/RST */
  //start the SPI library (setup SCK, MOSI, and MISO pins)
    SPI.begin();
    //initalize SPI chip select pin
    pinMode(SPI_CS_AD5940_Pin, OUTPUT);
    //initalize Reset pin
    pinMode(AD5940_ResetPin, OUTPUT);
    
    /* Step2: initialize GPIO interrupt that connects to AD5940's interrupt output pin(Gp0, Gp3, Gp4, Gp6 or Gp7 ) */
    //init AD5940 interrupt pin
    pinMode(AD5940_IntPin, INPUT_PULLUP);
    //attach ISR for falling edge
    attachInterrupt(digitalPinToInterrupt(AD5940_IntPin), Ext_Int0_Handler, FALLING);

    AD5940_CsSet();  // CS alapértelmezetten HIGH
    AD5940_RstSet(); // Reset alapértelmezetten HIGH
    return 0;
}
