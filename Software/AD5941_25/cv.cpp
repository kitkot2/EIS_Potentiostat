//D. Bill (adapted from Analog Devices example code)
#include "Arduino.h"
#include "debug.h"

#include "ad5940.h"
#include "RampTest.h"

#include <LibPrintf.h> //allows to use c-style print() statement instead of Arduino style Serial.print()
#define PLOT_DATA /* use this macro to output data via serial port for python plotting */
#define APPBUFF_SIZE 1024

extern void ChangePixelsColor(int16_t& red, int16_t redInc, int16_t& green, int16_t greenInc, int16_t& blue, int16_t blueInc);

// value of RCAL on the board in Ohm
extern float fRcal;

static uint32_t _AppBuff[APPBUFF_SIZE]; //buffer to fetch AD5940 samples
static float _LFOSCFreq;

float V_start = -200.0;
float V_stop = 600.0;
float Estep = 2.0;          /**< The potential difference between each step in mV --> determines StepNumber. Ideally a multiple of DAC12BITVOLT_1LSB*/
float ScanRate = 100.0;     /**< Slope of the ramp in mV/sec --> determines RampDuration*/
uint16_t CycleNumber = 2;   /**< The number of cycles to repeat the ramp test */

// -----------------------------------------------------------------------------
/* Measured LFOSC frequency */
static int32_t RampShowResult(float* pData, uint32_t DataCount)
{
    /* Print data*/
    for (unsigned i = 0; i < DataCount; i += 2)
    {
        //pack voltage and current in a byte array
        //write the bytes to serial port
        Serial.print(pData[i]);
        Serial.print(",");
        Serial.print(pData[i + 1]);
        Serial.println(",");
    }
    return 0;
}

// -----------------------------------------------------------------------------
static int32_t AD5940PlatformCfg(void)
{
    //Serial.println (__func__);

    CLKCfg_Type clk_cfg;
    SEQCfg_Type seq_cfg;
    FIFOCfg_Type fifo_cfg;
    AGPIOCfg_Type gpio_cfg;
    LFOSCMeasure_Type LfoscMeasure;

    /* Use hardware reset */
    AD5940_HWReset();
    AD5940_Initialize();
    /* Call this right after AFE reset */
    /* Platform configuration */
    /* Step1. Configure clock */
    clk_cfg.HFOSCEn = bTRUE;
    clk_cfg.HFXTALEn = bFALSE;
    clk_cfg.LFOSCEn = bTRUE;
    clk_cfg.HfOSC32MHzMode = bFALSE;
    clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
    clk_cfg.SysClkDiv = SYSCLKDIV_1;
    clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
    clk_cfg.ADCClkDiv = ADCCLKDIV_1;
    AD5940_CLKCfg(&clk_cfg);

    // Serial.println (" platform 1");

     /* Step2. Configure FIFO and Sequencer*/
     /* Configure FIFO and Sequencer */
    fifo_cfg.FIFOEn = bTRUE;
    /* We will enable FIFO after all parameters configured */
    fifo_cfg.FIFOMode = FIFOMODE_FIFO;
    fifo_cfg.FIFOSize = FIFOSIZE_2KB;
    /* 2kB for FIFO, The reset 4kB for sequencer */
    fifo_cfg.FIFOSrc = FIFOSRC_SINC3;
    /* */
    fifo_cfg.FIFOThresh = 4;
    /*  Don't care, set it by application paramter */
    AD5940_FIFOCfg(&fifo_cfg);
    seq_cfg.SeqMemSize = SEQMEMSIZE_4KB;
    /* 4kB SRAM is used for sequencer, others for data FIFO */
    seq_cfg.SeqBreakEn = bFALSE;
    seq_cfg.SeqIgnoreEn = bTRUE;
    seq_cfg.SeqCntCRCClr = bTRUE;
    seq_cfg.SeqEnable = bFALSE;
    seq_cfg.SeqWrTimer = 0;
    AD5940_SEQCfg(&seq_cfg);

    //Serial.println (" platform 2");

    /* Step3. Interrupt controller */
    AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);
    /* Enable all interrupt in INTC1, so we can check INTC flags */
    AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
    AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH | AFEINTSRC_ENDSEQ | AFEINTSRC_CUSTOMINT0 | AFEINTSRC_CUSTOMINT1 | AFEINTSRC_GPT1INT_TRYBRK | AFEINTSRC_DATAFIFOOF, bTRUE);
    AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

    //Serial.println (" platform 3");

    /* Step4: Configure GPIO */
    gpio_cfg.FuncSet = GP0_INT | GP1_GPIO | GP2_SYNC;
    /* GPIO1 indicates AFE is in sleep state. GPIO2 indicates ADC is sampling. */
    gpio_cfg.InputEnSet = 0;
    gpio_cfg.OutputEnSet = AGPIO_Pin0 | AGPIO_Pin1 | AGPIO_Pin2;
    gpio_cfg.OutVal = AGPIO_Pin1; //set high to turn off LED
    gpio_cfg.PullEnSet = 0;
    AD5940_AGPIOCfg(&gpio_cfg);
    /* Measure LFOSC frequency */
    /**@note Calibrate LFOSC using system clock. The system clock accuracy decides measurement accuracy. Use XTAL to get better result. */
    LfoscMeasure.CalDuration = 1000.0;
    /* 1000ms used for calibration. */
    LfoscMeasure.CalSeqAddr = 0;
    /* Put sequence commands from start address of SRAM */
    LfoscMeasure.SystemClkFreq = 16000000.0f;
    /* 16MHz in this firmware. */
    AD5940_LFOSCMeasure(&LfoscMeasure, &_LFOSCFreq);
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
    /*  */
    return 0;
}

// -----------------------------------------------------------------------------
static void _AD5940RampStructInit(void)
{
    AppRAMPCfg_Type* pRampCfg;
    AppRAMPGetCfg(&pRampCfg);
    /* Step1: configure general parameters */
    pRampCfg->SeqStartAddr = 0x10;
    /* leave 16 commands for LFOSC calibration.  */
    pRampCfg->MaxSeqLen = 1024 - 0x10;
    /* 4kB/4 = 1024  */
    pRampCfg->RcalVal = fRcal;
    /* 10kOhm RCAL */
    pRampCfg->ADCRefVolt = 1820.0f;
    /* The real ADC reference voltage. Measure it from capacitor C3 (AD5941 FeatherWing Rev1 Board) with DMM. */  pRampCfg->FifoThresh = 100;
    /* Maximum value is 2kB/4-1 = 512-1. Set it to higher value to save power. */
    pRampCfg->SysClkFreq = 16000000.0f;
    /* System clock is 16MHz by default */
    pRampCfg->LFOSCClkFreq = _LFOSCFreq;
    /* LFOSC frequency */
    /* Configure ramp signal parameters */
    //pRampCfg->RampStartVolt =V_start;  // itt -300.0f volt
    /* in mV*/
   // pRampCfg->RampPeakVolt1 =V_start;  // itt -300.0f volt
    /* If FIX_WE_POT defined, make sure (|RampPeakVolt1 - RampPeakVolt2| + 35mV) <= (VzeroHighLevel-VzeroLowLevel) */
    //pRampCfg->RampPeakVolt2 =V_stop;    // itt 500.0f volt
    pRampCfg->VzeroLimitHigh = 2400;
    /* 2.2V */
    pRampCfg->VzeroLimitLow = 200;
    /* 0.4V */
    pRampCfg->Estep = Estep;
    pRampCfg->ScanRate = ScanRate;
    pRampCfg->CycleNumber = CycleNumber;
    pRampCfg->StepNumber = 800;
    pRampCfg->RampDuration = 24 * 1000;
    pRampCfg->ADCSinc3Osr = ADCSINC3OSR_4;
    pRampCfg->ADCSinc2Osr = ADCSINC2OSR_667;
    pRampCfg->SampleDelay = 7.0f;
    /* 7ms. Time between update DAC and ADC sample. Unit is ms. SampleDelay > 1.0ms is acceptable.*/
    pRampCfg->LpAmpPwrMod = LPAMPPWR_NORM;
    /* restrict to max. +/- 750 uA cell current*/
    pRampCfg->LPTIARtiaSel = LPTIARTIA_1K;
    /* Maximum current decides RTIA value: Imax = 0.9V / RTIA */
    pRampCfg->LPTIARloadSel = LPTIARLOAD_SHORT;
    pRampCfg->AdcPgaGain = ADCPGA_1P5;
    pRampCfg->bRampOneDir = bFALSE; //activate LSV instead of CV
}

// -----------------------------------------------------------------------------
static void _AD5940_Main(void)
{
    ENTER("cv _AD5940_Main");

    uint32_t temp;
    AppRAMPCfg_Type *pRampCfg;
    AppRAMPInit(_AppBuff, APPBUFF_SIZE);
    /* Initialize RAMP application. Provide a buffer, which is used to store sequencer commands */
    //send ramp data to python application
#ifdef PLOT_DATA
    AppRAMPGetCfg(&pRampCfg);
    int Sinc3OSR[] = {2, 4, 5};
    int Sinc2OSR[] = {22, 44, 89, 178, 267, 533, 640, 667, 800, 889, 1067, 1333};
    //pack voltage and current in a byte array
    uint8_t byteData[30];
    byteData[0] = ((int16_t)pRampCfg->RampStartVolt & 0xFF00) >> 8;
    byteData[1] = (int16_t)pRampCfg->RampStartVolt & 0x00FF;
    byteData[2] = ((int16_t)pRampCfg->RampPeakVolt1 & 0xFF00) >> 8;
    byteData[3] = (int16_t)pRampCfg->RampPeakVolt1 & 0x00FF;
    byteData[4] = ((int16_t)pRampCfg->RampPeakVolt2 & 0xFF00) >> 8;
    byteData[5] = (int16_t)pRampCfg->RampPeakVolt2 & 0x00FF;
    byteData[6] = ((int16_t)pRampCfg->Estep & 0xFF00) >> 8;
    byteData[7] = (int16_t)pRampCfg->Estep & 0x00FF;
    byteData[8] = ((int16_t)pRampCfg->ScanRate & 0xFF00) >> 8;
    byteData[9] = (int16_t)pRampCfg->ScanRate & 0x00FF;
    byteData[10] = ((int16_t)pRampCfg->CycleNumber & 0xFF00) >> 8;
    byteData[11] = (int16_t)pRampCfg->CycleNumber & 0x00FF;
    byteData[12] = ((int16_t)Sinc3OSR[pRampCfg->ADCSinc3Osr] & 0xFF00) >> 8;
    byteData[13] = (int16_t)Sinc3OSR[pRampCfg->ADCSinc3Osr] & 0x00FF;
    byteData[14] = ((int16_t)Sinc2OSR[pRampCfg->ADCSinc2Osr] & 0xFF00) >> 8;
    byteData[15] = (int16_t)Sinc2OSR[pRampCfg->ADCSinc2Osr] & 0x00FF;
    byteData[16] = ((int16_t)pRampCfg->StepNumber & 0xFF00) >> 8;
    byteData[17] = (int16_t)pRampCfg->StepNumber & 0x00FF;
    //write the bytes to serial port
    //Serial.write(&byteData[0], 18); // fog így működni?? igen!!!!
    //delay to allow python to set everything up
    delay(3000);
#endif
    AppRAMPCtrl(APPCTRL_START, 0);
	BlinkLed();
    
    int16_t red = 0;
    int16_t green = 0;
    int16_t blue = 0;

    /* Control RAMP measurement to start. Second parameter has no meaning with this command. */
    while (!pRampCfg->bTestFinished)
    {
        AppRAMPGetCfg(&pRampCfg);
        if (AD5940_GetMCUIntFlag())
        {
            AD5940_ClrMCUIntFlag();
            temp = APPBUFF_SIZE;
            AppRAMPISR(_AppBuff, &temp); //temp now holds the number of calculated means (>0, if at least one step was finished within the current data set from FIFO)
            //print data in case at least one step was finished
            if (temp > 0)
            {
                RampShowResult((float *)_AppBuff, temp);
            }
	
			BlinkLed();
            ChangePixelsColor(red, 4, green, 0, blue, -4);
#if DEBUG
			Trace::FlushLogBuffer();
#endif
        }
	
#if MEASURE
		Measure();
#endif
    }
    //end of test
    //after test finished, reset flag to be able to start new test
       
    
    pRampCfg->bTestFinished = bFALSE;
#if DEBUG
	Trace::FlushLogBuffer();
#endif
    LEAVE;
}

// -----------------------------------------------------------------------------
void cvSetup(float vstart, float vstop)
{
    AppRAMPCfg_Type *pRampCfg;
    AppRAMPGetCfg (&pRampCfg);
    //Serial.print ("RampStartVolt ");
    //Serial.println (pRampCfg->RampStartVolt);
    //Serial.print ("RampPeakVolt2 ");
    //Serial.println (pRampCfg->RampPeakVolt2);
    //init GPIOs (SPI, AD5940 Reset and Interrupt)
    AD5940_MCUResourceInit(0);
    //configure AFE
    AD5940PlatformCfg();
    //init application with pre-defined parameters
     _AD5940RampStructInit();

	pRampCfg->RampStartVolt = vstart;
	pRampCfg->RampPeakVolt1 = pRampCfg->RampStartVolt;
	pRampCfg->RampPeakVolt2 = vstop;

    //run voltammetry once
     _AD5940_Main();
}
