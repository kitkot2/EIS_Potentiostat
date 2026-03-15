/*
    AUTHOR: Richard Morrison
    EMAIL: instruments4chem@gmail.com

    DISCLAIMER:
    Instruments4Chem code, firmware, and software is released under the MIT License
    (http://opensource.org/licenses/MIT).

    The MIT License (MIT)

    Copyright (c) 2024 Instruments4Chem, Melbourne, AUSTRALIA

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights to
    use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
    of the Software, and to permit persons to whom the Software is furnished to do
    so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/
// D:\Work\VamosPista\Richard\AD5940_RM_EIS161224\AD5940_RM_EIS161224.ino

#define USE_250202_Version  1
#define SET_VOLTAGE_ON_RP2040_D6  0
// Code to run EIS experiments using an Analog Devices AD5941 chip
// Richard J. S. Morrison, December 10th, 2024

#include "ad5940.h"
#include "AD5940.h"
#include <stdio.h>
#include "string.h"
#include <Adafruit_NeoPixel.h>
#include "utilities.h"

extern uint32_t AD5940_TakeMeasurement(uint32_t afectrl_bits, uint32_t afectrl_readybit, uint32_t afectrl_resultType, int32_t *time_out);

// V start/stop for ramptest (cv.cpp)
extern float V_start;
extern float V_stop;
extern float Estep;             /**< The potential difference between each step in mV --> determines StepNumber. Ideally a multiple of DAC12BITVOLT_1LSB*/
extern float ScanRate;          /**< Slope of the ramp in mV/sec --> determines RampDuration*/
extern uint16_t CycleNumber;    /**< The number of cycles to repeat the ramp test */
extern void  cvSetup (float start, float stop);

int Power = 11;
int PIN  = 12;
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Define the ADC input pins for WE and RE
#define WEpin A0            // ADC input pin for WE
#define REpin A3            // ADC input pin for RE

const float ADC_AVDD = 3300.0;              // Power supply for analogue-to-digital converter, nominal voltage 3.3V
const int ADCresolution = 12;               // RP2040 ADC has 12 bit resolution
const int ADCFullValue = (1 << ADCresolution) - 1;
float WEmV = 0.0;                           // expected voltage on WE, set by the host in OCPCalibration mode
float WEfrom = 0.0;                         // OCP calibration start voltage on WE, set by the host
float WEto = 0.0;                           // OCP calibration stop voltage on WE, set by the host
float WEstep = 0.0;                         // OCP calibration voltage step on WE, set by the host
uint32_t HSDACDAT;                          // used in OCP calculation
BoolFlag ocpCalibration = bFALSE;           // calibration of OCP calculation is in progress
BoolFlag ocpCalibrationCycling = bFALSE;    // a series calibration of OCP calculations are to be performed from WEfrom to WEto with step of WEstep
float constA = 32772.0;                     // used in OCP calculation
float constB = -26.719;                     // used in OCP calculation
bool useConstAB = false;                    // tells if constA & B is to be used when calculating OCP

#define ADCPGA_GAIN_SEL ADCPGA_1P5

// See the E command for a description of the following two parameters
// They provide settling delays and were added during testing
// Current setup is two fixed delays of 2 ms and 1 ms - but changing them
// didn't seem to make any difference however they have been retained for now

#define settling_parameter 2
#define adc_delay_ms  1

#define SYS_CLOCK_HZ    16000000.0

extern int csPin;
extern int resetPin;
extern int intPin;

int receivedNumber = 0;
char commandType = 0;
bool hex = false;

FreqParams_Type FP;

uint8_t pga_gain = 1, tia_rf = 3, EIS_mode, vzero = 26, use_variable_gain = 0;   // EIS_mode: 0 - Rz, 1 - Rcal
uint16_t nfreqs = 50, vbias = 1664, amplitude = 126, offset = 4040, OCP_npts ;
uint16_t    settling_delay_ms ;                                                  //  MS: i is deleted (it is very dangerous to declare cycle variable in global scope!)
uint32_t freqlo, freqhi, _CGmax = 30000, _CGmin = 7500, OCP_sum, ADCCON;

uint32_t ADCFILTERCON, DFTCON, AFECON;

AFERefCfg_Type aferef_cfg;

// value of RCAL on the board in Ohm
float fRcal = 10000.0;
float fAmplitude = 50.0,
    fBias = 0.0,
    fOffset = 0.0;

HSLoopCfg_Type HpLoopCfg;
ADCFilterCfg_Type adc_filter;
ADCBaseCfg_Type adc_base;
DFTCfg_Type DftCfg;
FIFOCfg_Type fifo_cfg;
CLKCfg_Type clk_cfg;
ClksCalInfo_Type clks_cal;

// Set proper bit in verbose to show and log changes in internal states of variables
// 1 - activates Info to Serial
// 2.. - activates info and log channels
extern uint32_t verbose;

// bool SeeedStatMode is status flag. By default it is false to work with LabVIEW program (EIS_161224.exe). When using with SeeedStat, 'D' command (setting frequency range)
// sent by SeeedStat sets bool SeeedStatMode to true, and in this case real and imaginary values are sent in ASCII when executing measurement (P command).
// After measurement, bool SeeedStatMode is set back to false.
bool SeeedStatMode = false;

extern char history[];
extern char* pHistory;

// array to store measured values
// 4: real and imag for measured values of unknown and rcal resp.
// 7: max. number of decades: 0.1, 1, 10, 100, 1000, 10000, 100000
// 20: max. number of frequencies per decade
float Measurements[4 * 7 * 20];
float* pMeasurement;
int numberOfMeasurements;

#define SYS_CLOCK_HZ 16000000.0 // System clock frequency

#if SET_VOLTAGE_ON_RP2040_D6
void RP2040_SetVoltageOnPin(int pin, float mV, bool withDelay)
{
    if (mV <= 1.0)
    {
        pinMode(pin, INPUT);
    }
    else
    {
        // Set the PWM pin as outputs
        pinMode(pin, OUTPUT);
        
        // input voltage on AD5940 shall be between 0.2 and 2.1 V
//*        mV = constrain(mV, 200.0, 2100.0);
        
        const int resolution = 16;                          // resolution of PWM DAC in RP2040
        analogWriteResolution(resolution);
        int value = ((1 << resolution) - 1) * mV / 3300.0;  // supply voltage is 3.3V
        analogWrite(pin, value);                            // Set duty cycle
    }
    
    if (withDelay)
    {
        delay(3000);
    }
}
#endif

uint32_t SetDACLevel(float mV)
{
    uint32_t HSDACCON = AD5940_ReadReg(REG_AFE_HSDACCON);
    float INAMPGNMDE = (HSDACCON & BITM_AFE_HSDACCON_INAMPGNMDE) == BITM_AFE_HSDACCON_INAMPGNMDE ? 0.25 : 2.0;
    float ATTENEN = (HSDACCON & BITM_AFE_HSDACCON_ATTENEN) == BITM_AFE_HSDACCON_ATTENEN ? 0.2 : 1.0;
    // HSDACDAT = mV * 2048.0 / (404.4 * INAMPGNMDE * ATTENEN) + 2048.0;  // formula containing 404.4 mV for direct write to HSDACDAT, see page 43 of AD5940/AD5941.pdf
    HSDACDAT = mV * 2047.0 / (808.0 * INAMPGNMDE * ATTENEN);     // formula containing 808.8 mV for use with WG, see page 43 of AD5940/AD5941.pdf
    //HSDACDAT = constrain(HSDACDAT, 0x200, 0xe00);

    WGCfg_Type wgInit;
    wgInit.WgType = WGTYPE_MMR;         // Direct write to DAC using register
    wgInit.GainCalEn = bTRUE;           // Enable Gain calibration
    wgInit.OffsetCalEn = bFALSE;        // Disable offset calibration
    wgInit.WgCode = HSDACDAT;
    AD5940_WGCfgS(&wgInit);

    Log(0x80, __LINE__, "mV=%.2f HSDACCON=0x%lX INAMPGNMDE=%.2f ATTENEN=%.1f HSDACDAT=%ld(0x%lX) ",
                         mV,     HSDACCON,      INAMPGNMDE,     ATTENEN,     HSDACDAT, constrain(HSDACDAT, 0x200, 0xe00));
                         
#if SET_VOLTAGE_ON_RP2040_D6
    RP2040_SetVoltageOnPin(D6, mV, false);
#endif

    return HSDACDAT;
}

// This routine should be invoked in Setup and at the end of a LabVIEW run to leave the system in a
// well-defined default state.  It should also be called if ADC/DFT/Wavegen parameters may have changed.

void AD5941_InitAll()
{
    AD5940_HWReset();
    AD5940_MCUResourceInit(0);
    AD5940_Initialize();
}

// This routine finds optimum values of TIA_Rf and PGA_gain given an input frequency
// The algorithm makes use of combined gain(CG) values determined by multiplying the gain resistor
// value in ohms by the PGA gain factor.  The user specifies maximum and minimum values for CG -
// the former assumed to apply at 0.1 Hz and the latter at 100 kHz.  From these values the Slope(m)
// and intercept(c) based on the relation log(CG) = m * log(frequency) + c are then used to calculate
// a log(CG) value and then an optimum CG for the given input frequency.
//
// The code then scans through all possible TIA_Rf and PGA gain settings to find the one that is closest to
// this optimum CG value.  The two values passed back are the AD5941 codes needed to correctly set
// these two parameters.  The closest combined gain found is also returned.
//
// for example - if we wish to set Rf = 40k and PGA = 9 at 0.1 Hz - CGmax becomes 360000
// and we then require to set Rf = 200 and PGA = 1 at 100 kHz - CGmin becomes 200

void FindOptimum_Rf_PGA(uint32_t CGmax, uint32_t CGmin, float freq, uint8_t* TIA_Rf, uint8_t* PGA_gain, float* closest_CG)
{
    uint32_t rf_values[] = {200, 1000, 5000, 10000, 20000, 40000, 80000, 160000};
    float pga_values[] = {1.0, 1.5, 2, 4, 9};

    float logmax, logmin, logf, logCG, m, c, CG, target_CG, best_error, output_CG, error;

    logf = log10(freq);

    logmax = log10(CGmax);
    logmin = log10(CGmin);

    m = (logmin - logmax) / 6.0;
    c = (5 * logmax + logmin) / 6.0;

    logCG = m * logf + c;
    CG = pow(10, logCG);

    target_CG = CG;
    best_error = 1000000.0;
    *closest_CG = 0.0;

    // Try all combinations of rf_values and pga_values
    for (uint8_t i = 0; i < 8; i++)
    {
        for (uint8_t j = 0; j < 5; j++)
        {
           output_CG = rf_values[i] * pga_values[j];
           error = abs(target_CG - output_CG);
           if (error < best_error)
           {
               best_error = error;

               *TIA_Rf = i;
               *PGA_gain = j;

               *closest_CG = output_CG;
            }
        }
    }
}

static void AD5940_PGA_Calibration(void)
{
    AD5940Err err;
    ADCPGACal_Type pgacal;
    pgacal.AdcClkFreq = 16e6;
    pgacal.ADCSinc2Osr = ADCSINC2OSR_1333;  //*FP.ADCSinc2Osr;
    pgacal.ADCSinc3Osr = ADCSINC3OSR_4;     //*FP.ADCSinc3Osr;
    pgacal.SysClkFreq = 16e6;
    pgacal.TimeOut10us = 1000;
    pgacal.VRef1p11 = 1.11f;
    pgacal.VRef1p82 = 1.82f;
    pgacal.PGACalType = PGACALTYPE_OFFSETGAIN;
    //pgacal.ADCPga = ADCPGA_GAIN_SEL;
    pgacal.ADCPga = pga_gain;
    err = AD5940_ADCPGACal(&pgacal);
    if(err != AD5940ERR_OK)
    {
        Serial.println("AD5940 PGA calibration failed.");
    }
}

void Do_WaveGen(uint8_t mode, float frequency, uint16_t amplitude, uint8_t tia_code)
{
    if (SeeedStatMode)
    {
        digitalWrite(D4, HIGH);
    }

    uint8_t _TIA_Rf, _PGA_gain;
    float closest_CG;

    clk_cfg.ADCClkDiv = ADCCLKDIV_1;
    clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
    clk_cfg.SysClkDiv = SYSCLKDIV_1;
    clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
    clk_cfg.HfOSC32MHzMode = bFALSE;
    clk_cfg.HFOSCEn = bTRUE;
    clk_cfg.HFXTALEn = bFALSE;
    clk_cfg.LFOSCEn = bTRUE;
    AD5940_CLKCfg(&clk_cfg);

    aferef_cfg.HpBandgapEn = bTRUE;
    aferef_cfg.Hp1V1BuffEn = bTRUE;
    aferef_cfg.Hp1V8BuffEn = bTRUE;
    aferef_cfg.Disc1V1Cap = bFALSE;
    aferef_cfg.Disc1V8Cap = bFALSE;
    aferef_cfg.Hp1V8ThemBuff = bFALSE;
    aferef_cfg.Hp1V8Ilimit = bFALSE;
    aferef_cfg.Lp1V1BuffEn = bFALSE;
    aferef_cfg.Lp1V8BuffEn = bFALSE;

    // LP reference control
    aferef_cfg.LpBandgapEn = bTRUE;
    aferef_cfg.LpRefBufEn = bTRUE;
    aferef_cfg.LpRefBoostEn = bFALSE;
    AD5940_REFCfgS(&aferef_cfg);

    // lines marked with //- are modifications based on Richard's version 2025.01.26
    //-HpLoopCfg.HsDacCfg.ExcitBufGain = EXCITBUFGAIN_2;
    //-HpLoopCfg.HsDacCfg.HsDacGain = HSDACGAIN_1;
    //-HpLoopCfg.HsDacCfg.HsDacUpdateRate = 0x1B;

    HpLoopCfg.HsTiaCfg.DiodeClose = bFALSE;
    HpLoopCfg.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
    HpLoopCfg.HsTiaCfg.HstiaCtia = 16; /* 16pF */
    HpLoopCfg.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
    HpLoopCfg.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_TODE;

    // here we set the all important TIA_Rf and PGA gain values !!!
    // they can either be constant (from LabVIEW input) or variable
    // see

    if(use_variable_gain)
    {
        FindOptimum_Rf_PGA(_CGmax, _CGmin, frequency, &_TIA_Rf, &_PGA_gain, &closest_CG);
        HpLoopCfg.HsTiaCfg.HstiaRtiaSel = _TIA_Rf;
        adc_base.ADCPga = _PGA_gain;
    }
    else
    {
        HpLoopCfg.HsTiaCfg.HstiaRtiaSel = tia_code;
        adc_base.ADCPga = pga_gain;
    }

    AD5940_ADCBaseCfgS(&adc_base); // this actually sets the PGA gain

    if(mode == 0)
    {
        HpLoopCfg.SWMatCfg.Dswitch = SWD_CE0;
        HpLoopCfg.SWMatCfg.Pswitch = SWP_RE0;
        HpLoopCfg.SWMatCfg.Nswitch = SWN_SE0;
        HpLoopCfg.SWMatCfg.Tswitch = SWT_TRTIA | SWT_SE0LOAD;
    }
    else
    {
        HpLoopCfg.SWMatCfg.Dswitch = SWD_RCAL0;             // DR0 closed
        HpLoopCfg.SWMatCfg.Pswitch = SWP_RCAL0;             // PR0 closed
        HpLoopCfg.SWMatCfg.Nswitch = SWN_RCAL1;             // NR1 closed
        HpLoopCfg.SWMatCfg.Tswitch = SWT_TRTIA | SWT_RCAL1; // T9 and TR1 closed
    }

    AD5940_AFECtrlS(AFECTRL_WG, bFALSE);  // before making changes to wavegen parameters

    HpLoopCfg.WgCfg.WgType = WGTYPE_SIN;
    HpLoopCfg.WgCfg.GainCalEn = bFALSE;
    HpLoopCfg.WgCfg.OffsetCalEn = bFALSE;
    HpLoopCfg.WgCfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(frequency, SYS_CLOCK_HZ);
    HpLoopCfg.WgCfg.SinCfg.SinAmplitudeWord = amplitude;
    HpLoopCfg.WgCfg.SinCfg.SinOffsetWord = offset;
    HpLoopCfg.WgCfg.SinCfg.SinPhaseWord = 0;
    AD5940_HSLoopCfgS(&HpLoopCfg);

    AD5940_AFECtrlS(AFECTRL_DACREFPWR, bTRUE);
    AD5940_AFECtrlS(AFECTRL_EXTBUFPWR | AFECTRL_INAMPPWR | AFECTRL_HSTIAPWR | AFECTRL_HSDACPWR, bTRUE);
    AD5940_AFECtrlS(AFECTRL_WG, bTRUE);
    AD5940_AFECtrlS(AFECTRL_DCBUFPWR, bTRUE);

    AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);

    if (SeeedStatMode)
    {
        digitalWrite(D4, LOW);
    }
}

// Important note - January 25th, 2025
// This routine is now used instead of calling the AD5940_GetFreqParameters routine
// which did not behave nicely when bias voltage was applied !!!
//
// Configures ADC filter and DFT parameters
// Values here can be customized according to various frequency intervals
//
// If doing so the key requirements are -:
// ADC Sample rate must satisfy the Nyquist theorem (ideally fs >> 2*fexc)
// DFT engine must sample for at least one full period but preferable many

// For information here are the possible SINC2OSR and SINC3OSR constants -:
// SINC2OSR allowed values are 22,44,89,178,267,533,640,667,800,889,1067 and 1333
// SINC3OSR allowed values are 2, 4 and 5

void init_AD5940_ADC(float freq)
{
    fifo_cfg.FIFOEn = bFALSE;
    fifo_cfg.FIFOMode = FIFOMODE_FIFO;
    fifo_cfg.FIFOSize = FIFOSIZE_4KB;
    fifo_cfg.FIFOSrc = FIFOSRC_DFT;
    fifo_cfg.FIFOThresh = 2;
    AD5940_FIFOCfg(&fifo_cfg);

    fifo_cfg.FIFOEn = bTRUE;
    AD5940_FIFOCfg(&fifo_cfg);

    AD5940_ADCMuxCfgS(ADCMUXP_HSTIA_P, ADCMUXN_HSTIA_N);   // Configure ADC MUX

    AD5940_StructInit(&adc_filter, sizeof(adc_filter));
    AD5940_StructInit(&DftCfg, sizeof(DftCfg));

    // SINC output = 800000/(1067*4) = 187 SPS, DFT takes 16384/187 = 87s
    // Period of lowest frequency (0.02 Hz) in interval is 50 s

    if (freq < .11)
    {
        adc_filter.ADCAvgNum = ADCAVGNUM_16;  // Parameter not relevant
        adc_filter.ADCSinc2Osr = ADCSINC2OSR_1067;
        adc_filter.ADCSinc3Osr = ADCSINC3OSR_4;
        adc_filter.BpNotch = bTRUE;
        adc_filter.BpSinc3 = bFALSE;
        adc_filter.Sinc2NotchEnable = bTRUE;
        adc_filter.ADCRate = ADCRATE_800KHZ;

        DftCfg.DftNum = DFTNUM_16384;
        DftCfg.DftSrc = DFTSRC_SINC2NOTCH;
        DftCfg.HanWinEn = bTRUE;
    }

    // SINC output = 800000/(267*5) = 599 SPS, DFT takes 8192/599 = 13.7s
    // Period of lowest frequency (0.12 Hz) in interval is 8.3 s; giving nperiods = 1.65

    else if (freq < .51) // 0.11 < frequency < 0.51
    {
        adc_filter.ADCAvgNum = ADCAVGNUM_16;
        adc_filter.ADCSinc2Osr = ADCSINC2OSR_267;
        adc_filter.ADCSinc3Osr = ADCSINC3OSR_5;
        adc_filter.BpNotch = bTRUE;
        adc_filter.BpSinc3 = bFALSE;
        adc_filter.Sinc2NotchEnable = bTRUE;
        adc_filter.ADCRate = ADCRATE_800KHZ;

        DftCfg.DftNum = DFTNUM_8192;
        DftCfg.DftSrc = DFTSRC_SINC2NOTCH;
        DftCfg.HanWinEn = bTRUE;
    }

    // SINC output = 800000/(89*4) = 2247 SPS, DFT takes 8192/2247 = 3.6s
    // Period of lowest frequency (0.52 Hz) in interval is 1.9 s; giving nperiods = 1.9

    else if(freq < 5) // 0.51 < frequency < 5
    {
        adc_filter.ADCAvgNum = ADCAVGNUM_16;
        adc_filter.ADCSinc2Osr = ADCSINC2OSR_178; // was 89
        adc_filter.ADCSinc3Osr = ADCSINC3OSR_4;
        adc_filter.BpNotch = bTRUE;
        adc_filter.BpSinc3 = bFALSE;
        adc_filter.Sinc2NotchEnable = bTRUE;
        adc_filter.ADCRate = ADCRATE_800KHZ;

        DftCfg.DftNum = DFTNUM_8192;
        DftCfg.DftSrc = DFTSRC_SINC2NOTCH;
        DftCfg.HanWinEn = bTRUE;
    }

    // SINC output = 800000/(44*4) = 4545 SPS, DFT takes 4096/4545 = 900 ms
    // Period of lowest frequency (5 Hz) in interval is 200 ms; giving nperiods = 4.5

    else if(freq < 450)  // 5 < frequency < 450
    {
        adc_filter.ADCAvgNum = ADCAVGNUM_16;
        adc_filter.ADCSinc2Osr = ADCSINC2OSR_44;  // was 178
        adc_filter.ADCSinc3Osr = ADCSINC3OSR_4;
        adc_filter.BpNotch = bTRUE;
        adc_filter.BpSinc3 = bFALSE;
        adc_filter.Sinc2NotchEnable = bTRUE;
        adc_filter.ADCRate = ADCRATE_800KHZ;

        DftCfg.DftNum = DFTNUM_4096;
        DftCfg.DftSrc = DFTSRC_SINC2NOTCH;
        DftCfg.HanWinEn = bTRUE;
    }

    // SINC output = 800000/(4) = 200k SPS, DFT takes 16384/200000 = 82 ms
    // Period of lowest frequency (450 Hz) in interval is 2.2 ms; giving nperiods = 37.3

    else if(freq < 80000)  // 450 < frequency < 80000
    {
        adc_filter.ADCAvgNum = ADCAVGNUM_16;
        adc_filter.ADCSinc2Osr = ADCSINC2OSR_178;
        adc_filter.ADCSinc3Osr = ADCSINC3OSR_4;
        adc_filter.BpNotch = bTRUE;
        adc_filter.BpSinc3 = bFALSE;
        adc_filter.Sinc2NotchEnable = bFALSE;
        adc_filter.ADCRate = ADCRATE_800KHZ;

        DftCfg.DftNum = DFTNUM_16384;
        DftCfg.DftSrc = DFTSRC_SINC3;
        DftCfg.HanWinEn = bTRUE;
    }

    // SINC output = 1600000/(2) = 400k SPS, DFT takes 16384/800000 = 20.5 ms
    // Period of lowest frequency (80 kHz) in interval is 12.5 us; giving nperiods = 1640

    else // 80000 < frequency < 200000
    {
        adc_filter.ADCAvgNum = ADCAVGNUM_16;
        adc_filter.ADCSinc2Osr = ADCSINC2OSR_178;
        adc_filter.ADCSinc3Osr = ADCSINC3OSR_2;
        adc_filter.BpNotch = bTRUE;
        adc_filter.BpSinc3 = bFALSE;
        adc_filter.Sinc2NotchEnable = bFALSE;
        adc_filter.ADCRate = ADCRATE_1P6MHZ;

        DftCfg.DftNum = DFTNUM_16384;
        DftCfg.DftSrc = DFTSRC_SINC3;
        DftCfg.HanWinEn = bTRUE;
    }

    AD5940_ADCFilterCfgS(&adc_filter);
    AD5940_DFTCfgS(&DftCfg);
}

// Direct DAC output to CE pin
// this functionality is taken from ad5940-examples-master\examples\AD5940_WG\AD5940_WGArbitrary.c
void ConfigureHSLoopCfg(uint32_t hsDacDat)
{
    HSLoopCfg_Type HpLoopCfg;
    AD5940_StructInit(&HpLoopCfg, sizeof(HSLoopCfg_Type));
    
    HpLoopCfg.HsDacCfg.ExcitBufGain = EXCITBUFGAIN_2;
    HpLoopCfg.HsDacCfg.HsDacGain = HSDACGAIN_1;
    HpLoopCfg.HsDacCfg.HsDacUpdateRate = 7;

    HpLoopCfg.HsTiaCfg.DiodeClose = bFALSE;
    HpLoopCfg.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
    HpLoopCfg.HsTiaCfg.HstiaCtia = 16; /* 16pF */
    HpLoopCfg.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
    HpLoopCfg.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_TODE;    /* Connect HSTIA output to DE0 pin */
    HpLoopCfg.HsTiaCfg.HstiaRtiaSel = HSTIARTIA_200;

    HpLoopCfg.SWMatCfg.Dswitch = SWD_CE0;
    HpLoopCfg.SWMatCfg.Pswitch = SWP_CE0;
    HpLoopCfg.SWMatCfg.Nswitch = SWN_SE0LOAD;
    HpLoopCfg.SWMatCfg.Tswitch = SWT_TRTIA | SWT_SE0LOAD;

    HpLoopCfg.WgCfg.WgType = WGTYPE_MMR;    /* We use sequencer to update DAC data point by point. */
    HpLoopCfg.WgCfg.GainCalEn = bFALSE;
    HpLoopCfg.WgCfg.OffsetCalEn = bFALSE;
    HpLoopCfg.WgCfg.WgCode = hsDacDat;
    AD5940_HSLoopCfgS(&HpLoopCfg);
}

void Config_AD5941_OCP_Measurement(float wemV)
{
    AD5940_PGA_Calibration();
    AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);

    // Initialize ADC filters ADCRawData-->SINC3-->SINC2+NOTCH
    // In this configuration both SINC3 and SINC2 blocks are active
    // Data rate after the SINC2 filter will be 800kSPS/4/1333 = 150.0375Hz

    adc_base.ADCPga = ADCPGA_GAIN_SEL;
    AD5940_ADCBaseCfgS(&adc_base);

    adc_filter.ADCSinc3Osr = ADCSINC3OSR_4;
    adc_filter.ADCSinc2Osr = ADCSINC2OSR_1333;
    adc_filter.ADCAvgNum = ADCAVGNUM_2;         // irrelevant as only used by DFT
    adc_filter.ADCRate = ADCRATE_800KHZ;
    adc_filter.BpNotch = bTRUE;
    adc_filter.BpSinc3 = bFALSE;
    adc_filter.Sinc2NotchEnable = bTRUE;
    AD5940_ADCFilterCfgS(&adc_filter);

    AD5940_ADCMuxCfgS(ADCMUXP_VSE0, ADCMUXN_AIN3);          // RE is input on AIN3/BUF_VREF1V8 into ADC MUXSELN
    
    AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);
    AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
    ///AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_ADCCNV | AFECTRL_WG | AFECTRL_DACREFPWR | AFECTRL_EXTBUFPWR | AFECTRL_INAMPPWR | AFECTRL_DCBUFPWR | AFECTRL_HSDACPWR, bFALSE);
    
    OutputPulse(D4, 300);
    if (ocpCalibration == bTRUE)
    {
        digitalWrite(D4, HIGH);
        uint32_t hsDacDat = SetDACLevel(wemV);
        digitalWrite(D4, LOW);

        ConfigureHSLoopCfg(hsDacDat);
    }

    AD5940_AFECtrlS(AFECTRL_DACREFPWR, ocpCalibration);
    AD5940_AFECtrlS(AFECTRL_EXTBUFPWR | AFECTRL_INAMPPWR | AFECTRL_HSTIAPWR | AFECTRL_HSTIAPWR | AFECTRL_HSDACPWR, ocpCalibration);
    AD5940_AFECtrlS(AFECTRL_WG, ocpCalibration);
    AD5940_AFECtrlS(AFECTRL_DCBUFPWR, ocpCalibration);

    // n.b. There are no calls to configure the LP loop here
    // This is important as we don't want potentiostatic control
    // affecting the RE pin during the OCP measurements
}

uint32_t Do1_AD5941_OCP_Measurement()
{
    uint32_t result = 0;
    digitalWrite(D5, HIGH);
#if 0
    while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_SINC2RDY) == bFALSE);
    AD5940_INTCClrFlag(AFEINTSRC_SINC2RDY);
    result = AD5940_ReadAfeResult(AFERESULT_SINC2);
#else
    int32_t time_out = 1000;
    result = AD5940_TakeMeasurement(AFECTRL_ADCPWR | AFECTRL_ADCCNV, AFEINTSRC_SINC2RDY, AFERESULT_SINC2, &time_out);
#endif    
    digitalWrite(D5, LOW);

    return result;
}

float RP2040_MeasurePin(int pin)
{
    uint16_t value = analogRead(pin);
    float mV = ADC_AVDD * value / ADCFullValue;
    return mV;
}

float RP2040_MeasureOCP()
{
    float wemV = RP2040_MeasurePin(WEpin);
    float remV = RP2040_MeasurePin(REpin);
    return remV - wemV;                     // sending Vre-Vwe according to 'T' command response
}

void PrintOCP(float we, float ocp1, float ocpMeasured)
{
    if (SeeedStatMode)
    {
        char buffer[500];
        sprintf(buffer, "%04.0f,%05.1f,%03.1f", we, ocp1, ocpMeasured);
        Serial.println(buffer);
    }
    else
    {
        Serial.write((uint8_t*)&we, 4);
        Serial.write((uint8_t*)&ocp1, 4);
        Serial.write((uint8_t*)&ocpMeasured, 4);
    }
}

void Do_AD5941_OCP_Measurement()
{
    SWMatrixCfg_Type swm = AD5940_SWMatrixCfgSGet();
    Log(0x80, __LINE__, "REG_AFE_SWCON=0x%lX Dswitch=0x%X Nswitch=0x%x Pswitch=0x%X Tswitch=0x%X ",
          AD5940_ReadReg(REG_AFE_SWCON), swm.Dswitch, swm.Nswitch, swm.Pswitch, swm.Tswitch);

    AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_ADCCNV, bTRUE);  // Power-up ADC and start conversions

    delay(5);

    digitalWrite(D4, HIGH);
    if (ocpCalibrationCycling == bTRUE)
    {
        float step = WEfrom < WEto ? abs(WEstep) : -abs(WEstep);
        // Prevent zero step to avoid infinite loop
        if (fabs(step) < 0.001f) {
            step = (WEfrom < WEto) ? 1.0f : -1.0f;
        }
        // Calculate number of steps
        int steps = (int)((WEto - WEfrom) / step) + 1;
        if (steps < 1) steps = 1;
        for (int i = 0; i < steps; i++)
        {
            float we = WEfrom + i * step;
            // Ensure we don't overshoot due to floating point errors
            if ((step > 0 && we > WEto + 0.5f * step) || (step < 0 && we < WEto + 0.5f * step)) {
                break;
            }
            delayMicroseconds(500);
            digitalWrite(D4, HIGH);

            SetDACLevel(we);
            
            uint32_t ocp_sum = 0;
            float ocpMeasured = 0.0;
            for(uint16_t j = 0; j < OCP_npts; j++)
            {
                ocp_sum += Do1_AD5941_OCP_Measurement();
                ocpMeasured += RP2040_MeasureOCP();
            }

            // sending Vre-Vwe instead of Vwe-Vre
            PrintOCP(we, -(static_cast<float>(ocp_sum) / OCP_npts), ocpMeasured / OCP_npts);

            digitalWrite(D4, LOW);
        }
        // Reset calibration flags after cycling
        ocpCalibration = bFALSE;
        ocpCalibrationCycling = bFALSE;
    }
    else
    {
        OCP_sum = 0;
        for(uint16_t i = 0; i < OCP_npts; i++)
        {
            delayMicroseconds(500);
            OCP_sum += Do1_AD5941_OCP_Measurement();
        }
    }
        
    AD5940_AFECtrlS(AFECTRL_ADCPWR | AFECTRL_ADCCNV, bFALSE);   // Power-down ADC and stop conversions

    digitalWrite(D4, LOW);

    ADCCON = AD5940_ReadReg(REG_AFE_ADCCON);
    
    if (ocpCalibrationCycling != bTRUE)
    {
        Serial.print('Z');
    }
    ocpCalibrationCycling = bFALSE;
    SeeedStatMode = false;
}

// Calculate Open Circuit Potential based on AD5940/AD5941.pdf (Rev. D) page 56
float CalculateOCP()
{
    float ocp_mV;
    float ocp_1 = static_cast<float>(OCP_sum) / OCP_npts;
    if (SeeedStatMode)
    {
        ocp_1 *= -1.0;    // sending Vre-Vwe instead of Vwe-Vre
    }
    
    if (useConstAB)
    {
        float ocp_npts = static_cast<float>(OCP_npts);
        
        ocp_mV = (ocp_1 - constA) / constB;

        Log(0x80, __LINE__, "ocp_1=%.2f constA=%.3f constB=%.3f ocp_mV=%.6f ",
                             ocp_1,     constA,     constB,     ocp_mV);
    }
    else
    {
        uint8_t GNPGA = (ADCCON & BITM_AFE_ADCCON_GNPGA) >> BITP_AFE_ADCCON_GNPGA;
        float Vref_mV = (GNPGA == 1) ? 1835.0 : 1820.0;       // Vref = 1.835V or 1.82V depending on GNPGA
        float PGA_G_values[] = { 1.0, 1.5, 2.0, 4.0, 9.0, 9.0 };
        float PGA_G = PGA_G_values[GNPGA];                    // PGA gain, depends on GNPGA bits of ADCCON register

        uint8_t MUXSELP = (ADCCON & BITM_AFE_ADCCON_MUXSELP) >> BITP_AFE_ADCCON_MUXSELP;    // 0xE: Voltage at SE0 - measured at pin
        uint8_t MUXSELN = (ADCCON & BITM_AFE_ADCCON_MUXSELN) >> BITP_AFE_ADCCON_MUXSELN;    // 0x7: AIN3/BUF_VREF1V8

        float ocp_npts = static_cast<float>(OCP_npts);
        const uint8_t MUXSELN_VBIAS_CAP = 8;                                                // VBIAS_CAP value in MUXSELN
        float VBIAS_CAP_mV = (MUXSELN == MUXSELN_VBIAS_CAP) ? 1110.0 : 0.0;                 // VBIAS_CAP (1.11V) is to be added only if MUXSELN equals to 8
        const float ADC_MIDDLE_AND_ACTIVE_RANGE = 1 << 15;
        
        // based on formulae (12) and (13) on page 56 in AD5940/AD5941.pdf:
        ocp_mV = (ocp_1 - ADC_MIDDLE_AND_ACTIVE_RANGE) * (Vref_mV / PGA_G) / ADC_MIDDLE_AND_ACTIVE_RANGE + VBIAS_CAP_mV;

        Log(0x80, __LINE__, "ADCCON=0x%X GNPGA=%i MUXSELP=0x%X MUXSELN=0x%X VBIAS_CAP=%.2f Vref_mV=%.0f PGA_G=%.1f ocp_1=%.2f ocp_mV=%.6f ",
                             ADCCON,     GNPGA,   MUXSELP,     MUXSELN,   VBIAS_CAP_mV/1000, Vref_mV,   PGA_G,     ocp_1,     ocp_mV);
    }
    
    Log(0x100, __LINE__, "WEmV=%.0f HSDACDAT=%d ocp_1=%.2f ocp_mV=%.3f ",
                          WEmV,     HSDACDAT,   ocp_1,     ocp_mV);
    return ocp_mV;
}

void Config_LPLOOP()
{
    LPDACCfg_Type lpdac_cfg;
    lpdac_cfg.LpdacSel = LPDAC0;
    lpdac_cfg.LpDacVbiasMux = LPDACVBIAS_12BIT; /* Use Vbias to tuning BiasVolt. */
    lpdac_cfg.LpDacVzeroMux = LPDACVZERO_6BIT;  /* Vbias-Vzero = BiasVolt */
    lpdac_cfg.DacData6Bit = vzero;              /* Set Vzero to middle scale. */
    lpdac_cfg.DacData12Bit = vbias;
    lpdac_cfg.DataRst = bFALSE;                 /* Do not reset data register */
    lpdac_cfg.LpDacSW = LPDACSW_VBIAS2LPPA | LPDACSW_VBIAS2PIN | LPDACSW_VZERO2LPTIA | LPDACSW_VZERO2PIN | LPDACSW_VZERO2HSTIA;
    lpdac_cfg.LpDacRef = LPDACREF_2P5;
    lpdac_cfg.LpDacSrc = LPDACSRC_MMR;      /* Use MMR data, we use LPDAC to generate bias voltage for LPTIA - the Vzero */
    lpdac_cfg.PowerEn = bTRUE;              /* Power up LPDAC */
    AD5940_LPDACCfgS(&lpdac_cfg);
}

// The HSDAC has a number of different gain settings as shown below.
// The HSDAC needs to be calibrated seperately for each gain setting. HSDAC has two power
// modes. There are seperate offset registers for both, DACOFFSET and DACOFFSETHP. The
// HSDAC needs to be calibrated for each mode.

// HSDACCON[12] |   HSDACCON[0] |   Output Range    |
// 0            |   0           |   +-607mV         |
// 1            |   0           |   +- 75mV         |
// 1            |   1           |   +- 15.14mV      |
// 0            |   1           |   +-121.2mV       |


// power_mode = 0 for AFEPWR_LP, mode = 1 for AFEPWR_HP
// pgag = 0 (1), 1 (1.5), 2 (2), 3 (4) and 4 (9)

void Calibrate_HSDAC(uint8_t power_mode, uint8_t pga_g)
{
    HSDACCal_Type hsdac_cal;
    ADCPGACal_Type adcpga_cal;
    CLKCfg_Type clk_cfg;

    AD5940_AFEPwrBW(power_mode, AFEBW_250KHZ);    // Changed this line, RJSM 111224

    clk_cfg.ADCClkDiv = ADCCLKDIV_1;
    clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
    clk_cfg.SysClkDiv = SYSCLKDIV_1;
    clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
    clk_cfg.HfOSC32MHzMode = bTRUE;
    clk_cfg.HFOSCEn = bTRUE;
    clk_cfg.HFXTALEn = bFALSE;
    clk_cfg.LFOSCEn = bTRUE;
    AD5940_CLKCfg(&clk_cfg);

    // ADC Offset Cal
    adcpga_cal.AdcClkFreq = 16000000;
    adcpga_cal.ADCPga = pga_g;      // Changed this line, RJSM 111224
    adcpga_cal.ADCSinc2Osr = ADCSINC2OSR_1333;
    adcpga_cal.ADCSinc3Osr = ADCSINC3OSR_4;
    adcpga_cal.PGACalType = PGACALTYPE_OFFSET;
    adcpga_cal.TimeOut10us = 1000;
    adcpga_cal.VRef1p11 = 1.11;
    adcpga_cal.VRef1p82 = 1.82;
    AD5940_ADCPGACal(&adcpga_cal);

    // 607mV Range Cal
    hsdac_cal.ExcitBufGain = EXCITBUFGAIN_2;
    hsdac_cal.HsDacGain = HSDACGAIN_1;
    hsdac_cal.AfePwrMode = power_mode;    // Changed this line, RJSM 111224
    hsdac_cal.ADCSinc2Osr = ADCSINC2OSR_1333;
    hsdac_cal.ADCSinc3Osr = ADCSINC3OSR_4;
    AD5940_HSDACCal(&hsdac_cal);

    // ADC Offset Cal for the PGA gain setting passed in
    adcpga_cal.ADCPga = pga_g;
    AD5940_ADCPGACal(&adcpga_cal);

    // 125mV Range Cal
    hsdac_cal.ExcitBufGain = EXCITBUFGAIN_2;
    hsdac_cal.HsDacGain = HSDACGAIN_0P2;
    AD5940_HSDACCal(&hsdac_cal);

    // 75mV Range Cal
    hsdac_cal.ExcitBufGain = EXCITBUFGAIN_0P25;
    hsdac_cal.HsDacGain = HSDACGAIN_1;
    AD5940_HSDACCal(&hsdac_cal);

    // 15mV Range Cal
    hsdac_cal.ExcitBufGain = EXCITBUFGAIN_0P25;
    hsdac_cal.HsDacGain = HSDACGAIN_0P2;
    AD5940_HSDACCal(&hsdac_cal);
}

bool IsAD5940Calculating()
{
    return !(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH));
}

void eisScan(uint8_t eisMode)
{
    LED(WHITE);
    Calibrate_HSDAC(0, pga_gain);

#if USE_250202_Version
    AD5940_WriteReg(REG_AFE_ADCMAX, 61440);  // important - we need to set up a condition for ADCMAXERR detection !!!
    AD5940_WriteReg(REG_AFE_ADCMAXSMEN, 61184);
#endif

    if(eisMode == 0) LED(BLUE);    // measuring Rz
    else LED(RED);                 // measuring Rcal

    float flo = (float) freqlo / 1000.0;
    float fhi = (float) freqhi / 1000.0;

    float log_start = log10(flo);
    float log_end = log10(fhi);
    float log_step = (log_end - log_start) / (nfreqs - 1);

    for(uint16_t i = 0; i < nfreqs; i++)
    {
        // Very important to initialize the AD5941 before any ADC parameter changes
        AD5941_InitAll();

        Config_LPLOOP(); // sets vbias and vzero

        float frequency = pow(10, log_start + i * log_step);
        Do_WaveGen(eisMode, frequency, amplitude, tia_rf);  // see AD5940_Wavegen131124 for test code

        // Set power mode
        if(frequency > 80000)
        {
#if USE_250202_Version
            HpLoopCfg.HsDacCfg.ExcitBufGain = EXCITBUFGAIN_2;  // or could be EXCITBUFGAIN_0P25
            HpLoopCfg.HsDacCfg.HsDacGain = HSDACGAIN_1;  // or could be HSDACGAIN_0P2
#else
            HpLoopCfg.HsDacCfg.ExcitBufGain = excitbuf_gain;
            HpLoopCfg.HsDacCfg.HsDacGain = hsdac_gain;
#endif
            HpLoopCfg.HsDacCfg.HsDacUpdateRate = 0x07;
            AD5940_HSLoopCfgS(&HpLoopCfg);

            clk_cfg.HfOSC32MHzMode = bTRUE;
            AD5940_CLKCfg(&clk_cfg);

            AD5940_HPModeEn(bTRUE); // 32MHz oscillator
        }
        else
        {
            HpLoopCfg.HsDacCfg.ExcitBufGain = EXCITBUFGAIN_2;  // or could be EXCITBUFGAIN_0P25
            HpLoopCfg.HsDacCfg.HsDacGain = HSDACGAIN_1;  // or could be HSDACGAIN_0P2
            HpLoopCfg.HsDacCfg.HsDacUpdateRate = 0x1B;
            AD5940_HSLoopCfgS(&HpLoopCfg);

            clk_cfg.HfOSC32MHzMode = bFALSE;
            AD5940_CLKCfg(&clk_cfg);

            AD5940_HPModeEn(bFALSE); // 16 Mhz oscillator
        }

        init_AD5940_ADC(frequency); // Configure ADC filter and DFT parameters --- GetFreqParameters is not reliable !!!

        if (SeeedStatMode)
        {
            digitalWrite(D4, HIGH); // show start of delay set with settling_parameter
        }

        // Provide a delay to ensure the waveform generator and ADCfilters have settled
        // 3 options are supported -:
        // 0 = no delay, 1 = frequency-based delay (1 period), >1 = fixed delay in msec

        if(settling_parameter > 1)
        {
            Delay(settling_parameter, 16, 0, 16);
        }
        else if(settling_parameter == 1)
        {
            if(frequency < 100.0)settling_delay_ms = (uint16_t)(1000/frequency) ;
            else settling_delay_ms = 10; // fixed settling above 100 Hz

            Delay(settling_delay_ms, 16, 0, 16);
        }

        if (SeeedStatMode)
        {
            digitalWrite(D4, LOW);  // show end of delay set with settling_parameter
        }

        AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_DFT, bFALSE);
        AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE); // ADC power

        if (adc_delay_ms)
        {
            Delay(adc_delay_ms, -16, 0, -16); // ADC power-up time
        }

        AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_ALLINT, bTRUE); // Enable all interrupts
        AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

        digitalWrite(D4, HIGH);
        AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_DFT, bTRUE);  // Start ADC and DFT

        // while(!(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH)))
        // {}
        Delay(&IsAD5940Calculating, 16, 0, 16);

        // Disable ADCCNV and DFT
        AD5940_AFECtrlS(AFECTRL_ADCCNV | AFECTRL_DFT, bFALSE);

        uint32_t DFTREAL, DFTIMAG;
        //DFTREAL = AD5940_ReadReg(REG_AFE_DFTREAL);
        //DFTIMAG = AD5940_ReadReg(REG_AFE_DFTIMAG);
        DFTREAL = AD5940_ReadReg(REG_AFE_DATAFIFORD);
        DFTIMAG = AD5940_ReadReg(REG_AFE_DATAFIFORD);
        digitalWrite(D4, LOW);

        AddMeasurementToHistory(DFTREAL, DFTIMAG);

        // Pulse D5 if ADC has gone out of range
        if(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_ADCMINERR | AFEINTSRC_ADCMAXERR))
        {
            OutputPulse(D5, 10);
        }

        AD5940_INTCClrFlag(AFEINTSRC_ALLINT);  // Clear all interrupts

        if (SeeedStatMode)
        {
            float real = ToFloat(DFTREAL);
            float imag = ToFloat(DFTIMAG);
            Log(0x20, __LINE__, "eisMode=%i frequency=%.2f DFTREAL=0x%X DFTIMAG=0x%X real=%.2f imag=%.2f ",
                                 eisMode,   frequency, DFTREAL & 0x3ffff, DFTIMAG & 0x3ffff, real, imag);

            if (verbose & 1)
            {
                Serial.print(i + 1);
                Serial.print("=");

                Serial.print(real);
                Serial.print(",");
                Serial.print(imag);
                Serial.println(",");
            }

            if (numberOfMeasurements < sizeof(Measurements) / sizeof(Measurements[0]) / 2)
            {
                *pMeasurement++ = real;
                *pMeasurement++ = imag;
                ++numberOfMeasurements;
            }
        }
        else
        {
            Serial.write((uint8_t*)&DFTREAL, 4);
            Serial.write((uint8_t*)&DFTIMAG, 4);
        }
    }

    LED(GREEN);     // done
    SetPixelsColor(0, 255, 0);
}

void SeeedStatScan()
{
    AD5940_PGA_Calibration();

    pMeasurement = Measurements;

    numberOfMeasurements = 0;
    eisScan(EIS_mode = 0);

    numberOfMeasurements = 0;
    eisScan(EIS_mode = 1);

    CalculateNyquistCurve();
}

void CalculateMagAndPhase(float* pRealAndImag, float& mag, float& phase)
{
    float real = *pRealAndImag++;
    float imag = *pRealAndImag;

    mag = sqrt(real * real + imag * imag);
    phase = atan2(-imag, real);
}

void CalculateNyquistPoint(float* pRZ, float* pRCAL)
{
    float RZmag, RZphase;
    CalculateMagAndPhase(pRZ, RZmag, RZphase);

    float RCALmag, RCALphase;
    CalculateMagAndPhase(pRCAL, RCALmag, RCALphase);

    float ZUnknownMag = abs(RCALmag / RZmag) * fRcal;
    float ZUnknownPhase = RZphase - RCALphase;

    float nyquistPointX = ZUnknownMag * cos(ZUnknownPhase);
    float nyquistPointY = ZUnknownMag * sin(ZUnknownPhase);

    Log(0x20, __LINE__, "RZmag=%.2f RZphase=%.2f", RZmag, RZphase);
    Log(0x20, __LINE__, "RCALmag=%.2f RCALphase=%.2f", RCALmag, RCALphase);
    Log(0x20, __LINE__, "ZUnknownMag=%.2f ZUnknownPhase=%.2f", ZUnknownMag, ZUnknownPhase);
    Log(0x20, __LINE__, "nyquistPointX=%.2f nyquistPointY=%.2f", nyquistPointX, nyquistPointY);

    char buf[100];
    sprintf(buf, "%.3f,%.3f,", nyquistPointX, nyquistPointY);
    Serial.println(buf);
    Serial.flush();
    delay(1);
}

void CalculateNyquistCurve()
{
    float* pRZ = Measurements;                              // points to real and imag pairs of Rz measurements
    float* pRCAL = &Measurements[2 * numberOfMeasurements]; // points to real and imag pairs of Rcal measurements
    for (int i = 0; i < numberOfMeasurements; ++i)
    {
        CalculateNyquistPoint(pRZ, pRCAL);
        pRZ += 2;                                           // points to next pair of real and imag values of Rz measurements
        pRCAL += 2;                                         // points to next pair of real and imag values of Rcal measurements
    }
}

void ShowParameter(const char* name, int value, bool verb)
{
    if (verb)
    {
        Serial.print(name);
        Serial.println(value);
    }
}

void ShowParameter8(const char* name, uint8_t value, bool verb)
{
    if (verb)
    {
        Serial.print(name);
        Serial.println(value == 0 ? "Rz" : "Rcal");
    }
}

void ShowParameterF(const char* format, float value, bool verb)
{
    if (verb)
    {
        char buffer[500];
        sprintf(buffer, format, value);
        Serial.println(buffer);
    }
}

void ShowParameter2(const char* format, int value1, float value2, bool verb)
{
    if (verb)
    {
        char buffer[500];
        sprintf(buffer, format, value1, value2);
        Serial.println(buffer);
    }
}

void ShowAction(const char* name, bool verb)
{
    if (verb)
    {
        Serial.println(name);
    }
}

uint32_t FreqToLabVIEW(float param)
{
    float freq = param * 1000.0;
    return (uint32_t)freq;

}

// Constant values are taken from LabVIEW application
uint16_t ConvertFloatBiasToUint16(float value)
{
    return 1664 + (int)(-value * (1850.0 - 1664.0) / 100.0 + 0.5);
}

float ConvertUint16BiasToFloat(uint16_t value)
{
    float result = static_cast<float>(value);
    result = (float)((result - 1664.0) * 100.0 / (1850.0 - 1664.0));
    return result;
}

uint16_t ConvertFloatAmplitudeToUint16(float value)
{
    return (uint16_t)(value * 126.0 / 50.0 + 0.5);
}

float ConvertUint16AmplitudeToFloat(uint16_t value)
{
    float result = static_cast<float>(value);
    result = (float)(result * 50.0 / 126.0);
    return result;
}

uint16_t ConvertFloatOffsetToUint16(float value)
{
    return (uint16_t)((int16_t)value - 56) & 0xFFF;
}

float ConvertUint16OffsetToFloat(uint16_t value)
{
    // WGOFFSET[11:0] field is SINEOFFSET: 12 bit width in 2's complement
    float result = static_cast<float>(value);
    if (value & (1 << 11))
    {
        int16_t val16 = value | 0xF000;
        result = static_cast<float>(val16);
    }
    result += 56.0;
    return result;
}


// parses token for 2nd uint32_t (i.e. hex2), even if it is greater than 0x7FFFFFFF (sscanf cannot handle value greater than 0x7FFFFFFF)
uint32_t ParseTokenForHex2(const char* token)
{
    uint32_t hex2 = 0;
    char* pc = strchr(token, ',');
    if (pc != NULL)
    {
        char c;
        for (pc += 3; (c = *pc) != 0; ++pc)
        {
            hex2 <<= 4;
            hex2 += c <= '9'
                        ? c - '0'
                        : c <= 'F'
                            ? c - 'A' + 10
                            : c - 'a' + 10;
        }
    }

    return hex2;
}

void setup(void)
{
    AD5940_StructInit(&HpLoopCfg, sizeof(HSLoopCfg_Type));
    AD5940_StructInit(&adc_filter, sizeof(ADCFilterCfg_Type));
    AD5940_StructInit(&adc_base, sizeof(ADCBaseCfg_Type));
    AD5940_StructInit(&DftCfg, sizeof(DFTCfg_Type));
    AD5940_StructInit(&fifo_cfg, sizeof(FIFOCfg_Type));
    AD5940_StructInit(&clk_cfg, sizeof(CLKCfg_Type));
    AD5940_StructInit(&clks_cal, sizeof(ClksCalInfo_Type));

    Serial.begin(1000000);

    pinMode(D4, OUTPUT);
    digitalWrite(D4, LOW);
    pinMode(D5, OUTPUT);
    digitalWrite(D5, LOW);

    AD5941_InitAll();

    pixels.begin();
    pinMode(Power,OUTPUT);
    digitalWrite(Power, HIGH);

    LED(RED);   // sign-on
    delay(250);
    AD5940_PGA_Calibration();
    delay(250);
    LED(GREEN); // start
    SetPixelsColor(0, 255, 0);

    amplitude = ConvertFloatAmplitudeToUint16(fAmplitude);
    vbias = ConvertFloatBiasToUint16(fBias);
    offset = ConvertFloatOffsetToUint16(fOffset);
}

void ShowParameters()
{
    Serial.println("----------------------");

    ShowParameter("verbose mode        (@)=", verbose, true);
    ShowParameter("SeeedStat mode      (S)=", SeeedStatMode, true);
    ShowParameter("pga_gain            (g)=", pga_gain, true);
    //ShowParameter("excitbuf_gain       (h)=", excitbuf_gain, true);
    //ShowParameter("hsdac_gain          (i)=", hsdac_gain, true);
    ShowParameter("tia_rf              (r)=", tia_rf, true);
    ShowParameterF("constA              (i)=%.3f", constA, true);
    ShowParameterF("constB              (j)=%.3f", constB, true);
    ShowParameter("useConstAB             =", useConstAB, true);
    ShowParameter("EIS_mode            (m)=", EIS_mode, true);
    ShowParameter("OCP_npts            (n)=", OCP_npts, true);
    ShowParameter("vzero               (a)=", vzero, true);
    ShowParameter("use_variable_gain   (s)=", use_variable_gain, true);
    ShowParameter("nfreqs              (y)=", nfreqs, true);
    ShowParameter2("vbias               (b)=%i (%.1f mV)", vbias, ConvertUint16BiasToFloat(vbias), true);
    ShowParameter2("amplitude           (z)=%i (0-pk %.1f mV)", amplitude, ConvertUint16AmplitudeToFloat(amplitude), true);
    ShowParameter2("offset              (v)=%i (%.1f mV)", offset, ConvertUint16OffsetToFloat(offset), true);
    ShowParameter("freqlo              (w)=", freqlo, true);
    ShowParameter("freqhi              (x)=", freqhi, true);
    ShowParameter("_CGmax              (t)=", _CGmax, true);
    ShowParameter("_CGmin              (u)=", _CGmin, true);
    ShowParameter("adc_delay_ms        (e)=", adc_delay_ms, true);
    ShowParameter("settling_parameter  (d)=", settling_parameter, true);
    ShowParameter("rcal                (c)=", fRcal, true);
    ShowParameter("numberOfMeasurements =", numberOfMeasurements, true);
    Serial.println("Show history:       !");
    Serial.println("Show parameters:    ?");
    Serial.println("Do eisScan:         E");
    Serial.println("Do SeeedStatScan:   P");
    Serial.println("Do stress test:     f");

    Serial.println("----------------------");

    ShowParameterF("Bias      (B)=%.1f mV", ConvertUint16BiasToFloat(vbias), true);
    ShowParameterF("Amplitude (Y)=%.1f mV", ConvertUint16AmplitudeToFloat(amplitude), true);
    ShowParameterF("Offset    (V)=%.1f mV", ConvertUint16OffsetToFloat(offset), true);

    Serial.println("-------------------");
}

// Monitor commands:
// an sets Vzero (6 bit number)
// bn sets Vbias (12 bit number)
// C PGA calibration
// d settling parameter
// e ADC delay
// E runs EIS scan
// P runs EIS scan [MS] added for working with SeeedStat
// g set PGA gain
// h set Excbug gain
// i set HSDAC gain
// mn sets EIS mode n=0:ExtImp, n=1:Rcal
// r set TIA_Rf
// s use variable gain
// t set combined gain max
// u set combined gain min
// vn sets offset
// wn sets freqlo
// xn sets freqhi
// yn sets nfreqs
// zn sets amplitude

bool ProcessCommand2Int(const char command, uint32_t param1, uint32_t param2)
{
    uint16_t RegAddr;
    uint32_t RegData;
    switch (command)
    {
        case 'O':                       // output to AD5940 port
            RegAddr = (uint16_t)param1;
            RegData = (uint32_t)param2;
            AD5940_WriteReg(RegAddr, RegData);
            char buf[100];
            sprintf(buf, "O 0x%X=0x%X", RegAddr, RegData);
            Serial.println(buf);
            return true;
    }

    return false;
}

bool ProcessCommand2Float(const char command, float param1, float param2)
{
    switch (command)
    {
        case 'D':                       // parameterize EIS or CV measurement by SeeedStat
            SeeedStatMode = true;
            freqlo = FreqToLabVIEW(param1);
            freqhi = FreqToLabVIEW(param2);
            
            V_start = param1;
            V_stop = param2;
            
            Info(1, "V_start=%.1f V_stop=%.1f freqlo=%i freqhi=%i",
                     V_start,     V_stop,     freqlo,   freqhi);
            return true;
            
        case 'O':                       // write (Output) AD5940 register
            uint16_t RegAddr = (uint16_t)param1;
            uint32_t RegData = (uint32_t)param2;
            AD5940_WriteReg(RegAddr, RegData);
            char buf[100];
            sprintf(buf, "O 0x%X=0x%X", RegAddr, RegData);
            Serial.println(buf);
            return true;
    }

    return false;
}

bool ProcessCommand1Int(const char command, uint16_t param)
{
    uint16_t RegAddr;
    uint32_t RegData;
    int n = (int)param;
    switch (command)
    {
        case '@':                       // set verbosity mode
            verbose = n;
            ShowParameter("verbose mode(@)=", verbose, verbose & 1);
            return true;

        case 'I':                       // read (Input) AD5940 register
            RegAddr = (uint16_t)param;
            RegData = AD5940_ReadReg(RegAddr);
            char buf[100];
            sprintf(buf, "I 0x%X=0x%X", RegAddr, RegData);
            Serial.println(buf);
            return true;

        case 'M' :                      // set/reset OcpCalibration parameter
            WEmV = param;
            ocpCalibration = bTRUE;
            ShowParameterF("WEmV=%.0f", WEmV, verbose & 1);
            SetDACLevel(WEmV);
            return true;
    }

    return false;
}

bool ProcessCommand1Float(const char command, float param)
{
    int n = (int)param;
    switch (command)
    {
        case '@':                       // set verbosity mode
            verbose = n;
            ShowParameter("verbose mode(@)=", verbose, verbose & 1);
            return true;

        case 'a':                       // set vzero in binary
            vzero = n;
            ShowParameter("vzero(a)=", vzero, verbose & 1);
            return true;

        case 'B':                       // set bias in mV
            fBias = param;
            n = ConvertFloatBiasToUint16(fBias);    // falling through case 'b':
        case 'b':
            vbias = n;
            ShowParameter2("vbias(b)=%i (%.2f mV)", vbias, ConvertUint16BiasToFloat(vbias), verbose & 1);
            return true;

        case 'c':                       // set Rcal in Ohm
            fRcal = param;
            ShowParameter("RCal(d)=", fRcal, verbose & 1);
            return true;

        // case 'd':
            // settling_parameter = n;
            // ShowParameter("settling_parameter(d)=", settling_parameter, verbose & 1);
            // return true;

        // case 'e':
            // adc_delay_ms = n;
            // ShowParameter("adc_delay_ms(e)=", adc_delay_ms, verbose & 1);
            // return true;

        // PGA Gain options - 3 bit code
        // Gain = 1 (0), 1.5 (1), 2 (2), 4 (3), 9 (4)
        case 'g':                       // set PGA gain
            pga_gain = n;
            adc_base.ADCPga = pga_gain;
            ShowParameter("pga_gain(g)=", pga_gain, verbose & 1);
            return true;

        // case 'h':
            // excitbuf_gain = n;
            // ShowParameter("excitbuf_gain(h)=", excitbuf_gain, verbose & 1);
            // return true;

        // // PGA Gain options - 3 bit code
        // // Gain = 1 (0), 1.5 (1), 2 (2), 4 (3), 9 (4)

        // case 'i':
            // hsdac_gain = n;
            // ShowParameter("hsdac_gain(i)=", hsdac_gain, verbose & 1);
            // return true;

        case 'i':
            constA = param;
            ShowParameterF("constA(i)=%.3f", constA, verbose & 1);
            useConstAB = true;
            return true;

        case 'j':
            constB = param;
            ShowParameterF("constB(j)=%.3f", constB, verbose & 1);
            useConstAB = true;
            return true;

        case 'M' :                      // set/reset OcpCalibration parameter
            WEmV = param;
            ocpCalibration = bTRUE;
            ShowParameterF("WEmV=%.0f", WEmV, verbose & 1);
            SetDACLevel(WEmV);
            return true;

        case 'm':
            EIS_mode = n;
            ShowParameter("EIS_mode(m)=", EIS_mode, verbose & 1);
            return true;

        case 'n' :
            OCP_npts = n ;
            ShowParameter("OCP_npts(n)=", OCP_npts, verbose & 1);
            return true;

        case 'r':
            tia_rf = n;
            ShowParameter("tia_rf(r)=", tia_rf, verbose & 1);
            return true;

        case 's':
            use_variable_gain = n;
            ShowParameter("use_variable_gain(s)=", use_variable_gain, verbose & 1);
            return true;

        case 'S':
            SeeedStatMode = n ? true : false;
            ShowParameter("SeeedStat mode(S)=", SeeedStatMode, verbose & 1);
            return true;

        case 't':
            _CGmax = n;
            ShowParameter("_CGmax(t)=", _CGmax, verbose & 1);
            return true;

        case 'u':
            _CGmin = n;
            ShowParameter("_CGmin(u)=", _CGmin, verbose & 1);
            return true;

        case 'V':                                           // set offset in mV
            fOffset = param;
            n = ConvertFloatOffsetToUint16((uint16_t)fOffset);  // falling through case 'v':
        case 'v':
            offset = (uint16_t)n;
            ShowParameter2("offset(v)=%i (%.2f)", offset, ConvertUint16OffsetToFloat(offset), verbose & 1);
            return true;

        case 'W':                                           // set freqlo in Hz
            n = FreqToLabVIEW(param);
        case 'w':
            freqlo = n;
            ShowParameter("freqlo(w)=", freqlo, verbose & 1);
            return true;

        case 'X':                                           // set freqhi in Hz
            n = FreqToLabVIEW(param);
        case 'x':
            freqhi = n;
            ShowParameter("freqhi(x)=", freqhi, verbose & 1);
            return true;

        case 'y':
            nfreqs = n;
            ShowParameter("nfreqs(y)=", nfreqs, verbose & 1);
            return true;

        case 'Y':                                           // set amplitude in mV
            fAmplitude = param;
            n = ConvertFloatAmplitudeToUint16(fAmplitude);  // falling through case 'z':
        case 'z':
            amplitude = n;
            ShowParameter2("amplitude(z)=%i (0-pk %.2f mV)", amplitude, ConvertUint16AmplitudeToFloat(amplitude), verbose & 1);
            return true;
    }

    return false;
}

bool ProcessCommand(const char command)
{
    char buffer[30];
    float ocp_1, ocp_mV;
    switch (command)
    {
        case '?':   // [MS] added '?' for investigation purposes
            ShowParameters();
            return true;

        case '!':   // [MS] added '!' for investigation purposes
            Serial.println(history);
            pHistory = history;
            return true;

        case 'C':
            ShowAction("AD5940_PGA_Calibration(C)", verbose & 1);
            AD5940_PGA_Calibration();
            return true;

        case 'E':
            ShowAction("eisScan(E)", verbose & 1);
            eisScan(EIS_mode);
            return true;

        case 'f':   // [MS] added 'f' stress test (for only diagnostic purpose)
            ShowAction("eisScan(E)", verbose & 1);
            while (!Serial.available())
            {
                pMeasurement = Measurements;
                eisScan(EIS_mode = 0);
            }
            while (Serial.available())
            {
                Serial.read();
            }
            return true;

        case 'I' :
            AD5941_InitAll() ;
            Config_AD5941_OCP_Measurement(WEmV);
            ocpCalibration = bFALSE;
            useConstAB = false;
            return true;

        case 'M' :                      // start ramp test (cv)
            LED(RED);
            cvSetup(V_start, V_stop);
            LED(GREEN);
            SetPixelsColor(0, 255, 0);
            return true;

        case 'O' :
            LED(MAGENTA);
            Do_AD5941_OCP_Measurement();
            LED(GREEN);
            SetPixelsColor(0, 255, 0);
            return true;

        case 'P':   // [MS] added 'P' for working with SeeedStat
            ShowAction("SeeedStatScan(P)", verbose & 1);
            SeeedStatScan();
            SeeedStatMode = false;
            return true;

        case 'T' :
            delay(10);
            ocp_mV = CalculateOCP();
            sprintf(buffer, "%.6f ", ocp_mV);  // sending Vre-Vwe instead of Vwe-Vre
            Serial.println(buffer);
            return true;

        case 'U' :
            delay(10) ;
            Serial.write((uint8_t *)&OCP_sum, 4) ;

            if (verbose & 0x80)
            {
                CalculateOCP();
            }
            return true;

        case 'Z' : // we issue a Z command when a run completes to reset the AD5941
            AD5941_InitAll() ;
            return true;
    }

    return false;
}

void ProcessToken(char* token)
{
    
    Log(1, __LINE__, "ProcessToken:%s", token);

    char command;
    float float1;
    float float2;
    uint32_t hex1;
    uint32_t hex2;
    int32_t int1;
    int32_t int2;
    int32_t int3;
    bool success = false;

    // ===== CHECK 1: OCP calibration =====
    if ((sscanf(token, "M%i,%i,%i", &int1, &int2, &int3) == 3))
    {
        WEfrom = static_cast<float>(int1);
        WEto = static_cast<float>(int2);
        WEstep = static_cast<float>(int3);
        ocpCalibration = ocpCalibrationCycling = bTRUE;
        
        analogReadResolution(ADCresolution);
        pinMode(A0, INPUT);
        pinMode(A3, INPUT);
        
        success = true;
    }
    // ===== CHECK 2: D command (5 parameters) =====
    else
    {
        float t1, t2, t3, t4;
        int t5;
        int result = sscanf(token, "D %f,%f,%f,%f,%i", &t1, &t2, &t3, &t4, &t5);
        
        if (result == 5)
        {
            V_start = t1;
            V_stop = t2;
            Estep = t3;
            ScanRate = t4;
            CycleNumber = t5;
            
            SeeedStatMode = true;
            freqlo = FreqToLabVIEW(V_start);
            freqhi = FreqToLabVIEW(V_stop);
            
            Info(1, "V_start=%.1f V_stop=%.1f Estep=%.1f ScanRate=%.1f CycleNumber=%u",
                 V_start, V_stop, Estep, ScanRate, CycleNumber);
            
            success = true;
        }
    }
    
    // ===== CHECK 3: Memory commands =====
    if (!success)
    {
        if (sscanf(token, "ri80x%lX", &hex1) == 1)
        {
            int8_t i = *(int8_t*)hex1;
            Info(1, "0x%lX=0x%lX", hex1, i);
            Log(1, __LINE__, "0x%lX=0x%X", hex1, i);
            success = true;
        }
        else if (sscanf(token, "ri160x%lX", &hex1) == 1)
        {
            int16_t i = *(int16_t*)hex1;
            Info(1, "0x%lX=0x%lX", hex1, i);
            Log(1, __LINE__, "0x%lX=0x%X", hex1, i);
            success = true;
        }
        else if (sscanf(token, "ri320x%lX", &hex1) == 1)
        {
            int32_t i = *(int32_t*)hex1;
            Info(1, "0x%lX=0x%lX", hex1, i);
            Log(1, __LINE__, "0x%lX=0x%lX", hex1, i);
            success = true;
        }
        else if (sscanf(token, "ru80x%lX", &hex1) == 1)
        {
            uint8_t u = *(uint8_t*)hex1;
            Info(1, "0x%lX=0x%lX", hex1, u);
            Log(1, __LINE__, "0x%lX=0x%X", hex1, u);
            success = true;
        }
        else if (sscanf(token, "ru160x%lX", &hex1) == 1)
        {
            uint16_t u = *(uint16_t*)hex1;
            Info(1, "0x%lX=0x%lX", hex1, u);
            Log(1, __LINE__, "0x%lX=0x%X", hex1, u);
            success = true;
        }
        else if (sscanf(token, "ru320x%lX", &hex1) == 1)
        {
            uint32_t u = *(uint32_t*)hex1;
            Info(1, "0x%lX=0x%lX", hex1, u);
            Log(1, __LINE__, "0x%lX=0x%lX", hex1, u);
            success = true;
        }
        // write memory commands
        else if (sscanf(token, "wi80x%lX,0x%lX", &hex1, &hex2) == 2)
        {
            *(int8_t*)hex1 = (int8_t)hex2;
            Info(1, "0x%lX <- 0x%lX", hex1, hex2);
            Log(1, __LINE__, "0x%lX <- 0x%lX", hex1, hex2);
            success = true;
        }
        else if (sscanf(token, "wi160x%lX,0x%lX", &hex1, &hex2) == 2)
        {
            *(int16_t*)hex1 = (int16_t)hex2;
            Info(1, "0x%lX <- 0x%lX", hex1, hex2);
            Log(1, __LINE__, "0x%lX <- 0x%lX", hex1, hex2);
            success = true;
        }
        else if (sscanf(token, "wi320x%lX,0x%lX", &hex1, &hex2) == 2)
        {
            *(int32_t*)hex1 = (int32_t)hex2;
            Info(1, "0x%lX <- 0x%lX", hex1, hex2);
            Log(1, __LINE__, "0x%lX <- 0x%lX", hex1, hex2);
            success = true;
        }
        else if (sscanf(token, "wu80x%lX,0x%lX", &hex1, &hex2) == 2)
        {
            *(uint8_t*)hex1 = (uint8_t)hex2;
            Info(1, "0x%lX <- 0x%lX", hex1, hex2);
            Log(1, __LINE__, "0x%lX <- 0x%lX", hex1, hex2);
            success = true;
        }
        else if (sscanf(token, "wu160x%lX,0x%lX", &hex1, &hex2) == 2)
        {
            *(uint16_t*)hex1 = (uint16_t)hex2;
            Info(1, "0x%lX <- 0x%lX", hex1, hex2);
            Log(1, __LINE__, "0x%lX <- 0x%lX", hex1, hex2);
            success = true;
        }
        else if (sscanf(token, "wu320x%lX,0x%lX", &hex1, &hex2) == 2)
        {
            *(uint32_t*)hex1 = (uint32_t)hex2;
            Info(1, "0x%lX <- 0x%lX", hex1, hex2);
            Log(1, __LINE__, "0x%lX <- 0x%lX", hex1, hex2);
            success = true;
        }
    }
    
    // ===== CHECK 4: 2-parameter hex commands =====
    if (!success)
    {
        if (sscanf(token, "%c 0x%lX,0x%lX", &command, &hex1, &hex2) == 3)
        {
            Log(1, __LINE__, "ParseTokenForHex2:%c 0x%lX,0x%lX", command, hex1, hex2);
            hex2 = ParseTokenForHex2(token);
            success = ProcessCommand2Int(command, hex1, hex2);
        }
        else if (sscanf(token, "%c0x%lX,0x%lX", &command, &hex1, &hex2) == 3)
        {
            Log(1, __LINE__, "ParseTokenForHex2:%c0x%lX,0x%lX", command, hex1, hex2);
            hex2 = ParseTokenForHex2(token);
            success = ProcessCommand2Int(command, hex1, hex2);
        }
    }

    // ===== CHECK 5: 2-parameter float commands =====
    if (!success)
    {
        if (sscanf(token, "%c %f,%f", &command, &float1, &float2) == 3)
        {
            Log(1, __LINE__, "ProcessCommand2Float:%c %f,%f", command, float1, float2);
            success = ProcessCommand2Float(command, float1, float2);
        }
        else if (sscanf(token, "%c%f,%f", &command, &float1, &float2) == 3)
        {
            Log(1, __LINE__, "ProcessCommand2Float:%c%f,%f", command, float1, float2);
            success = ProcessCommand2Float(command, float1, float2);
        }
    }

    // ===== CHECK 6: 1-parameter commands =====
    if (!success)
    {
        if (sscanf(token, "%c %f", &command, &float1) == 2)
        {
            Log(1, __LINE__, "ProcessCommand1Float:%c %f", command, float1);
            success = ProcessCommand1Float(command, float1);
        }
        else if (sscanf(token, "%c%f", &command, &float1) == 2)
        {
            Log(1, __LINE__, "ProcessCommand1Float:%c%f", command, float1);
            success = ProcessCommand1Float(command, float1);
        }
    }
    
    // ===== CHECK 7: Single character commands =====
    if (!success)
    {
        command = token[0];
        Log(1, __LINE__, "ProcessCommand:%c", command);
        success = ProcessCommand(command);
    }

    if (!success)
    {
        Info(1, "Unrecognized command: %c", command);
        Log(1, __LINE__, "Unrecognized command: %c", command);
    }
}

void SplitAndProcessCommands(char* buf)
{
    // tokenize the input command sequence and execute them one by one
    char commandDelimiters[] = ";|\r\n";
    for (char* token = strtok(buf, commandDelimiters); token; token = strtok(NULL, commandDelimiters))
    {
        if (strlen(token) > 0)
        {
            AddCommandToHistory(token);
            ProcessToken(token);
        }
    }
}

void loop()
{
    if (Serial.available())
    {
        char buf[1000];
        int n = Serial.readBytes (buf, sizeof(buf) - 1);
        buf[n] = 0;
        if (buf[--n] == '\n')
        {
            buf[n] = 0;
        }

        Info(1, "%s", buf);

        SplitAndProcessCommands(buf);
    }
}
