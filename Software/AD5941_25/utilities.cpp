#include <arduino.h>
#include <Adafruit_NeoPixel.h>
#include "utilities.h"

extern Adafruit_NeoPixel pixels;
extern uint8_t pixelsRed, pixelsGreen, pixelsBlue;

// Set proper bit in verbose to show and log changes in internal states of variables
// 1 - activates Info to Serial
// 2.. - activates info and log channels
uint32_t verbose = 0x0;

#define HISTORY_LENGTH  150000
char history[HISTORY_LENGTH];
char* pHistory = history;

#define	BLINK_PERIOD	100
long nextBlinkTime = 0;

const int markerPin = D4;    // select the output pin for marker
const int PINB = 25;
const int PING = 16;
const int PINR = 17;

uint8_t pixelsColor = 0;
uint8_t pixelsRed = 0;
uint8_t pixelsGreen = 0;
uint8_t pixelsBlue = 0;

void Info(uint32_t mask, const char* format, ...)
{
    if ((verbose & 1) && (verbose & mask))
    {
        char buffer[300];

        va_list args;
        va_start(args, format);
        vsprintf(buffer, format, args);
        va_end(args);

        Serial.println(buffer);
    }
}

// Converts 18 bit two's complement (DFTREAL and DFTIMAG) into float
float ToFloat(uint32_t num)
{
    num &= 0x3ffff;                     // Data is 18bit in two's complement, bit17 is the sign bit, lowest 2 bits are fraction
    int8_t fraction = num & 3;          // lowest 2 bits are fractional part
    int16_t integer = num >> 2;         // bits 17:2 are integer part, now shifted to right by 2
    return static_cast<float>(integer) + (static_cast<float>(fraction) / 4.0);
}

bool AddTextToHistory(const char* text)
{
    size_t length = strlen(text);
    if (pHistory + length >= history + sizeof(history) - 1)
    {
        return false;
    }

    strcpy(pHistory, text);
    pHistory += length;
    *pHistory = '\0';

    return true;
}

void AddMeasurementToHistory(uint32_t dftReal, uint32_t dftImag)
{
    float real = ToFloat(dftReal);
    float imag = ToFloat(dftImag);
    char buf[100];
    sprintf(buf, "\n[%.2f,%.2f]", real, imag);
    AddTextToHistory(buf);
}

void AddCommandToHistory(const char* pCommand)
{
    if (AddTextToHistory(pCommand))
    {
        AddTextToHistory(";");
    }
}

void Log(uint32_t mask, int lineNum, const char* format, ...)
{
    if ((verbose & mask) != 0)
    {
        char buffer[500];

        sprintf(buffer, "\n#%i ", lineNum);

        va_list args;
        va_start(args, format);
        vsprintf(buffer + strlen(buffer), format, args);
        va_end(args);

        AddTextToHistory(buffer);
    }
}

void OutputPulse(int pin, int usec, bool invert)
{
    bool state = invert ? digitalRead(pin) : LOW;
    digitalWrite(pin, !state);
    delayMicroseconds(usec);
    digitalWrite(pin, state);
}

void SetPixelsColor(uint8_t red, uint8_t green, uint8_t blue)
{
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(red, green, blue));
    pixels.show();
}

void ChangePixelsColor(int16_t& red, int16_t redInc, int16_t& green, int16_t greenInc, int16_t& blue, int16_t blueInc)
{
    //pixels.clear();

    red += redInc;
    if (red > pixelsRed)    red = 0;
    else if (red < 0)       red = pixelsRed;

    green += greenInc;
    if (green > pixelsGreen)  green = 0;
    else if (green < 0)       green = pixelsGreen;

    blue += blueInc;
    if (blue > pixelsBlue) blue = 0;
    else if (blue < 0)     blue = pixelsBlue;

    //pixels.setPixelColor(0, pixels.Color((uint8_t)red, (uint8_t)green, (uint8_t)blue));
    //pixels.show();
    SetPixelsColor(red, green, blue);
}

void Delay(uint32_t time, int16_t redInc, int16_t greenInc, int16_t blueInc)
{
    uint32_t until = millis() + time;

    int16_t red = 0;
    int16_t green = 0;
    int16_t blue = 0;

    while (millis() < until)
    {
        ChangePixelsColor(red, redInc, green, greenInc, blue, blueInc);

        uint32_t diff = until - millis();
        delay(diff < 100 ? diff : 100);
    }
}

void Delay(bool (*func)(), int16_t redInc, int16_t greenInc, int16_t blueInc)
{
    int16_t red = 0;
    int16_t green = 0;
    int16_t blue = 0;

    while ((*func)())
    {
        ChangePixelsColor(red, redInc, green, greenInc, blue, blueInc);

        delay(100);
    }
}

void LED(uint8_t colorval)
{
    pixels.clear();
    switch(colorval)
    {
        case RED:
            pixels.setPixelColor(0, pixels.Color(pixelsRed = 255, pixelsGreen = 0,   pixelsBlue = 0));
            break;
        case ORANGE:
            pixels.setPixelColor(0, pixels.Color(pixelsRed = 255, pixelsGreen = 128, pixelsBlue = 0));
            break;
        case YELLOW:
            pixels.setPixelColor(0, pixels.Color(pixelsRed = 255, pixelsGreen = 255, pixelsBlue = 0));
            break;
        case GREEN:
            pixels.setPixelColor(0, pixels.Color(pixelsRed = 0,   pixelsGreen = 255, pixelsBlue = 0));
            break;
        case CYAN:
            pixels.setPixelColor(0, pixels.Color(pixelsRed = 0,   pixelsGreen = 255, pixelsBlue = 255));
            break;
        case BLUE:
            pixels.setPixelColor(0, pixels.Color(pixelsRed = 0,   pixelsGreen = 0,   pixelsBlue = 255));
            break;
        case PURPLE:
            pixels.setPixelColor(0, pixels.Color(pixelsRed = 128, pixelsGreen = 0,   pixelsBlue = 255));
            break;
        case MAGENTA:
            pixels.setPixelColor(0, pixels.Color(pixelsRed = 255, pixelsGreen = 0,   pixelsBlue = 255));
            break;
        case PINK:
            pixels.setPixelColor(0, pixels.Color(pixelsRed = 255, pixelsGreen = 102, pixelsBlue = 78));
            break;
        case WHITE:
            pixels.setPixelColor(0, pixels.Color(pixelsRed = 255, pixelsGreen = 255, pixelsBlue = 255));
            break;
    }

    pixels.show();
    pixelsColor = colorval;
}

// -----------------------------------------------------------------------------
void markerToggle()
{
	//ENTER("cmd1 markerToggle");
	digitalWrite(markerPin, digitalRead(markerPin) ? LOW : HIGH);
	BlinkLed();
	//LEAVE;
}
// -----------------------------------------------------------------------------
void markerOn()
{
	//ENTER("cmd1 markerOn");
	digitalWrite(markerPin, HIGH);
	BlinkLed();
	//LEAVE;
}
// -----------------------------------------------------------------------------
void markerOff()
{
	//ENTER("cmd1 markerOff");
	digitalWrite(markerPin, LOW);
	//LEAVE;
}
// -----------------------------------------------------------------------------
void LightLed(/*int color,*/ bool red, bool green, bool blue)
{
	//ENTER("cmd1 LightLed");
	digitalWrite(PINB, blue);
	digitalWrite(PING, green);
	digitalWrite(PINR, red);
	//LEAVE;
}
// -----------------------------------------------------------------------------
void BlinkLed()
{
#define	LED_ON	LOW
#define	LED_OFF	HIGH

    static int color = 0;
	if (millis() > nextBlinkTime)
	{
		nextBlinkTime = millis() + BLINK_PERIOD;
		switch(color % 8)
		{
			case 0:	LightLed(/*color,*/ LED_OFF, LED_OFF, LED_OFF);			break;
			case 1:	LightLed(/*color,*/ LED_OFF, LED_OFF, LED_ON);			break;
			case 2:	LightLed(/*color,*/ LED_OFF, LED_ON,  LED_OFF);			break;
			case 3:	LightLed(/*color,*/ LED_OFF, LED_ON,  LED_ON);	    	break;
			case 4:	LightLed(/*color,*/ LED_ON,  LED_OFF, LED_OFF);			break;
			case 5:	LightLed(/*color,*/ LED_ON,  LED_OFF, LED_ON);  		break;
			case 6:	LightLed(/*color,*/ LED_ON,  LED_ON,  LED_OFF);		    break;
			case 7:	LightLed(/*color,*/ LED_ON,  LED_ON,  LED_ON);			break;
		}
		++color;
	}
}
// -----------------------------------------------------------------------------

