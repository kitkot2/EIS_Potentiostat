// LED color codes
#define RED     0
#define ORANGE  1
#define YELLOW  2
#define GREEN   3
#define CYAN    4
#define BLUE    5
#define PURPLE  6
#define MAGENTA 7
#define PINK    8
#define WHITE   9

// Converts 18 bit two's complement (DFTREAL and DFTIMAG) into float
extern float ToFloat(uint32_t num);
extern void Info(uint32_t mask, const char* format, ...);
extern bool AddTextToHistory(const char* text);
extern void AddMeasurementToHistory(uint32_t dftReal, uint32_t dftImag);
extern void AddCommandToHistory(const char* pCommand);
extern void Log(uint32_t mask, int lineNum, const char* format, ...);
extern void OutputPulse(int pin, int usec, bool invert = false);

extern void LED(uint8_t colorval);
extern void SetPixelsColor(uint8_t red, uint8_t green, uint8_t blue);
extern void ChangePixelsColor(int16_t& red, int16_t redInc, int16_t& green, int16_t greenInc, int16_t& blue, int16_t blueInc);
extern void Delay(uint32_t time, int16_t redInc, int16_t greenInc, int16_t blueInc);
extern void Delay(bool (*func)(), int16_t redInc, int16_t greenInc, int16_t blueInc);
extern void markerToggle();
extern void markerOn();
extern void markerOff();
extern void BlinkLed();
