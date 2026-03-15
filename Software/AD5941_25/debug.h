#define	DEBUG 	0
#define	MEASURE	0

extern void Log(uint32_t mask, int lineNum, const char* format, ...);
extern void Info(uint32_t mask, const char* format, ...);

#if DEBUG
extern void PrintTime(long time);
extern char Pipes[];
extern int Indentation;
extern long loopStartTime;

	#define ENTER(FunctionName)		long __entry__ = millis();			\
									const char __functionName__[] = FunctionName; \
									Serial.print(Pipes + Indentation);	\
									Serial.print("/- ");				\
									Serial.print(FunctionName);			\
									Serial.print(" ms=");				\
									Serial.println(__entry__ - loopStartTime); \
									Indentation -= 2;
								
	#define LEAVE				{										\
									Indentation += 2;					\
									Serial.print(Pipes + Indentation);	\
									Serial.print("\\- ");				\
									Serial.print(__functionName__);		\
									Serial.print("#");					\
									Serial.print(__LINE__);				\
									Serial.print(" ");					\
									PrintTime(millis() - loopStartTime);\
									Serial.print(", duration=");		\
									PrintTime(millis() - __entry__);	\
									Serial.println();					\
								}

									
	#define	RETURN(Value)		{										\
									Indentation += 2;					\
									Serial.print(Pipes + Indentation);	\
									Serial.print("\\- ");				\
									Serial.print(__functionName__);		\
									Serial.print("#");					\
									Serial.print(__LINE__);				\
									Serial.print(" ");					\
									PrintTime(millis() - loopStartTime);\
									Serial.print(" return (");			\
									Serial.print(Value);				\
									Serial.print("), duration=");		\
									PrintTime(millis() - __entry__);	\
									Serial.println();					\
									return Value;						\
								}

	#define	PRINTVAR(Name, Value) {										\
									Serial.print(Pipes + Indentation);	\
									Serial.print("   #");				\
									Serial.print(__LINE__);				\
									Serial.print(" ");					\
									PrintTime(millis() - loopStartTime);\
									Serial.print(" ");					\
									Serial.print(Name);					\
									Serial.print("=");					\
									Serial.println(Value);				\
								}

	#define	PRINTVARHEX(Name, Value) {									\
									Serial.print(Pipes + Indentation);	\
									Serial.print("   #");				\
									Serial.print(__LINE__);				\
									Serial.print(" ");					\
									PrintTime(millis() - loopStartTime);\
									Serial.print(" ");					\
									Serial.print(Name);					\
									Serial.print("=");					\
									Serial.println(Value, HEX);			\
								}

	#define	PRINT2VARHEX(Value1, Value2) {								\
									Serial.print(Pipes + Indentation);	\
									Serial.print("   #");				\
									Serial.print(__LINE__);				\
									Serial.print(" ");					\
									PrintTime(millis() - loopStartTime);\
									Serial.print(" ");					\
									Serial.print(Value1, HEX);			\
									Serial.print("=");					\
									Serial.println(Value2, HEX);		\
								}

	#define	PRINTVARBIN(Name, Value) {									\
									Serial.print(Pipes + Indentation);	\
									Serial.print("   #");				\
									Serial.print(__LINE__);				\
									Serial.print(" ");					\
									PrintTime(millis() - loopStartTime);\
									Serial.print(" ");					\
									Serial.print(Name);					\
									Serial.print("=");					\
									Serial.println(Value, BIN);			\
								}

	#define	SETVALUE(Name, Expression)									\
								{										\
									Serial.print(Pipes + Indentation);	\
									Serial.print("   #");				\
									Serial.print(__LINE__);				\
									Serial.print(" ");					\
									PrintTime(millis() - loopStartTime);\
									Serial.print(" ");					\
									Serial.print(Name);					\
									Serial.print("=");					\
									Serial.println(Expression);			\
								}

	#define	PRINTLINENUMBER		{										\
									Serial.print(Pipes + Indentation);	\
									Serial.print("   ");				\
									Serial.print(__functionName__);		\
									Serial.print("#");					\
									Serial.print(__LINE__);				\
									Serial.print(" ms=");				\
									PrintTime(millis() - loopStartTime);\
									Serial.println();			\
								}	
  #if MEASURE
	typedef struct
	{
		long time;
		int value;
	} Measurement_Type;

	#define MEASUREMENTS		20000
	#define MEASUREMENT_PERIOD	25	// in millisec

	extern int sensorPin;    // select the input pin for the potentiometer
	extern Measurement_Type measurements[];
	extern uint32_t measurementIndex;
	extern long measurementTime;
	extern int measurementPeriod;
	extern void Measure();
  #endif
#else
	#define ENTER(FunctionName)
	#define LEAVE
	#define RETURN(Value)				return Value;
	#define	PRINTVAR(Name, Value)
	#define	PRINTVARHEX(Name, Value)
	#define	PRINT2VARHEX(Value1, Value2)
	#define	PRINTVARBIN(Name, Value)
	#define	SETVALUE(Name, Expression)	(Expression);
	#define	PRINTLINENUMBER
#endif

#define	LED_ON	LOW
#define	LED_OFF	HIGH
extern void LightLed(bool red, bool green, bool blue);
extern void BlinkLed();
