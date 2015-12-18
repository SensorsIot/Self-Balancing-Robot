// Debug utilities

#if DEBUG >= 0
#define DEBUGPRINT0(x) Serial.print(x)
#define DEBUGPRINTLN0(x) Serial.println(x)
#else
#define DEBUGPRINT0(x)
#define DEBUGPRINTLN0(x)
#endif

#if DEBUG >= 1
#define DEBUGPRINT1(x) Serial.print(x)
#define DEBUGPRINTLN1(x) Serial.println(x)
#else
#define DEBUGPRINT1(x)
#define DEBUGPRINTLN1(x)
#endif

#if DEBUG >= 2
#define DEBUGPRINT2(x) Serial.print(x)
#define DEBUGPRINTLN2(x) Serial.println(x)
#else
#define DEBUGPRINT2(x)
#define DEBUGPRINTLN2(x)
#endif

#if DEBUG >= 3
#define DEBUGPRINT3(x) Serial.print(x)
#define DEBUGPRINTLN3(x) Serial.println(x)
#else
#define DEBUGPRINT3(x)
#define DEBUGPRINTLN3(x)
#endif

// fast port handling

#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

#define testPinLow(pin) {digitalWrite(pin, LOW); digitalWrite(pin, HIGH); }
#define testPinhigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }