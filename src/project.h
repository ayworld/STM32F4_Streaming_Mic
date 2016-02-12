#ifndef _PROJECT_H_
#define _PROJECT_H_

#include "ch.h"

void mp45dt02Init(void);
void mp45dt02Shutdown(void);

void lan8720Init(void);
void lan8720Shutdown(void);
void lan8720TestUDP(void);

void debugSerialPrint(const char * fmt, ...);

#define _PRINT(FMT, ...)

#if 1
#define PRINT(FMT, ...)                                                     \
        debugSerialPrint("(%s:%d) " FMT "\n\r", __FILE__, __LINE__, __VA_ARGS__)  
#else
#define PRINT(FMT, ...) _PRINT(FMT, ...)
#endif

#define PRINT_ERROR(FMT, ...)\
        PRINT("ERROR:" FMT, __VA_ARGS__);                   \
                                                            \
        while (1)                                           \
        {                                                   \
            LED_RED_SET();                                  \
            chThdSleepMilliseconds(500);                    \
            LED_RED_CLEAR();                                \
            chThdSleepMilliseconds(500);                    \
            LED_RED_SET();                                  \
            chSysHalt("ERROR");                             \
        }

#define LED_ORANGE_SET()    palSetPad(GPIOD, GPIOD_LED3);
#define LED_ORANGE_CLEAR()  palClearPad(GPIOD, GPIOD_LED3);
#define LED_ORANGE_TOGGLE() palTogglePad(GPIOD, GPIOD_LED3);

#define LED_GREEN_SET()     palSetPad(GPIOD, GPIOD_LED4);
#define LED_GREEN_CLEAR()   palClearPad(GPIOD, GPIOD_LED4);
#define LED_GREEN_TOGGLE()  palTogglePad(GPIOD, GPIOD_LED4);

#define LED_RED_SET()       palSetPad(GPIOD, GPIOD_LED5);
#define LED_RED_CLEAR()     palClearPad(GPIOD, GPIOD_LED5);
#define LED_RED_TOGGLE()    palTogglePad(GPIOD, GPIOD_LED5);

#define LED_BLUE_SET()      palSetPad(GPIOD, GPIOD_LED6);
#define LED_BLUE_CLEAR()    palClearPad(GPIOD, GPIOD_LED6);
#define LED_BLUE_TOGGLE()   palTogglePad(GPIOD, GPIOD_LED6);

#endif /* _PROJECT_H_ */
