#ifndef DEBUG_H
#define DEBUG_H

#include <stdint.h>
#include <stdio.h>

#define DEBUG_LEVEL_ALL         0
#define DEBUG_LEVEL_VERBOSE     1
#define DEBUG_LEVEL_INFO        2
#define DEBUG_LEVEL_WARN        3
#define DEBUG_LEVEL_ERROR       4

#define DEBUG_LEVEL             DEBUG_LEVEL_VERBOSE  

#include "SEGGER_RTT.h"
#if 1
#define KNRM  "\x1B[0m"
#define KRED  RTT_CTRL_TEXT_RED
#define KGRN  RTT_CTRL_TEXT_GREEN
#define KYEL  RTT_CTRL_TEXT_YELLOW
#define KBLU  RTT_CTRL_TEXT_BLUE
#define KMAG  RTT_CTRL_TEXT_MAGENTA
#define KCYN  RTT_CTRL_TEXT_CYAN
#define KWHT  RTT_CTRL_TEXT_WHITE
#else
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#endif

#define DEBUG_RTT               1
#define DEBUG_RAW               app_debug_rtt_raw


#define DEBUG_DUMP                           app_debug_dump

#if (DEBUG_LEVEL_VERBOSE > DEBUG_LEVEL)
#define DEBUG_VERBOSE(s, args...)               app_debug_rtt_raw(KMAG "<%u> [I] %s : " s KNRM,  sys_get_ms(), "", ##args)
#else
#define DEBUG_VERBOSE(s, args...)               app_debug_rtt_nothing(s, ##args)
#endif

#if (DEBUG_LEVEL_INFO > DEBUG_LEVEL)
#define DEBUG_INFO(s, args...)                  app_debug_rtt_raw(KGRN "<%u> [I] %s : " s KNRM,  sys_get_ms(), "", ##args)
#else
#define DEBUG_INFO(s, args...)                  app_debug_rtt_nothing(s, ##args)
#endif

#if (DEBUG_LEVEL_ERROR > DEBUG_LEVEL)
#define DEBUG_ERROR(s, args...)                 app_debug_rtt_raw(KRED "<%u> [E] %s : " s KNRM,  sys_get_ms(), "", ##args)
#else
#define DEBUG_ERROR(s, args...)                 app_debug_rtt_nothing(s, ##args)
#endif

#if (DEBUG_LEVEL_WARN > DEBUG_LEVEL)
#define DEBUG_WARN(s, args...)                  app_debug_rtt_raw(KYEL "<%u> [W] %s : " s KNRM,  sys_get_ms(), "", ##args)
#else
#define DEBUG_WARN(s, args...)                  app_debug_rtt_nothing(s, ##args)
#endif



#define DEBUG_COLOR(color, s, args...)       app_debug_rtt_raw(color s KNRM, ##args)


#define DEBUG_PRINTF            app_debug_rtt
#ifndef DEBUG_PRINTF
#define DEBUG_PRINTF(String...)	SEGGER_RTT_printf(0, String)
#endif

#ifndef DEBUG_FLUSH
#define DEBUG_FLUSH()      while(0)
#endif


typedef uint8_t(*p_func_write)(uint32_t index, const void*Buffer, uint32_t Len);

/**
 * @brief	 Add call back function to debug array
 *
 * Add your function to print data on screen, such as UartTransmit, CDCTransmit,...
 *
 * @param  p_func  Use p_function_write type: - Input(uint32_t,const void*,uint32_t). Return: uint8_t
 *              
 * @return None
 *     
 */
void app_debug_add_callback_print_function(p_func_write p_func);

/**
 * @brief	 Get timebase Value
 *
 * Get the current value of the timebase value. For example, Systick, TimerX Counter Value 
 *
 * @param  None
 *              
 * @return Current Counter Value of the TimeBase 
 *     
 */
extern uint32_t sys_get_ms(void);
/**
 * @brief	 Print nothing on screen
 *
 * Called when the DEBUG_FUNCTION level is less than the DEBUG_LEVEL value
 *
 * @param  fmt Pointer to the string want to print on the screen
 *              
 * @return Always -1
 *     
 */
int app_debug_rtt_nothing(const char *fmt,...);
/**
 * @brief	 Print input string on screen
 *
 * @param  fmt Pointer to the string want to print on the screen
 *              
 * @return Printed string Length
 *     
 */
int app_debug_rtt(const char *fmt,...);
/**
 * @brief	 Print input string on screen
 *
 * @param  fmt Pointer to the string want to print on the screen
 *              
 * @return Printed string Length
 *     
 */
int app_debug_rtt_raw(const char *fmt,...);

/**
 * @brief	 Print input string on screen
 *
 * @param  data Pointer to the string want to print on the screen
 *         len             
 * 
 * @return Printed string Length
 *     
 */
void app_debug_dump(const void* data, int len, const char* string, ...);




#endif // !DEBUG_H



