#include "app_debug.h"
#include <stdarg.h>
#include "stdio.h"
#include <string.h>
#include <ctype.h>
#include "main.h"

#define NUMBER_OF_DEBUG_PORT    4
uint8_t m_is_usb_init = 0;

p_func_write p_write_function[NUMBER_OF_DEBUG_PORT];

uint32_t sys_get_ms()
{
    return HAL_GetTick();
}



uint8_t p_callback_idx = 0;

//#define SEGGER_RTT_PRINTF_BUFFER_SIZE 256
static char m_debug_buffer[SEGGER_RTT_PRINTF_BUFFER_SIZE];

extern uint32_t sys_get_ms(void);

int app_debug_rtt_nothing(const char *fmt,...)
{
    return -1;
}

int app_debug_rtt(const char *fmt,...)
{
    int     n;

    char *p = &m_debug_buffer[0];
    int size = SEGGER_RTT_PRINTF_BUFFER_SIZE;
    int time_stamp_size;

    p += sprintf(m_debug_buffer, "<%u>: ", sys_get_ms());
    time_stamp_size = (p-m_debug_buffer);
    size -= time_stamp_size;
    va_list args;

    va_start (args, fmt);
    n = vsnprintf(p, size, fmt, args);
    if (n > (int)size) 
    {
        
           // p_write_function[0](0,m_debug_buffer,n + time_stamp_size);
        //SEGGER_RTT_Write(0, m_debug_buffer, n + time_stamp_size);
        for (uint8_t index = 0; index < NUMBER_OF_DEBUG_PORT ; index++)
        {
            if(p_write_function[index] != NULL)
            {
                p_write_function[index](0, m_debug_buffer, n + time_stamp_size);
            }
        }
        
    } 
    else if (n > 0) 
    {
        //if(m_is_usb_init)
            //p_write_function[0](0,m_debug_buffer,n + time_stamp_size);
        //SEGGER_RTT_Write(0, m_debug_buffer, n + time_stamp_size);
        for (uint8_t index = 0; index < NUMBER_OF_DEBUG_PORT ; index++)
        {
            if(p_write_function[index] != NULL)
            {
                p_write_function[index](0, m_debug_buffer, n + time_stamp_size);
            }
        }

    }
    va_end(args);
    
    return n;
}

int app_debug_rtt_raw(const char *fmt,...)
{
    int     n;

    char *p = &m_debug_buffer[0];
    int size = SEGGER_RTT_PRINTF_BUFFER_SIZE;
    va_list args;

    va_start (args, fmt);
    n = vsnprintf(p, size, fmt, args);
    if (n > (int)size) 
    {

        //if(m_is_usb_init)
          //  p_write_function(0,m_debug_buffer,size);
       // SEGGER_RTT_Write(0, m_debug_buffer, size);
        for (uint8_t index = 0; index < NUMBER_OF_DEBUG_PORT ; index++)
        {
            if(p_write_function[index] != NULL)
            {
                p_write_function[index](0, m_debug_buffer, size);
            }
        }

    } 
    else if (n > 0) 
    {
        //if(m_is_usb_init)
            //p_write_function(0,m_debug_buffer,n);
        //SEGGER_RTT_Write(0, m_debug_buffer, n);
        for (uint8_t index = 0; index < NUMBER_OF_DEBUG_PORT ; index++)
        {
            if(p_write_function[index] != NULL)
            {
                p_write_function[index](0, m_debug_buffer, n );
            }
        }

    }
    va_end(args);
    
    return n;
}





void app_debug_dump(const void* data, int len, const char* string, ...)
{
	uint8_t* p = (uint8_t*)data;
    uint8_t  buffer[16];
    int32_t i_len;
    int32_t i;

//    DEBUG_RAW("%s %u bytes\n", string, len);
    while (len > 0)
    {
        i_len = (len > 16) ? 16 : len;
        memset(buffer, 0, 16);
        memcpy(buffer, p, i_len);
        for (i = 0; i < 16; i++)
        {
            if (i < i_len)
                DEBUG_RAW("%02X ", buffer[i]);
            else
                DEBUG_RAW("   ");
        }
        DEBUG_RAW("\t");
        for (i = 0; i < 16; i++)
        {
            if (i < i_len)
            {
                if (isprint(buffer[i]))
				{
                    DEBUG_RAW("%c", (char)buffer[i]);
				}
                else
				{
                    DEBUG_RAW(".");
				}
            }
            else
                DEBUG_RAW(" ");
        }
        DEBUG_RAW("\r\n");
        len -= i_len;
        p += i_len;
    }
}



void app_debug_add_callback_print_function(p_func_write p_func)
{
    uint8_t p_func_exist = 0;                   // Check for existion function pointer in function pointer arry
    if(p_func != NULL)
    {
        for (uint8_t func_count = 0; func_count < NUMBER_OF_DEBUG_PORT; func_count++)
        {
            if(p_func == p_write_function[func_count])
            {
                // Function pointer existed in array
                p_func_exist = 1;           
            }
        }
        // Add function pointer 
        if(!p_func_exist)                    
        {
            p_write_function[p_callback_idx] = p_func;
            p_callback_idx++;
        }
    }
}
/************************************** END OF FILE **************************************/








