#include "bsp_uart.h"

#include <stdarg.h>
#include <stdio.h>

int printf(const char *format, ...)
{
    char message[128];
    va_list args;
    va_start(args, format);
    const int formatted = vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    if (formatted < 0) return formatted;
    if (formatted == 0) return 0;
    size_t length = (size_t)formatted;
    if (length >= sizeof(message)) length = sizeof(message) - 1U;
    const int ret = bsp_uart_stdio_write((const uint8_t *)message, length);
    return ret == 0 ? (int)length : ret;
}
