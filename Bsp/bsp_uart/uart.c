#include "bsp_uart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

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

int puts(const char *string)
{
    char message[128];
    size_t length = strlen(string);
    if (length > sizeof(message) - 2U) length = sizeof(message) - 2U;
    memcpy(message, string, length);
    if (length == 0U || message[length - 1U] != '\n') {
        if (length == 0U || message[length - 1U] != '\r') {
            message[length++] = '\r';
        }
        message[length++] = '\n';
    }

    const int ret = bsp_uart_stdio_write((const uint8_t *)message, length);
    return ret == 0 ? 0 : EOF;
}
