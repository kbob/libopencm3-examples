#include <stddef.h>

#include "vcom.h"

#define BUF_SIZE 10
char buf[BUF_SIZE];

int main(void)
{
    VCOM_init();
    VCOM_init_stdio();

    VCOM_print_string("Hello, VCOM\n\n");

    while (1) {
        VCOM_print_string("prompt> ");
        size_t i = 0, j;
        while (1) {
            int c = VCOM_read_char(); // nonblocking, returns -1 until input
            if (c == '\r' || c == '\n') {
                VCOM_print_string("\r\n");
                break;
            }
            if (c < 32 || c >= '\177')
                continue;
            if (i < BUF_SIZE)
                buf[i++] = c;
            VCOM_print_char(c);
        }
        VCOM_print_string("echo:   ");
        for (j = 0; j < i; j++)
            VCOM_print_char(buf[j]);
        VCOM_print_string("\r\n");
    }
}
