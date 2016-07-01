#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "vcom.h"

#define BUF_SIZE 10
char buf[BUF_SIZE];

int main(void)
{
    VCOM_init();
    VCOM_init_stdio();

    printf("\n\n");
    printf("Hello, VCOM stdio!\n\n");

    while (1) {
        printf("prompt> ");
        fflush(stdout);
        size_t i = 0, j;
        while (1) {
            int c = getchar();
            if (c == '\r' || c == '\n') {
                printf("\r\n");
                break;
            }
            if (c < 32 || c >= '\177')
                continue;
            if (i < BUF_SIZE)
                buf[i++] = c;
            putchar(c);
            fflush(stdout);
       }
        printf("echo:   ");
        for (j = 0; j < i; j++)
            putchar(buf[j]);
        putchar('\n');
    }
}
