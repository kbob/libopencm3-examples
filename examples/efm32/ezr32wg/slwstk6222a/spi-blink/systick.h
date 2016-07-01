#ifndef SYSTICK_included
#define SYSTICK_included

#include <stdint.h>

extern volatile uint32_t millisecond_time;

extern void init_systick(void);

#endif /* !SYSTICK_included */
