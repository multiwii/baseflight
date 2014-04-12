#include <sys/unistd.h>
#include "stm32f10x_usart.h"

/*
 * sbrk
 * Increase program data space.
 */
caddr_t _sbrk(int incr)
{
    extern char _ebss; // Defined by the linker
    static char *heap;
    char *prevHeap;

    if (heap == 0)
        heap = &_ebss;

    prevHeap = heap;

    char * stack = (char*)__get_MSP();
    if (heap + incr > stack) {
        return (caddr_t)-1;
    }

    heap += incr;
    return (caddr_t)prevHeap;
}
