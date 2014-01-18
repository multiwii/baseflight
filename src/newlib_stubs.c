/*
 * newlib_stubs.c
 *
 *  Created on: 2 Nov 2010
 *      Author: nanoage.co.uk
 */

#include "errno.h"
#include "board.h"

caddr_t _sbrk(int incr)
{
    extern char _ebss; // Defined by the linker
    static char *heap_end;
    char *prev_heap_end;

    char * stack;

    if (heap_end == 0)
        heap_end = &_ebss;

    prev_heap_end = heap_end;

    stack = (char*) __get_MSP();
    if (heap_end + incr >  stack) {
        errno = ENOMEM;
        return  (caddr_t) -1;
    }

    heap_end += incr;
    return (caddr_t) prev_heap_end;
}
