/*
 * cli.h
 *
 *  Created on: 6 Apr 2014
 *      Author: Hydra
 */

#ifndef CLI_H_
#define CLI_H_

extern uint8_t cliMode;

typedef union {
    int32_t int_value;
    float float_value;
} int_float_value_t;

typedef struct {
    const char *name;
    const uint8_t type; // vartype_e
    void *ptr;
    const int32_t min;
    const int32_t max;
} clivalue_t;

typedef enum {
    VAR_UINT8   = 0,
    VAR_INT8    = 1,
    VAR_UINT16  = 2,
    VAR_INT16   = 3,
    VAR_UINT32  = 4,
    VAR_FLOAT   = 5
} vartype_e;

extern const clivalue_t valueTable[];

void cliSetVar(const clivalue_t *var, const int_float_value_t value);

#endif /* CLI_H_ */
