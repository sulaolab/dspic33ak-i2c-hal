#ifndef DSPIC33AK_I2C_DEVICE_H
#define DSPIC33AK_I2C_DEVICE_H

#include <stdbool.h>
#include "dspic33ak_i2c.h"
#include "dspic33ak_i2c_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool present;
    dspic33ak_i2c_regs_t regs;
} dspic33ak_i2c_device_t;

const dspic33ak_i2c_device_t *dspic33ak_i2c_get_device(
    dspic33ak_i2c_instance_t inst);

bool dspic33ak_i2c_instance_is_present(dspic33ak_i2c_instance_t inst);

#ifdef __cplusplus
}
#endif

#endif /* DSPIC33AK_I2C_DEVICE_H */
