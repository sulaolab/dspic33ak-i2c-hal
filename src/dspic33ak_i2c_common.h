#ifndef DSPIC33AK_I2C_COMMON_H
#define DSPIC33AK_I2C_COMMON_H

#include "dspic33ak_i2c.h"
#include "dspic33ak_i2c_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Internal shared helpers used by both the master and slave engines.
 *
 * These are intentionally pure (no module state):
 *   - inst_is_valid : range-check the instance enum
 *   - get_regs      : resolve an instance to its register pointer table
 *   - calc_brg      : compute the LBRG/HBRG divider from fcy/bus rates
 *
 * The master- and slave-specific state and engines live in their own
 * translation units and include this header to share these primitives.
 */

bool dspic33ak_i2c_inst_is_valid(dspic33ak_i2c_instance_t inst);

dspic33ak_i2c_status_t dspic33ak_i2c_get_regs(
    dspic33ak_i2c_instance_t inst,
    const dspic33ak_i2c_regs_t **regs);

uint32_t dspic33ak_i2c_calc_brg(uint32_t fcy_hz, uint32_t bus_hz);

#ifdef __cplusplus
}
#endif

#endif /* DSPIC33AK_I2C_COMMON_H */
