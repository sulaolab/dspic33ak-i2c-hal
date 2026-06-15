#include "dspic33ak_i2c.h"
#include "dspic33ak_i2c_device.h"
#include "dspic33ak_i2c_reg.h"
#include "dspic33ak_i2c_common.h"

/* --------------------------------------------------------------------------
 * Shared primitives used by both the master and slave engines.
 *
 * Nothing here keeps module state; per-instance state (initialized flag,
 * timeout configuration, pending-transaction tracking, slave callbacks)
 * lives in the master and slave translation units.
 * -------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------
 * Validate instance number
 * -------------------------------------------------------------------------- */
bool dspic33ak_i2c_inst_is_valid(dspic33ak_i2c_instance_t inst)
{
    return ((unsigned)inst < (unsigned)DSPIC33AK_I2C_INST_COUNT);
}

/* --------------------------------------------------------------------------
 * Resolve instance to register table
 * -------------------------------------------------------------------------- */
dspic33ak_i2c_status_t dspic33ak_i2c_get_regs(
    dspic33ak_i2c_instance_t inst,
    const dspic33ak_i2c_regs_t **regs)
{
    const dspic33ak_i2c_device_t *dev;

    if (regs == 0) {
        return DSPIC33AK_I2C_ERR_INVALID_ARG;
    }

    if (!dspic33ak_i2c_inst_is_valid(inst)) {
        return DSPIC33AK_I2C_ERR_INVALID_ARG;
    }

    dev = dspic33ak_i2c_get_device(inst);
    if (dev == 0) {
        return DSPIC33AK_I2C_ERR_NOT_PRESENT;
    }

    *regs = &dev->regs;
    return DSPIC33AK_I2C_OK;
}

/* --------------------------------------------------------------------------
 * Calculate BRG value
 * -------------------------------------------------------------------------- */
uint32_t dspic33ak_i2c_calc_brg(uint32_t fcy_hz, uint32_t bus_hz)
{
    uint64_t div;

    if (bus_hz == 0u) {
        return 0u;
    }

    /*
     * Round to the nearest divider while using 64-bit arithmetic to avoid
     * uint32_t overflow in fcy_hz + bus_hz on future faster devices.
     */
    div = ((uint64_t)fcy_hz + (uint64_t)bus_hz) /
          (2ull * (uint64_t)bus_hz);

    if (div == 0ull) {
        return 0u;
    }

    return (uint32_t)(div - 1ull);
}

/* --------------------------------------------------------------------------
 * Check whether I2C instance exists on the selected device
 * -------------------------------------------------------------------------- */
bool dspic33ak_i2c_is_present(dspic33ak_i2c_instance_t inst)
{
    return dspic33ak_i2c_instance_is_present(inst);
}
