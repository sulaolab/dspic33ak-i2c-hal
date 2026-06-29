#ifndef DSPIC33AK_I2C_H
#define DSPIC33AK_I2C_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Small, readable dsPIC33AK I2C driver - common types and lifecycle.
 *
 * This header carries only what the master and slave roles share:
 *   - the instance and status enumerations,
 *   - the millisecond-tick callback type,
 *   - presence / initialized queries and deinit.
 *
 * For the bus-master API include dspic33ak_i2c_master.h; for the slave API
 * include dspic33ak_i2c_slave.h. A program may include either or both. Both
 * pull in this header, so the shared types are always available.
 *
 * This header intentionally does not expose XC-DSC/DFP bitfield types.
 */

typedef enum {
    DSPIC33AK_I2C_INST_1 = 0,
    DSPIC33AK_I2C_INST_2,
    DSPIC33AK_I2C_INST_3,
    DSPIC33AK_I2C_INST_4,
    DSPIC33AK_I2C_INST_COUNT
} dspic33ak_i2c_instance_t;

typedef enum {
    DSPIC33AK_I2C_OK = 0,
    DSPIC33AK_I2C_ERR_INVALID_ARG,
    DSPIC33AK_I2C_ERR_NOT_PRESENT,
    DSPIC33AK_I2C_ERR_NOT_INITIALIZED,
    DSPIC33AK_I2C_ERR_BUSY,
    DSPIC33AK_I2C_ERR_TIMEOUT,
    DSPIC33AK_I2C_ERR_NACK,
    DSPIC33AK_I2C_ERR_BUS,
    DSPIC33AK_I2C_ERR_COLLISION,
    DSPIC33AK_I2C_ERR_UNSUPPORTED,
    DSPIC33AK_I2C_ERR_SEQUENCE
} dspic33ak_i2c_status_t;

typedef uint32_t (*dspic33ak_i2c_get_ms_fn)(void);

/* Shared lifecycle / query API -------------------------------------------- */

/*
 * Deinitialize the selected I2C instance (master or slave).
 *
 * If deinit recovers a stale pending transaction, it may return the recovery
 * status while still forcing the peripheral off and clearing HAL state.
 */
dspic33ak_i2c_status_t dspic33ak_i2c_deinit(
    dspic33ak_i2c_instance_t inst);

bool dspic33ak_i2c_is_present(
    dspic33ak_i2c_instance_t inst);

/*
 * Set the CPU interrupt priority symbols available for the selected I2C
 * instance on this device. Some instances expose only the event priority
 * symbol; others also expose dedicated RX/TX priority symbols. The application
 * still owns the vector functions; this hides the scattered _I2CxIP family.
 */
dspic33ak_i2c_status_t dspic33ak_i2c_set_interrupt_priority(
    dspic33ak_i2c_instance_t inst,
    uint8_t priority);

bool dspic33ak_i2c_is_initialized(
    dspic33ak_i2c_instance_t inst);

#ifdef __cplusplus
}
#endif

#endif /* DSPIC33AK_I2C_H */
