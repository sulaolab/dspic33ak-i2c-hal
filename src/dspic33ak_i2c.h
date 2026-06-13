#ifndef DSPIC33AK_I2C_H
#define DSPIC33AK_I2C_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Small, readable dsPIC33AK I2C driver interface.
 *
 * Design policy:
 *   - The normal user-facing API is blocking and easy to use.
 *   - Low-level functions separate "issue" and "status check" so advanced
 *     users can replace polling waits with interrupt flags or RTOS waits.
 *   - This header intentionally does not expose XC-DSC/DFP bitfield types.
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

typedef struct {
    uint32_t fcy_hz;
    uint32_t bus_hz;
    uint32_t timeout_ms;

    /*
     * Optional millisecond tick callback for timeout handling.
     * If get_ms is NULL, timeout handling is disabled.
     * If timeout_ms is 0, timeout handling is also disabled.
     *
     * pending_timeout_ms is independent from timeout_ms. If non-zero, a
     * no-STOP transaction left pending is recovered on the next public
     * transaction API call after this timeout has elapsed.
     */
    dspic33ak_i2c_get_ms_fn get_ms;
    uint32_t pending_timeout_ms;
} dspic33ak_i2c_config_t;

/* Normal blocking API ----------------------------------------------------- */

dspic33ak_i2c_status_t dspic33ak_i2c_init(
    dspic33ak_i2c_instance_t inst,
    const dspic33ak_i2c_config_t *config);

/*
 * Deinitialize the selected I2C instance.
 *
 * If deinit recovers a stale pending transaction, it may return the recovery
 * status while still forcing the peripheral off and clearing HAL state.
 */
dspic33ak_i2c_status_t dspic33ak_i2c_deinit(
    dspic33ak_i2c_instance_t inst);

bool dspic33ak_i2c_is_present(
    dspic33ak_i2c_instance_t inst);

bool dspic33ak_i2c_is_initialized(
    dspic33ak_i2c_instance_t inst);

/*
 * Update the I2C bus speed for an initialized idle instance.
 *
 * The selected instance must be initialized and idle. This API updates LBRG
 * and HBRG using the same BRG calculation as dspic33ak_i2c_init().
 *
 * Returns DSPIC33AK_I2C_ERR_BUSY if the host state machine is active or a
 * no-STOP transaction is pending.
 */
dspic33ak_i2c_status_t dspic33ak_i2c_set_bus_speed(
    dspic33ak_i2c_instance_t inst,
    uint32_t fcy_hz,
    uint32_t bus_hz);

dspic33ak_i2c_status_t dspic33ak_i2c_write(
    dspic33ak_i2c_instance_t inst,
    uint8_t addr7,
    const uint8_t *tx,
    size_t tx_len);

dspic33ak_i2c_status_t dspic33ak_i2c_read(
    dspic33ak_i2c_instance_t inst,
    uint8_t addr7,
    uint8_t *rx,
    size_t rx_len);

dspic33ak_i2c_status_t dspic33ak_i2c_write_read(
    dspic33ak_i2c_instance_t inst,
    uint8_t addr7,
    const uint8_t *tx,
    size_t tx_len,
    uint8_t *rx,
    size_t rx_len);

/* Master transaction API --------------------------------------------------
 * These functions expose the STOP-pending sequence needed by CMSIS-Driver
 * I2C xfer_pending style transfers.
 */

dspic33ak_i2c_status_t dspic33ak_i2c_master_write_no_stop(
    dspic33ak_i2c_instance_t inst,
    uint8_t addr7,
    const uint8_t *tx,
    size_t tx_len);

dspic33ak_i2c_status_t dspic33ak_i2c_master_read_after_restart(
    dspic33ak_i2c_instance_t inst,
    uint8_t addr7,
    uint8_t *rx,
    size_t rx_len);

dspic33ak_i2c_status_t dspic33ak_i2c_master_stop(
    dspic33ak_i2c_instance_t inst);

/* Low-level primitive API -------------------------------------------------
 * These functions are intentionally small. The blocking API above is built
 * from these issue/check operations.
 *
 * The pending transaction guard is applied to normal blocking and master
 * transaction APIs. Low-level primitive APIs do not enforce pending-state
 * sequencing.
 *
 * Normal application code should usually use dspic33ak_i2c_write(),
 * dspic33ak_i2c_read(), or dspic33ak_i2c_write_read().
 */

dspic33ak_i2c_status_t dspic33ak_i2c_ll_start_issue(
    dspic33ak_i2c_instance_t inst);
bool dspic33ak_i2c_ll_start_busy(dspic33ak_i2c_instance_t inst);
bool dspic33ak_i2c_ll_start_done(dspic33ak_i2c_instance_t inst);

/*
 * Repeated START primitive.
 *
 * A repeated START is a START condition generated without a preceding STOP.
 * The implementation uses the dsPIC33AK RSEN request bit and confirms
 * completion using RSEN clear plus STARTE set.
 */
dspic33ak_i2c_status_t dspic33ak_i2c_ll_restart_issue(
    dspic33ak_i2c_instance_t inst);
bool dspic33ak_i2c_ll_restart_busy(dspic33ak_i2c_instance_t inst);
bool dspic33ak_i2c_ll_restart_done(dspic33ak_i2c_instance_t inst);

dspic33ak_i2c_status_t dspic33ak_i2c_ll_stop_issue(
    dspic33ak_i2c_instance_t inst);
bool dspic33ak_i2c_ll_stop_busy(dspic33ak_i2c_instance_t inst);
bool dspic33ak_i2c_ll_stop_done(dspic33ak_i2c_instance_t inst);

dspic33ak_i2c_status_t dspic33ak_i2c_ll_write_byte_issue(
    dspic33ak_i2c_instance_t inst,
    uint8_t data);
bool dspic33ak_i2c_ll_write_byte_busy(dspic33ak_i2c_instance_t inst);
bool dspic33ak_i2c_ll_write_byte_acked(dspic33ak_i2c_instance_t inst);

dspic33ak_i2c_status_t dspic33ak_i2c_ll_read_byte_issue(
    dspic33ak_i2c_instance_t inst);
bool dspic33ak_i2c_ll_read_byte_ready(dspic33ak_i2c_instance_t inst);
dspic33ak_i2c_status_t dspic33ak_i2c_ll_read_byte_get(
    dspic33ak_i2c_instance_t inst,
    uint8_t *data);

dspic33ak_i2c_status_t dspic33ak_i2c_ll_ack_issue(
    dspic33ak_i2c_instance_t inst,
    bool ack);
bool dspic33ak_i2c_ll_ack_busy(dspic33ak_i2c_instance_t inst);

bool dspic33ak_i2c_ll_has_error(dspic33ak_i2c_instance_t inst);
bool dspic33ak_i2c_ll_has_nack(dspic33ak_i2c_instance_t inst);
bool dspic33ak_i2c_ll_has_collision(dspic33ak_i2c_instance_t inst);

/* Interrupt helper API ----------------------------------------------------
 * This is deliberately small. The driver does not force an interrupt-driven
 * transfer engine. Users may call these helpers from their own ISR design.
 *
 * Current sandbox implementation returns DSPIC33AK_I2C_ERR_UNSUPPORTED.
 * These functions are reserved for a future small interrupt helper layer.
 */

#define DSPIC33AK_I2C_IRQ_TRANSFER_DONE   (1u << 0)
#define DSPIC33AK_I2C_IRQ_ERROR           (1u << 1)
#define DSPIC33AK_I2C_IRQ_BUS_COLLISION   (1u << 2)
#define DSPIC33AK_I2C_IRQ_ALL             (DSPIC33AK_I2C_IRQ_TRANSFER_DONE | \
                                           DSPIC33AK_I2C_IRQ_ERROR | \
                                           DSPIC33AK_I2C_IRQ_BUS_COLLISION)

dspic33ak_i2c_status_t dspic33ak_i2c_irq_enable(
    dspic33ak_i2c_instance_t inst,
    uint32_t irq_mask);

dspic33ak_i2c_status_t dspic33ak_i2c_irq_disable(
    dspic33ak_i2c_instance_t inst,
    uint32_t irq_mask);

dspic33ak_i2c_status_t dspic33ak_i2c_irq_clear(
    dspic33ak_i2c_instance_t inst,
    uint32_t irq_mask);

#ifdef __cplusplus
}
#endif

#endif /* DSPIC33AK_I2C_H */
