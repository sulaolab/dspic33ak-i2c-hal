#include "dspic33ak_i2c.h"
#include "dspic33ak_i2c_device.h"
#include "dspic33ak_i2c_reg.h"

/* --------------------------------------------------------------------------
 * Module state
 * -------------------------------------------------------------------------- */

static uint32_t g_timeout_ms[DSPIC33AK_I2C_INST_COUNT];
static dspic33ak_i2c_get_ms_fn g_get_ms[DSPIC33AK_I2C_INST_COUNT];
static bool g_initialized[DSPIC33AK_I2C_INST_COUNT];

/* --------------------------------------------------------------------------
 * Local helper prototypes
 *
 * The implementation body is ordered for readability:
 *   1. Normal blocking API
 *   2. Low-level global API
 *   3. Static local helper functions
 * -------------------------------------------------------------------------- */

static bool inst_is_valid(dspic33ak_i2c_instance_t inst);
static dspic33ak_i2c_status_t get_regs(
    dspic33ak_i2c_instance_t inst,
    const dspic33ak_i2c_regs_t **regs);
static dspic33ak_i2c_status_t require_initialized(
    dspic33ak_i2c_instance_t inst,
    const dspic33ak_i2c_regs_t **regs);
static dspic33ak_i2c_status_t check_initialized(dspic33ak_i2c_instance_t inst);
static uint32_t calc_brg(uint32_t fcy_hz, uint32_t bus_hz);
static bool timeout_enabled(dspic33ak_i2c_instance_t inst);
static uint32_t timeout_start_ms(dspic33ak_i2c_instance_t inst);
static bool timeout_expired(dspic33ak_i2c_instance_t inst, uint32_t start_ms);
static void clear_transfer_status(const dspic33ak_i2c_regs_t *r);
static dspic33ak_i2c_status_t check_bus_fault(dspic33ak_i2c_instance_t inst);
static dspic33ak_i2c_status_t wait_until(
    dspic33ak_i2c_instance_t inst,
    bool (*done_fn)(dspic33ak_i2c_instance_t),
    bool check_nack);
static bool host_active(dspic33ak_i2c_instance_t inst);
static bool write_byte_ready(dspic33ak_i2c_instance_t inst);
static bool write_byte_done(dspic33ak_i2c_instance_t inst);
static bool write_data_accepted(dspic33ak_i2c_instance_t inst);
static bool ack_done(dspic33ak_i2c_instance_t inst);
static dspic33ak_i2c_status_t stop_quiet(dspic33ak_i2c_instance_t inst);
static dspic33ak_i2c_status_t start_blocking(dspic33ak_i2c_instance_t inst);
static dspic33ak_i2c_status_t restart_blocking(dspic33ak_i2c_instance_t inst);
static dspic33ak_i2c_status_t write_byte_blocking(
    dspic33ak_i2c_instance_t inst,
    uint8_t data);
static dspic33ak_i2c_status_t read_byte_blocking(
    dspic33ak_i2c_instance_t inst,
    uint8_t *data,
    bool ack);
static dspic33ak_i2c_status_t send_address_blocking(
    dspic33ak_i2c_instance_t inst,
    uint8_t addr7,
    bool read);

/* ========================================================================== */
/* 1. Normal blocking API                                                     */
/* ========================================================================== */

/* --------------------------------------------------------------------------
 * Initialize I2C instance
 * -------------------------------------------------------------------------- */
dspic33ak_i2c_status_t dspic33ak_i2c_init(
    dspic33ak_i2c_instance_t inst,
    const dspic33ak_i2c_config_t *config)
{
    const dspic33ak_i2c_regs_t *r;
    dspic33ak_i2c_status_t st;
    uint32_t brg;

    if (!inst_is_valid(inst)) {
        return DSPIC33AK_I2C_ERR_INVALID_ARG;
    }

    if (config == 0 || config->fcy_hz == 0u || config->bus_hz == 0u) {
        return DSPIC33AK_I2C_ERR_INVALID_ARG;
    }

    st = get_regs(inst, &r);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    *r->CON1 = 0x00001000u;
    *r->CON2 = 0x00000001u;
    clear_transfer_status(r);

    dspic33ak_i2c_reg_set(r->CON2, DSPIC33AK_I2C_CON2_BITE);
    dspic33ak_i2c_reg_write_field(r->BITO,
                                   DSPIC33AK_I2C_BITO_BITOTMR_MASK,
                                   76u);

    brg = calc_brg(config->fcy_hz, config->bus_hz);

    /* LBRG and HBRG are equal for a simple 50% duty-cycle SCL setup. */
    *r->LBRG = brg;
    *r->HBRG = brg;

    g_timeout_ms[inst] = config->timeout_ms;
    g_get_ms[inst] = config->get_ms;
    g_initialized[inst] = true;

    dspic33ak_i2c_reg_set(r->CON1, DSPIC33AK_I2C_CON1_ON);

    return DSPIC33AK_I2C_OK;
}

/* --------------------------------------------------------------------------
 * Deinitialize I2C instance
 * -------------------------------------------------------------------------- */
dspic33ak_i2c_status_t dspic33ak_i2c_deinit(
    dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;
    dspic33ak_i2c_status_t st;

    if (!inst_is_valid(inst)) {
        return DSPIC33AK_I2C_ERR_INVALID_ARG;
    }

    st = get_regs(inst, &r);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    if (dspic33ak_i2c_reg_is_set(r->STAT2, DSPIC33AK_I2C_STAT2_HSTACT) ||
        dspic33ak_i2c_reg_is_set(r->STAT1, DSPIC33AK_I2C_STAT1_TRSTAT) ||
        dspic33ak_i2c_reg_is_set(r->CON1,
                                  DSPIC33AK_I2C_CON1_SEN |
                                  DSPIC33AK_I2C_CON1_RSEN |
                                  DSPIC33AK_I2C_CON1_PEN |
                                  DSPIC33AK_I2C_CON1_RCEN |
                                  DSPIC33AK_I2C_CON1_ACKEN)) {
        return DSPIC33AK_I2C_ERR_BUSY;
    }

    dspic33ak_i2c_reg_clear(r->CON1, DSPIC33AK_I2C_CON1_ON);

    g_timeout_ms[inst] = 0u;
    g_get_ms[inst] = 0;
    g_initialized[inst] = false;

    return DSPIC33AK_I2C_OK;
}

/* --------------------------------------------------------------------------
 * Check whether I2C instance exists on the selected device
 * -------------------------------------------------------------------------- */
bool dspic33ak_i2c_is_present(dspic33ak_i2c_instance_t inst)
{
    return dspic33ak_i2c_instance_is_present(inst);
}

/* --------------------------------------------------------------------------
 * Check whether I2C instance has been initialized
 * -------------------------------------------------------------------------- */
bool dspic33ak_i2c_is_initialized(dspic33ak_i2c_instance_t inst)
{
    if (!inst_is_valid(inst)) {
        return false;
    }

    return g_initialized[inst];
}

/* --------------------------------------------------------------------------
 * Blocking write transaction
 * -------------------------------------------------------------------------- */
dspic33ak_i2c_status_t dspic33ak_i2c_write(
    dspic33ak_i2c_instance_t inst,
    uint8_t addr7,
    const uint8_t *tx,
    size_t tx_len)
{
    dspic33ak_i2c_status_t st;
    size_t i;

    if (tx_len != 0u && tx == 0) {
        return DSPIC33AK_I2C_ERR_INVALID_ARG;
    }

    st = check_initialized(inst);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    st = start_blocking(inst);
    if (st != DSPIC33AK_I2C_OK) {
        (void)stop_quiet(inst);
        return st;
    }

    st = send_address_blocking(inst, addr7, false);
    if (st != DSPIC33AK_I2C_OK) {
        (void)stop_quiet(inst);
        return st;
    }

    for (i = 0u; i < tx_len; i++) {
        st = write_byte_blocking(inst, tx[i]);
        if (st != DSPIC33AK_I2C_OK) {
            (void)stop_quiet(inst);
            return st;
        }
    }

    return stop_quiet(inst);
}

/* --------------------------------------------------------------------------
 * Blocking read transaction
 * -------------------------------------------------------------------------- */
dspic33ak_i2c_status_t dspic33ak_i2c_read(
    dspic33ak_i2c_instance_t inst,
    uint8_t addr7,
    uint8_t *rx,
    size_t rx_len)
{
    dspic33ak_i2c_status_t st;
    size_t i;

    if (rx == 0 || rx_len == 0u) {
        return DSPIC33AK_I2C_ERR_INVALID_ARG;
    }

    st = check_initialized(inst);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    st = start_blocking(inst);
    if (st != DSPIC33AK_I2C_OK) {
        (void)stop_quiet(inst);
        return st;
    }

    st = send_address_blocking(inst, addr7, true);
    if (st != DSPIC33AK_I2C_OK) {
        (void)stop_quiet(inst);
        return st;
    }

    for (i = 0u; i < rx_len; i++) {
        bool ack = ((i + 1u) < rx_len);
        st = read_byte_blocking(inst, &rx[i], ack);
        if (st != DSPIC33AK_I2C_OK) {
            (void)stop_quiet(inst);
            return st;
        }
    }

    return stop_quiet(inst);
}

/* --------------------------------------------------------------------------
 * Blocking write-read transaction with repeated START
 * -------------------------------------------------------------------------- */
dspic33ak_i2c_status_t dspic33ak_i2c_write_read(
    dspic33ak_i2c_instance_t inst,
    uint8_t addr7,
    const uint8_t *tx,
    size_t tx_len,
    uint8_t *rx,
    size_t rx_len)
{
    dspic33ak_i2c_status_t st;
    size_t i;

    if ((tx_len != 0u && tx == 0) || rx == 0 || rx_len == 0u) {
        return DSPIC33AK_I2C_ERR_INVALID_ARG;
    }

    st = check_initialized(inst);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    st = start_blocking(inst);
    if (st != DSPIC33AK_I2C_OK) {
        (void)stop_quiet(inst);
        return st;
    }

    st = send_address_blocking(inst, addr7, false);
    if (st != DSPIC33AK_I2C_OK) {
        (void)stop_quiet(inst);
        return st;
    }

    for (i = 0u; i < tx_len; i++) {
        st = write_byte_blocking(inst, tx[i]);
        if (st != DSPIC33AK_I2C_OK) {
            (void)stop_quiet(inst);
            return st;
        }
    }

    st = restart_blocking(inst);
    if (st != DSPIC33AK_I2C_OK) {
        (void)stop_quiet(inst);
        return st;
    }

    st = send_address_blocking(inst, addr7, true);
    if (st != DSPIC33AK_I2C_OK) {
        (void)stop_quiet(inst);
        return st;
    }

    for (i = 0u; i < rx_len; i++) {
        bool ack = ((i + 1u) < rx_len);
        st = read_byte_blocking(inst, &rx[i], ack);
        if (st != DSPIC33AK_I2C_OK) {
            (void)stop_quiet(inst);
            return st;
        }
    }

    return stop_quiet(inst);
}

/* ========================================================================== */
/* 2. Low-level global API                                                    */
/* ========================================================================== */

/* --------------------------------------------------------------------------
 * Issue START condition
 * -------------------------------------------------------------------------- */
dspic33ak_i2c_status_t dspic33ak_i2c_ll_start_issue(
    dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;
    dspic33ak_i2c_status_t st = require_initialized(inst, &r);

    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    clear_transfer_status(r);
    dspic33ak_i2c_reg_set(r->CON1, DSPIC33AK_I2C_CON1_SEN);
    return DSPIC33AK_I2C_OK;
}

/* --------------------------------------------------------------------------
 * Check START busy flag
 * -------------------------------------------------------------------------- */
bool dspic33ak_i2c_ll_start_busy(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;

    if (get_regs(inst, &r) != DSPIC33AK_I2C_OK) {
        return false;
    }

    return dspic33ak_i2c_reg_is_set(r->CON1, DSPIC33AK_I2C_CON1_SEN);
}

/* --------------------------------------------------------------------------
 * Check START done status
 * -------------------------------------------------------------------------- */
bool dspic33ak_i2c_ll_start_done(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;

    if (get_regs(inst, &r) != DSPIC33AK_I2C_OK) {
        return false;
    }

    return dspic33ak_i2c_reg_is_set(r->STAT2, DSPIC33AK_I2C_STAT2_STARTE) ||
           dspic33ak_i2c_reg_is_set(r->STAT1, DSPIC33AK_I2C_STAT1_S);
}

/* --------------------------------------------------------------------------
 * Issue repeated START condition
 * -------------------------------------------------------------------------- */
dspic33ak_i2c_status_t dspic33ak_i2c_ll_restart_issue(
    dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;
    dspic33ak_i2c_status_t st = require_initialized(inst, &r);

    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    /*
     * Issue a repeated START condition.
     *
     * A repeated START is a START condition generated without a preceding
     * STOP.  Use RSEN so this low-level API maps directly to the dsPIC33AK
     * repeated-START request bit.
     */
    dspic33ak_i2c_reg_clear(r->STAT2, DSPIC33AK_I2C_STAT2_STARTE);
    dspic33ak_i2c_reg_set(r->CON1, DSPIC33AK_I2C_CON1_RSEN);
    return DSPIC33AK_I2C_OK;
}

/* --------------------------------------------------------------------------
 * Check repeated START busy flag
 * -------------------------------------------------------------------------- */
bool dspic33ak_i2c_ll_restart_busy(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;

    if (get_regs(inst, &r) != DSPIC33AK_I2C_OK) {
        return false;
    }

    return dspic33ak_i2c_reg_is_set(r->CON1, DSPIC33AK_I2C_CON1_RSEN);
}

/* --------------------------------------------------------------------------
 * Check repeated START done status
 * -------------------------------------------------------------------------- */
bool dspic33ak_i2c_ll_restart_done(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;

    if (get_regs(inst, &r) != DSPIC33AK_I2C_OK) {
        return false;
    }

    /*
     * STARTE is cleared before issuing RSEN, then checked again here.
     * STAT1.S alone is not sufficient for repeated START completion because
     * it may already be set by the previous START condition.
     */
    return !dspic33ak_i2c_reg_is_set(r->CON1, DSPIC33AK_I2C_CON1_RSEN) &&
            dspic33ak_i2c_reg_is_set(r->STAT2, DSPIC33AK_I2C_STAT2_STARTE);
}

/* --------------------------------------------------------------------------
 * Issue STOP condition
 * -------------------------------------------------------------------------- */
dspic33ak_i2c_status_t dspic33ak_i2c_ll_stop_issue(
    dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;
    dspic33ak_i2c_status_t st = require_initialized(inst, &r);

    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    dspic33ak_i2c_reg_clear(r->STAT2, DSPIC33AK_I2C_STAT2_STOPE);
    dspic33ak_i2c_reg_set(r->CON1, DSPIC33AK_I2C_CON1_PEN);
    return DSPIC33AK_I2C_OK;
}

/* --------------------------------------------------------------------------
 * Check STOP busy flag
 * -------------------------------------------------------------------------- */
bool dspic33ak_i2c_ll_stop_busy(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;

    if (get_regs(inst, &r) != DSPIC33AK_I2C_OK) {
        return false;
    }

    return dspic33ak_i2c_reg_is_set(r->CON1, DSPIC33AK_I2C_CON1_PEN);
}

/* --------------------------------------------------------------------------
 * Check STOP done status
 * -------------------------------------------------------------------------- */
bool dspic33ak_i2c_ll_stop_done(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;

    if (get_regs(inst, &r) != DSPIC33AK_I2C_OK) {
        return false;
    }

    return dspic33ak_i2c_reg_is_set(r->STAT2, DSPIC33AK_I2C_STAT2_STOPE);
}

/* --------------------------------------------------------------------------
 * Issue one-byte write
 * -------------------------------------------------------------------------- */
dspic33ak_i2c_status_t dspic33ak_i2c_ll_write_byte_issue(
    dspic33ak_i2c_instance_t inst,
    uint8_t data)
{
    const dspic33ak_i2c_regs_t *r;
    dspic33ak_i2c_status_t st = require_initialized(inst, &r);

    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    if (dspic33ak_i2c_reg_is_set(r->STAT1, DSPIC33AK_I2C_STAT1_IWCOL)) {
        dspic33ak_i2c_reg_clear(r->STAT1, DSPIC33AK_I2C_STAT1_IWCOL);
    }

    dspic33ak_i2c_reg_clear(r->STAT2, DSPIC33AK_I2C_STAT2_NACKE);
    *r->TRN = data;
    return DSPIC33AK_I2C_OK;
}

/* --------------------------------------------------------------------------
 * Check one-byte write busy flag
 * -------------------------------------------------------------------------- */
bool dspic33ak_i2c_ll_write_byte_busy(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;

    if (get_regs(inst, &r) != DSPIC33AK_I2C_OK) {
        return false;
    }

    return dspic33ak_i2c_reg_is_set(r->STAT1, DSPIC33AK_I2C_STAT1_TRSTAT);
}

/* --------------------------------------------------------------------------
 * Check ACK result after one-byte write
 * -------------------------------------------------------------------------- */
bool dspic33ak_i2c_ll_write_byte_acked(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;

    if (get_regs(inst, &r) != DSPIC33AK_I2C_OK) {
        return false;
    }

    /*
     * ACK completion is validated by the byte-phase state checks in
     * write_byte_blocking().  Avoid using ACKSTAT here because this driver
     * uses NACKE and D_A as the observable transfer outcome.
     */
    if (dspic33ak_i2c_reg_is_set(r->STAT2, DSPIC33AK_I2C_STAT2_NACKE)) {
        return false;
    }

    return true;
}

/* --------------------------------------------------------------------------
 * Issue one-byte read
 * -------------------------------------------------------------------------- */
dspic33ak_i2c_status_t dspic33ak_i2c_ll_read_byte_issue(
    dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;
    dspic33ak_i2c_status_t st = require_initialized(inst, &r);

    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    dspic33ak_i2c_reg_set(r->CON1, DSPIC33AK_I2C_CON1_RCEN);
    return DSPIC33AK_I2C_OK;
}

/* --------------------------------------------------------------------------
 * Check receive buffer ready
 * -------------------------------------------------------------------------- */
bool dspic33ak_i2c_ll_read_byte_ready(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;

    if (get_regs(inst, &r) != DSPIC33AK_I2C_OK) {
        return false;
    }

    return dspic33ak_i2c_reg_is_set(r->STAT1, DSPIC33AK_I2C_STAT1_RBF);
}

/* --------------------------------------------------------------------------
 * Get received byte
 * -------------------------------------------------------------------------- */
dspic33ak_i2c_status_t dspic33ak_i2c_ll_read_byte_get(
    dspic33ak_i2c_instance_t inst,
    uint8_t *data)
{
    const dspic33ak_i2c_regs_t *r;
    dspic33ak_i2c_status_t st;

    if (data == 0) {
        return DSPIC33AK_I2C_ERR_INVALID_ARG;
    }

    st = get_regs(inst, &r);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    *data = (uint8_t)(*r->RCV & 0xFFu);
    return DSPIC33AK_I2C_OK;
}

/* --------------------------------------------------------------------------
 * Issue ACK or NACK after read
 * -------------------------------------------------------------------------- */
dspic33ak_i2c_status_t dspic33ak_i2c_ll_ack_issue(
    dspic33ak_i2c_instance_t inst,
    bool ack)
{
    const dspic33ak_i2c_regs_t *r;
    dspic33ak_i2c_status_t st = require_initialized(inst, &r);

    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    if (ack) {
        dspic33ak_i2c_reg_clear(r->CON1, DSPIC33AK_I2C_CON1_ACKDT);
    } else {
        dspic33ak_i2c_reg_set(r->CON1, DSPIC33AK_I2C_CON1_ACKDT);
    }

    dspic33ak_i2c_reg_set(r->CON1, DSPIC33AK_I2C_CON1_ACKEN);
    return DSPIC33AK_I2C_OK;
}

/* --------------------------------------------------------------------------
 * Check ACK/NACK issue busy flag
 * -------------------------------------------------------------------------- */
bool dspic33ak_i2c_ll_ack_busy(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;

    if (get_regs(inst, &r) != DSPIC33AK_I2C_OK) {
        return false;
    }

    return dspic33ak_i2c_reg_is_set(r->CON1, DSPIC33AK_I2C_CON1_ACKEN);
}

/* --------------------------------------------------------------------------
 * Check I2C error status
 * -------------------------------------------------------------------------- */
bool dspic33ak_i2c_ll_has_error(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;

    if (get_regs(inst, &r) != DSPIC33AK_I2C_OK) {
        return true;
    }

    return dspic33ak_i2c_reg_is_set(r->STAT2, DSPIC33AK_I2C_STAT2_ERR);
}

/* --------------------------------------------------------------------------
 * Check NACK status
 * -------------------------------------------------------------------------- */
bool dspic33ak_i2c_ll_has_nack(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;

    if (get_regs(inst, &r) != DSPIC33AK_I2C_OK) {
        return true;
    }

    return dspic33ak_i2c_reg_is_set(r->STAT2, DSPIC33AK_I2C_STAT2_NACKE);
}

/* --------------------------------------------------------------------------
 * Check bus collision status
 * -------------------------------------------------------------------------- */
bool dspic33ak_i2c_ll_has_collision(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;

    if (get_regs(inst, &r) != DSPIC33AK_I2C_OK) {
        return true;
    }

    return dspic33ak_i2c_reg_is_set(r->STAT1,
                                    DSPIC33AK_I2C_STAT1_IWCOL |
                                    DSPIC33AK_I2C_STAT1_BCL);
}

/* --------------------------------------------------------------------------
 * Enable selected I2C interrupts - future helper
 * -------------------------------------------------------------------------- */
dspic33ak_i2c_status_t dspic33ak_i2c_irq_enable(
    dspic33ak_i2c_instance_t inst,
    uint32_t irq_mask)
{
    (void)inst;
    (void)irq_mask;
    return DSPIC33AK_I2C_ERR_UNSUPPORTED;
}

/* --------------------------------------------------------------------------
 * Disable selected I2C interrupts - future helper
 * -------------------------------------------------------------------------- */
dspic33ak_i2c_status_t dspic33ak_i2c_irq_disable(
    dspic33ak_i2c_instance_t inst,
    uint32_t irq_mask)
{
    (void)inst;
    (void)irq_mask;
    return DSPIC33AK_I2C_ERR_UNSUPPORTED;
}

/* --------------------------------------------------------------------------
 * Clear selected I2C interrupt flags - future helper
 * -------------------------------------------------------------------------- */
dspic33ak_i2c_status_t dspic33ak_i2c_irq_clear(
    dspic33ak_i2c_instance_t inst,
    uint32_t irq_mask)
{
    (void)inst;
    (void)irq_mask;
    return DSPIC33AK_I2C_ERR_UNSUPPORTED;
}

/* ========================================================================== */
/* 3. Static local helper functions                                           */
/* ========================================================================== */

/* --------------------------------------------------------------------------
 * Validate instance number
 * -------------------------------------------------------------------------- */
static bool inst_is_valid(dspic33ak_i2c_instance_t inst)
{
    return ((unsigned)inst < (unsigned)DSPIC33AK_I2C_INST_COUNT);
}

/* --------------------------------------------------------------------------
 * Resolve instance to register table
 * -------------------------------------------------------------------------- */
static dspic33ak_i2c_status_t get_regs(
    dspic33ak_i2c_instance_t inst,
    const dspic33ak_i2c_regs_t **regs)
{
    const dspic33ak_i2c_device_t *dev;

    if (regs == 0) {
        return DSPIC33AK_I2C_ERR_INVALID_ARG;
    }

    if (!inst_is_valid(inst)) {
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
 * Resolve registers and require initialized state
 * -------------------------------------------------------------------------- */
static dspic33ak_i2c_status_t require_initialized(
    dspic33ak_i2c_instance_t inst,
    const dspic33ak_i2c_regs_t **regs)
{
    dspic33ak_i2c_status_t st;

    if (!inst_is_valid(inst)) {
        return DSPIC33AK_I2C_ERR_INVALID_ARG;
    }

    st = get_regs(inst, regs);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    if (!g_initialized[inst]) {
        return DSPIC33AK_I2C_ERR_NOT_INITIALIZED;
    }

    return DSPIC33AK_I2C_OK;
}

/* --------------------------------------------------------------------------
 * Require initialized state without exposing register pointer to caller
 * -------------------------------------------------------------------------- */
static dspic33ak_i2c_status_t check_initialized(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;
    return require_initialized(inst, &r);
}

/* --------------------------------------------------------------------------
 * Calculate BRG value
 * -------------------------------------------------------------------------- */
static uint32_t calc_brg(uint32_t fcy_hz, uint32_t bus_hz)
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
 * Check whether timeout is enabled
 * -------------------------------------------------------------------------- */
static bool timeout_enabled(dspic33ak_i2c_instance_t inst)
{
    return inst_is_valid(inst) &&
           (g_get_ms[inst] != 0) &&
           (g_timeout_ms[inst] != 0u);
}

/* --------------------------------------------------------------------------
 * Capture timeout start tick
 * -------------------------------------------------------------------------- */
static uint32_t timeout_start_ms(dspic33ak_i2c_instance_t inst)
{
    if (!timeout_enabled(inst)) {
        return 0u;
    }

    return g_get_ms[inst]();
}

/* --------------------------------------------------------------------------
 * Check timeout expiration
 * -------------------------------------------------------------------------- */
static bool timeout_expired(dspic33ak_i2c_instance_t inst, uint32_t start_ms)
{
    uint32_t now;

    if (!timeout_enabled(inst)) {
        return false;
    }

    now = g_get_ms[inst]();
    return ((uint32_t)(now - start_ms) >= g_timeout_ms[inst]);
}

/* --------------------------------------------------------------------------
 * Clear transfer-related status bits
 * -------------------------------------------------------------------------- */
static void clear_transfer_status(const dspic33ak_i2c_regs_t *r)
{
    /*
     * Keep status cleanup narrow.  Do not blindly write zero to all status
     * bits; only clear the bits used by this readable driver.
     */
    dspic33ak_i2c_reg_clear(r->STAT1,
                            DSPIC33AK_I2C_STAT1_IWCOL |
                            DSPIC33AK_I2C_STAT1_I2COV |
                            DSPIC33AK_I2C_STAT1_BCL);
    dspic33ak_i2c_reg_clear(r->STAT2,
                            DSPIC33AK_I2C_STAT2_ERR |
                            DSPIC33AK_I2C_STAT2_STARTE |
                            DSPIC33AK_I2C_STAT2_STOPE |
                            DSPIC33AK_I2C_STAT2_NACKE);
}

/* --------------------------------------------------------------------------
 * Convert bus fault status to driver status
 * -------------------------------------------------------------------------- */
static dspic33ak_i2c_status_t check_bus_fault(dspic33ak_i2c_instance_t inst)
{
    if (dspic33ak_i2c_ll_has_collision(inst)) {
        return DSPIC33AK_I2C_ERR_COLLISION;
    }

    if (dspic33ak_i2c_ll_has_nack(inst)) {
        return DSPIC33AK_I2C_ERR_NACK;
    }

    if (dspic33ak_i2c_ll_has_error(inst)) {
        return DSPIC33AK_I2C_ERR_BUS;
    }

    return DSPIC33AK_I2C_OK;
}

/* --------------------------------------------------------------------------
 * Wait for condition using optional timeout
 * -------------------------------------------------------------------------- */
static dspic33ak_i2c_status_t wait_until(
    dspic33ak_i2c_instance_t inst,
    bool (*done_fn)(dspic33ak_i2c_instance_t),
    bool check_nack)
{
    uint32_t start_ms = timeout_start_ms(inst);
    dspic33ak_i2c_status_t st;

    while (!done_fn(inst)) {
        if (check_nack) {
            st = check_bus_fault(inst);
        } else {
            if (dspic33ak_i2c_ll_has_collision(inst)) {
                st = DSPIC33AK_I2C_ERR_COLLISION;
            } else if (dspic33ak_i2c_ll_has_error(inst)) {
                st = DSPIC33AK_I2C_ERR_BUS;
            } else {
                st = DSPIC33AK_I2C_OK;
            }
        }

        if (st != DSPIC33AK_I2C_OK) {
            return st;
        }

        if (timeout_expired(inst, start_ms)) {
            return DSPIC33AK_I2C_ERR_TIMEOUT;
        }
    }

    if (dspic33ak_i2c_ll_has_collision(inst)) {
        return DSPIC33AK_I2C_ERR_COLLISION;
    }

    if (dspic33ak_i2c_ll_has_error(inst)) {
        return DSPIC33AK_I2C_ERR_BUS;
    }

    if (check_nack) {
        return check_bus_fault(inst);
    }

    return DSPIC33AK_I2C_OK;
}

/* --------------------------------------------------------------------------
 * Adapter: host state machine active condition
 * -------------------------------------------------------------------------- */
static bool host_active(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;

    if (get_regs(inst, &r) != DSPIC33AK_I2C_OK) {
        return false;
    }

    return dspic33ak_i2c_reg_is_set(r->STAT2, DSPIC33AK_I2C_STAT2_HSTACT);
}

/* --------------------------------------------------------------------------
 * Adapter: write byte ready condition
 * -------------------------------------------------------------------------- */
static bool write_byte_ready(dspic33ak_i2c_instance_t inst)
{
    /*
     * Same hardware bit as write_byte_done(), different phase meaning:
     * before TRN write, TRSTAT clear means the transmit state is ready.
     */
    return !dspic33ak_i2c_ll_write_byte_busy(inst);
}

/* --------------------------------------------------------------------------
 * Adapter: write byte done condition
 * -------------------------------------------------------------------------- */
static bool write_byte_done(dspic33ak_i2c_instance_t inst)
{
    /*
     * Same hardware bit as write_byte_ready(), different phase meaning:
     * after TRN write, TRSTAT clear means the current byte transfer is done.
     */
    return !dspic33ak_i2c_ll_write_byte_busy(inst);
}

/* --------------------------------------------------------------------------
 * Adapter: data/address phase accepted condition
 * -------------------------------------------------------------------------- */
static bool write_data_accepted(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;

    if (get_regs(inst, &r) != DSPIC33AK_I2C_OK) {
        return false;
    }

    return dspic33ak_i2c_reg_is_set(r->STAT1, DSPIC33AK_I2C_STAT1_D_A);
}

/* --------------------------------------------------------------------------
 * Adapter: ACK issue done condition
 * -------------------------------------------------------------------------- */
static bool ack_done(dspic33ak_i2c_instance_t inst)
{
    return !dspic33ak_i2c_ll_ack_busy(inst);
}

/* --------------------------------------------------------------------------
 * Issue STOP and wait quietly
 * -------------------------------------------------------------------------- */
static dspic33ak_i2c_status_t stop_quiet(dspic33ak_i2c_instance_t inst)
{
    dspic33ak_i2c_status_t st;

    st = dspic33ak_i2c_ll_stop_issue(inst);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    return wait_until(inst, dspic33ak_i2c_ll_stop_done, false);
}

/* --------------------------------------------------------------------------
 * Issue START and wait
 * -------------------------------------------------------------------------- */
static dspic33ak_i2c_status_t start_blocking(dspic33ak_i2c_instance_t inst)
{
    dspic33ak_i2c_status_t st;

    st = dspic33ak_i2c_ll_start_issue(inst);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    return wait_until(inst, dspic33ak_i2c_ll_start_done, false);
}

/* --------------------------------------------------------------------------
 * Issue repeated START and wait
 * -------------------------------------------------------------------------- */
static dspic33ak_i2c_status_t restart_blocking(dspic33ak_i2c_instance_t inst)
{
    dspic33ak_i2c_status_t st;

    st = dspic33ak_i2c_ll_restart_issue(inst);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    return wait_until(inst, dspic33ak_i2c_ll_restart_done, false);
}

/* --------------------------------------------------------------------------
 * Write one byte and wait
 * -------------------------------------------------------------------------- */
static dspic33ak_i2c_status_t write_byte_blocking(
    dspic33ak_i2c_instance_t inst,
    uint8_t data)
{
    dspic33ak_i2c_status_t st;

    /*
     * Byte write sequence:
     *   1. wait until the transmit state is idle
     *   2. wait until the host state machine owns the bus
     *   3. write TRN
     *   4. wait until the transmit state is complete
     *   5. wait until the module reports address/data phase accepted
     */
    st = wait_until(inst, write_byte_ready, false);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    st = wait_until(inst, host_active, false);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    st = dspic33ak_i2c_ll_write_byte_issue(inst, data);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    st = wait_until(inst, write_byte_done, false);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    st = wait_until(inst, write_data_accepted, false);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    if (!dspic33ak_i2c_ll_write_byte_acked(inst)) {
        return DSPIC33AK_I2C_ERR_NACK;
    }

    return DSPIC33AK_I2C_OK;
}

/* --------------------------------------------------------------------------
 * Read one byte, issue ACK/NACK, and wait
 * -------------------------------------------------------------------------- */
static dspic33ak_i2c_status_t read_byte_blocking(
    dspic33ak_i2c_instance_t inst,
    uint8_t *data,
    bool ack)
{
    dspic33ak_i2c_status_t st;

    st = dspic33ak_i2c_ll_read_byte_issue(inst);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    st = wait_until(inst, dspic33ak_i2c_ll_read_byte_ready, false);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    st = dspic33ak_i2c_ll_read_byte_get(inst, data);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    st = dspic33ak_i2c_ll_ack_issue(inst, ack);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    return wait_until(inst, ack_done, false);
}

/* --------------------------------------------------------------------------
 * Send 7-bit address with R/W bit
 * -------------------------------------------------------------------------- */
static dspic33ak_i2c_status_t send_address_blocking(
    dspic33ak_i2c_instance_t inst,
    uint8_t addr7,
    bool read)
{
    uint8_t addr_byte = (uint8_t)((addr7 << 1) | (read ? 1u : 0u));
    return write_byte_blocking(inst, addr_byte);
}
