#include <stdbool.h>
#include "../dspic33ak_i2c.h"

/*
 * Example only.
 *
 * The actual vector name, IEC/IFS flag names, and error-vector split depend on
 * the selected dsPIC33AK device and DFP header.
 *
 * The important idea is simple:
 *   - keep the ISR small
 *   - clear the interrupt flag
 *   - set a user-owned flag
 *   - let application code decide how to wait
 */

static volatile bool g_i2c3_done;
static volatile bool g_i2c3_error;

void i2c3_example_wait_flag_clear(void)
{
    g_i2c3_done = false;
    g_i2c3_error = false;
}

bool i2c3_example_is_done(void)
{
    return g_i2c3_done;
}

bool i2c3_example_has_error(void)
{
    return g_i2c3_error;
}

/*
 * Replace this function with the actual dsPIC33AK I2C3 ISR in the application.
 *
 * Example shape:
 *
 * void __attribute__((interrupt, no_auto_psv)) _I2C3Interrupt(void)
 * {
 *     dspic33ak_i2c_irq_clear(DSPIC33AK_I2C_INST_3,
 *                             DSPIC33AK_I2C_IRQ_TRANSFER_DONE);
 *     g_i2c3_done = true;
 * }
 */
void i2c3_example_transfer_done_isr_body(void)
{
    (void)dspic33ak_i2c_irq_clear(DSPIC33AK_I2C_INST_3,
                                  DSPIC33AK_I2C_IRQ_TRANSFER_DONE);
    g_i2c3_done = true;
}

/*
 * If the device has a separate I2C error interrupt, call this body from that ISR.
 */
void i2c3_example_error_isr_body(void)
{
    (void)dspic33ak_i2c_irq_clear(DSPIC33AK_I2C_INST_3,
                                  DSPIC33AK_I2C_IRQ_ERROR |
                                  DSPIC33AK_I2C_IRQ_BUS_COLLISION);
    g_i2c3_error = true;
}
