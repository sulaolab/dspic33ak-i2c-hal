#ifndef DSPIC33AK_I2C_SLAVE_H
#define DSPIC33AK_I2C_SLAVE_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "dspic33ak_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * dsPIC33AK I2C slave (device/client) driver interface.
 *
 * Include this header to answer as an I2C slave. The shared instance / status
 * types live in dspic33ak_i2c.h (included above); the bus-master role is in
 * dspic33ak_i2c_master.h. A program may include either or both.
 *
 * The slave is callback-driven and interrupt-based. The application owns the
 * three I2C interrupt vectors (_I2CxInterrupt / _I2CxRXInterrupt /
 * _I2CxTXInterrupt) and simply delegates each to the matching
 * dspic33ak_i2c_slave_*_irq() handler below (same pattern as the UART HAL).
 *
 * Scope: 7-bit addressing only. 10-bit and general-call are not handled yet.
 */

typedef struct {
    uint8_t addr7;          /* 7-bit own address (right-justified, e.g. 0x55) */
    uint8_t addr_mask;      /* I2CxMSK low 7 bits; 0 = exact match            */
    bool    clock_stretch;  /* STREN: hold SCL low to give callbacks time     */

    /* Address phase: the master addressed us. is_read is true when the master
     * wants to read from us (it will clock bytes out of on_tx_byte), false
     * when it will write to us (bytes arrive at on_rx_byte). May be NULL. */
    void (*on_addr_match)(bool is_read);

    /* Master-write: one received data byte. May be NULL (byte is dropped). */
    void (*on_rx_byte)(uint8_t b);

    /* Master-read: return the next byte to transmit. If NULL, 0xFF is sent. */
    uint8_t (*on_tx_byte)(void);

    /* STOP (or bus-release) ended the transaction. May be NULL. */
    void (*on_stop)(void);
} dspic33ak_i2c_slave_config_t;

/* Configure the instance as a slave at config->addr7 and enable it. */
dspic33ak_i2c_status_t dspic33ak_i2c_slave_init(
    dspic33ak_i2c_instance_t inst,
    const dspic33ak_i2c_slave_config_t *config);

/* Disable the slave: turn the peripheral off, mask its interrupts, drop state. */
dspic33ak_i2c_status_t dspic33ak_i2c_slave_deinit(
    dspic33ak_i2c_instance_t inst);

/* True once dspic33ak_i2c_slave_init() has configured this instance. */
bool dspic33ak_i2c_slave_is_active(dspic33ak_i2c_instance_t inst);

/* ISR delegates. Call from the corresponding application-owned vector:
 *   _I2CxInterrupt   -> dspic33ak_i2c_slave_event_irq(inst)
 *   _I2CxRXInterrupt -> dspic33ak_i2c_slave_rx_irq(inst)
 *   _I2CxTXInterrupt -> dspic33ak_i2c_slave_tx_irq(inst)
 * Each clears the hardware flag it handles. */
void dspic33ak_i2c_slave_event_irq(dspic33ak_i2c_instance_t inst);
void dspic33ak_i2c_slave_rx_irq(dspic33ak_i2c_instance_t inst);
void dspic33ak_i2c_slave_tx_irq(dspic33ak_i2c_instance_t inst);

#ifdef __cplusplus
}
#endif

#endif /* DSPIC33AK_I2C_SLAVE_H */
