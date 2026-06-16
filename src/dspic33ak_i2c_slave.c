#include "dspic33ak_i2c_slave.h"
#include "dspic33ak_i2c_device.h"
#include "dspic33ak_i2c_reg.h"
#include "dspic33ak_i2c_common.h"

/* --------------------------------------------------------------------------
 * dsPIC33AK I2C slave engine (interrupt-driven, callback-based).
 *
 * The peripheral is the newer "smart"/FIFO I2C, but we run it in the classic,
 * non-smart client mode and drive it one byte at a time (CON2.PSZ = 1). In
 * that mode the data sheet (DS70005591A, client-reception/transmission timing)
 * says the *event* interrupt I2CxIF fires on the address match and on every
 * data byte, and - with PCIE set - on STOP. The dedicated RX/TX interrupts are
 * mainly for the DMA/smart path, so we treat them only as a hedge: all three
 * ISR delegates funnel into one re-entrant-safe service routine that polls
 * STAT1 and acts on whatever is pending. Reading I2CxRCV clears RBF, so if more
 * than one flag is raised for the same byte the extra pass is a harmless no-op.
 *
 * STAT1 tells us what happened:
 *   RBF  - a byte (address or data) is in RCV
 *   D_A  - 0 = that byte was the address, 1 = data
 *   R_W  - at the address, 1 = master reads from us (we transmit)
 *   TBF  - transmit buffer full (clear => the master-read wants the next byte)
 *   P    - STOP detected
 * After handling a byte we set SCLREL to release SCL: the hardware holds the
 * clock after an address byte even when STREN = 0, so this is always required.
 * -------------------------------------------------------------------------- */

static dspic33ak_i2c_slave_config_t g_cfg[DSPIC33AK_I2C_INST_COUNT];
static bool                         g_active[DSPIC33AK_I2C_INST_COUNT];
static bool                         g_reading[DSPIC33AK_I2C_INST_COUNT];

/* --------------------------------------------------------------------------
 * Init
 * -------------------------------------------------------------------------- */
dspic33ak_i2c_status_t dspic33ak_i2c_slave_init(
    dspic33ak_i2c_instance_t inst,
    const dspic33ak_i2c_slave_config_t *config)
{
    const dspic33ak_i2c_regs_t *r;
    dspic33ak_i2c_status_t st;

    if (config == 0 || config->addr7 > 0x7Fu) {
        return DSPIC33AK_I2C_ERR_INVALID_ARG;
    }

    st = dspic33ak_i2c_get_regs(inst, &r);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    /* The slave engine needs the slave-only registers (ADD/MSK/INTC) and the
     * event interrupt descriptor. An instance whose device table entry only
     * maps the master registers (e.g. an I2Cx that exists but was not given
     * slave mappings) must not be driven as a slave - that would dereference
     * NULL. Reject it instead. */
    if (r->ADD == 0 || r->MSK == 0 || r->INTC == 0 ||
        r->irq_event.ifs == 0 || r->irq_event.iec == 0) {
        return DSPIC33AK_I2C_ERR_NOT_PRESENT;
    }

    /* Configure with the module disabled. */
    dspic33ak_i2c_reg_clear(r->CON1, DSPIC33AK_I2C_CON1_ON);

    /* Classic client mode: 7-bit (A10M=0), no host request bits, no smart/FIFO
     * features. PCIE makes a STOP feed the client interrupt (CLIIF). */
    dspic33ak_i2c_reg_write(r->CON1, DSPIC33AK_I2C_CON1_PCIE);
    dspic33ak_i2c_reg_write(r->CON2, DSPIC33AK_I2C_CON2_PSZ_1_BYTE);

    /* Route every client condition to the event interrupt I2CxIF: address-match
     * (CADDRIE), received byte (CDRXIE) and the "send next byte" request after a
     * transmitted byte is ACKed (CDTXIE) all feed CLIIF, which CLTIE gates onto
     * I2CxIF. Without this the hardware sets RBF/TBF but raises no interrupt. */
    *r->INTC = DSPIC33AK_I2C_INTC_CLTIE   |
               DSPIC33AK_I2C_INTC_CADDRIE |
               DSPIC33AK_I2C_INTC_CDRXIE  |
               DSPIC33AK_I2C_INTC_CDTXIE;

    if (config->clock_stretch) {
        dspic33ak_i2c_reg_set(r->CON1, DSPIC33AK_I2C_CON1_STREN);
    }

    /* 7-bit own address (right-justified in ADD<6:0>) and address mask. */
    *r->ADD = (uint32_t)config->addr7;
    *r->MSK = (uint32_t)config->addr_mask;

    /* Drop any stale receive-overflow latch. */
    dspic33ak_i2c_reg_clear(r->STAT1, DSPIC33AK_I2C_STAT1_I2COV);

    g_cfg[inst]     = *config;
    g_reading[inst] = false;
    g_active[inst]  = true;
    dspic33ak_i2c_set_role(inst, DSPIC33AK_I2C_ROLE_SLAVE);

    /* All client activity (address / RX / TX-continue / STOP) is aggregated
     * into the single event interrupt via INTC above, so only that vector is
     * enabled here. */
    dspic33ak_i2c_reg_irq_clear(&r->irq_event);
    dspic33ak_i2c_reg_irq_enable(&r->irq_event);

    /* Release SCL and turn the slave on. */
    dspic33ak_i2c_reg_set(r->CON1, DSPIC33AK_I2C_CON1_SCLREL);
    dspic33ak_i2c_reg_set(r->CON1, DSPIC33AK_I2C_CON1_ON);

    return DSPIC33AK_I2C_OK;
}

/* --------------------------------------------------------------------------
 * Deinit
 * -------------------------------------------------------------------------- */
dspic33ak_i2c_status_t dspic33ak_i2c_slave_deinit(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;
    dspic33ak_i2c_status_t st;

    st = dspic33ak_i2c_get_regs(inst, &r);
    if (st != DSPIC33AK_I2C_OK) {
        return st;
    }

    /* As in slave_init: an instance without slave register mappings was never
     * (and cannot be) a slave, so there is nothing to tear down. Guard against
     * dereferencing the NULL irq/INTC pointers such an instance would carry. */
    if (r->ADD == 0 || r->MSK == 0 || r->INTC == 0 ||
        r->irq_event.ifs == 0 || r->irq_event.iec == 0) {
        return DSPIC33AK_I2C_ERR_NOT_PRESENT;
    }

    dspic33ak_i2c_reg_irq_disable(&r->irq_event);
    dspic33ak_i2c_reg_irq_disable(&r->irq_rx);
    dspic33ak_i2c_reg_irq_disable(&r->irq_tx);

    dspic33ak_i2c_reg_clear(r->CON1, DSPIC33AK_I2C_CON1_ON);

    g_active[inst]  = false;
    g_reading[inst] = false;
    dspic33ak_i2c_set_role(inst, DSPIC33AK_I2C_ROLE_NONE);

    return DSPIC33AK_I2C_OK;
}

bool dspic33ak_i2c_slave_is_active(dspic33ak_i2c_instance_t inst)
{
    if (!dspic33ak_i2c_inst_is_valid(inst)) {
        return false;
    }
    return g_active[inst];
}

/* --------------------------------------------------------------------------
 * Shared service: poll STAT1 and act on whatever is pending. Called from every
 * ISR delegate; idempotent because reading RCV clears RBF.
 * -------------------------------------------------------------------------- */
static uint8_t next_tx_byte(dspic33ak_i2c_instance_t inst)
{
    if (g_cfg[inst].on_tx_byte != 0) {
        return g_cfg[inst].on_tx_byte();
    }
    return 0xFFu;
}

static void slave_service(dspic33ak_i2c_instance_t inst,
                          const dspic33ak_i2c_regs_t *r)
{
    uint32_t stat = *r->STAT1;

    if ((stat & DSPIC33AK_I2C_STAT1_RBF) != 0u) {
        uint8_t b = (uint8_t)(*r->RCV & 0xFFu);     /* read clears RBF */

        if ((stat & DSPIC33AK_I2C_STAT1_D_A) == 0u) {
            /* Address byte: latch direction and notify. The hardware has
             * stretched SCL after the address; release it below. */
            bool is_read = ((stat & DSPIC33AK_I2C_STAT1_R_W) != 0u);
            g_reading[inst] = is_read;

            if (g_cfg[inst].on_addr_match != 0) {
                g_cfg[inst].on_addr_match(is_read);
            }
            if (is_read) {
                /* Master-read: load the first byte to transmit. */
                *r->TRN = (uint32_t)next_tx_byte(inst);
            }
        } else if (!g_reading[inst]) {
            /* Master-write data byte. */
            if (g_cfg[inst].on_rx_byte != 0) {
                g_cfg[inst].on_rx_byte(b);
            }
        }

        dspic33ak_i2c_reg_set(r->CON1, DSPIC33AK_I2C_CON1_SCLREL);
    } else if (g_reading[inst]) {
        /* Master-read in progress: this interrupt is the falling edge of the
         * ACK/NACK after the byte we just transmitted. */
        if ((stat & DSPIC33AK_I2C_STAT1_ACKSTAT) == 0u) {
            /* ACK: the host wants more -> load the next byte and release. */
            *r->TRN = (uint32_t)next_tx_byte(inst);
            dspic33ak_i2c_reg_set(r->CON1, DSPIC33AK_I2C_CON1_SCLREL);
        } else {
            /* NACK: the read is finished. Do not write TRN again; the module
             * stops stretching on its own and a STOP follows. */
            g_reading[inst] = false;
        }
    }

    if ((stat & DSPIC33AK_I2C_STAT1_P) != 0u) {
        /* STOP: end of transaction. */
        g_reading[inst] = false;
        if (g_cfg[inst].on_stop != 0) {
            g_cfg[inst].on_stop();
        }
    }

    /* Never let a receive-overflow latch wedge the slave. */
    if (dspic33ak_i2c_reg_is_set(r->STAT1, DSPIC33AK_I2C_STAT1_I2COV)) {
        dspic33ak_i2c_reg_clear(r->STAT1, DSPIC33AK_I2C_STAT1_I2COV);
    }
}

/* --------------------------------------------------------------------------
 * ISR delegates: each clears its own flag *before* servicing, so a new event
 * raised while servicing (the master can resume the moment we release SCLREL)
 * keeps its flag set and re-enters rather than being cleared away. The service
 * routine is idempotent, so a spurious re-entry is harmless.
 * -------------------------------------------------------------------------- */
void dspic33ak_i2c_slave_event_irq(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;

    if (dspic33ak_i2c_get_regs(inst, &r) != DSPIC33AK_I2C_OK) {
        return;
    }
    dspic33ak_i2c_reg_irq_clear(&r->irq_event);
    if (g_active[inst]) {
        slave_service(inst, r);
    }
}

void dspic33ak_i2c_slave_rx_irq(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;

    if (dspic33ak_i2c_get_regs(inst, &r) != DSPIC33AK_I2C_OK) {
        return;
    }
    dspic33ak_i2c_reg_irq_clear(&r->irq_rx);
    if (g_active[inst]) {
        slave_service(inst, r);
    }
}

void dspic33ak_i2c_slave_tx_irq(dspic33ak_i2c_instance_t inst)
{
    const dspic33ak_i2c_regs_t *r;

    if (dspic33ak_i2c_get_regs(inst, &r) != DSPIC33AK_I2C_OK) {
        return;
    }
    dspic33ak_i2c_reg_irq_clear(&r->irq_tx);
    if (g_active[inst]) {
        slave_service(inst, r);
    }
}
