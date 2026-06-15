#include <xc.h>
#include "dspic33ak_i2c_device.h"

/*
 * Device/instance mapping layer.
 *
 * This is the only place that should know about I2C1CON1/I2C2CON1/I2C3CON1
 * symbol names.  The portable-ish driver logic should use only the register
 * pointer table returned from dspic33ak_i2c_get_device().
 */

static const dspic33ak_i2c_device_t g_i2c_devices[DSPIC33AK_I2C_INST_COUNT] = {
#if defined(I2C1CON1)
    [DSPIC33AK_I2C_INST_1] = {
        .present = true,
        .regs = {
            .CON1 = &I2C1CON1,
            .CON2 = &I2C1CON2,
            .STAT1 = &I2C1STAT1,
            .STAT2 = &I2C1STAT2,
            .BITO = &I2C1BITO,
            .LBRG = &I2C1LBRG,
            .HBRG = &I2C1HBRG,
            .TRN = &I2C1TRN,
            .RCV = &I2C1RCV,
            .ADD = &I2C1ADD,
            .MSK = &I2C1MSK,
            .INTC = &I2C1INTC,
            .irq_event = { &IFS2, &IEC2, 0x00400000UL },  /* I2C1IF   IFS2<22> */
            .irq_rx    = { &IFS2, &IEC2, 0x00800000UL },  /* I2C1RXIF IFS2<23> */
            .irq_tx    = { &IFS2, &IEC2, 0x01000000UL },  /* I2C1TXIF IFS2<24> */
        },
    },
#else
    [DSPIC33AK_I2C_INST_1] = { .present = false },
#endif

#if defined(I2C2CON1)
    [DSPIC33AK_I2C_INST_2] = {
        .present = true,
        .regs = {
            .CON1 = &I2C2CON1,
            .CON2 = &I2C2CON2,
            .STAT1 = &I2C2STAT1,
            .STAT2 = &I2C2STAT2,
            .BITO = &I2C2BITO,
            .LBRG = &I2C2LBRG,
            .HBRG = &I2C2HBRG,
            .TRN = &I2C2TRN,
            .RCV = &I2C2RCV,
            .ADD = &I2C2ADD,
            .MSK = &I2C2MSK,
            .INTC = &I2C2INTC,
            .irq_event = { &IFS2, &IEC2, 0x04000000UL },  /* I2C2IF   IFS2<26> */
            .irq_rx    = { &IFS2, &IEC2, 0x08000000UL },  /* I2C2RXIF IFS2<27> */
            .irq_tx    = { &IFS2, &IEC2, 0x10000000UL },  /* I2C2TXIF IFS2<28> */
        },
    },
#else
    [DSPIC33AK_I2C_INST_2] = { .present = false },
#endif

#if defined(I2C3CON1)
    [DSPIC33AK_I2C_INST_3] = {
        .present = true,
        .regs = {
            .CON1 = &I2C3CON1,
            .CON2 = &I2C3CON2,
            .STAT1 = &I2C3STAT1,
            .STAT2 = &I2C3STAT2,
            .BITO = &I2C3BITO,
            .LBRG = &I2C3LBRG,
            .HBRG = &I2C3HBRG,
            .TRN = &I2C3TRN,
            .RCV = &I2C3RCV,
            .ADD = &I2C3ADD,
            .MSK = &I2C3MSK,
            .INTC = &I2C3INTC,
            .irq_event = { &IFS2, &IEC2, 0x40000000UL },  /* I2C3IF   IFS2<30> */
            .irq_rx    = { &IFS2, &IEC2, 0x80000000UL },  /* I2C3RXIF IFS2<31> */
            .irq_tx    = { &IFS3, &IEC3, 0x00000001UL },  /* I2C3TXIF IFS3<0>  */
        },
    },
#else
    [DSPIC33AK_I2C_INST_3] = { .present = false },
#endif

#if defined(I2C4CON1)
    [DSPIC33AK_I2C_INST_4] = {
        .present = true,
        .regs = {
            .CON1 = &I2C4CON1,
            .CON2 = &I2C4CON2,
            .STAT1 = &I2C4STAT1,
            .STAT2 = &I2C4STAT2,
            .BITO = &I2C4BITO,
            .LBRG = &I2C4LBRG,
            .HBRG = &I2C4HBRG,
            .TRN = &I2C4TRN,
            .RCV = &I2C4RCV,
        },
    },
#else
    [DSPIC33AK_I2C_INST_4] = { .present = false },
#endif
};

const dspic33ak_i2c_device_t *dspic33ak_i2c_get_device(
    dspic33ak_i2c_instance_t inst)
{
    if ((unsigned)inst >= (unsigned)DSPIC33AK_I2C_INST_COUNT) {
        return 0;
    }

    if (!g_i2c_devices[inst].present) {
        return 0;
    }

    return &g_i2c_devices[inst];
}

bool dspic33ak_i2c_instance_is_present(dspic33ak_i2c_instance_t inst)
{
    return (dspic33ak_i2c_get_device(inst) != 0);
}
