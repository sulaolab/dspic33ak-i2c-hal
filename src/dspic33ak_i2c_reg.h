#ifndef DSPIC33AK_I2C_REG_H
#define DSPIC33AK_I2C_REG_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Internal register helper layer.
 *
 * This file intentionally uses 32-bit register pointers and bit masks instead
 * of XC-DSC bitfield structures such as I2C1CON1BITS.  The goal is to keep
 * compiler/DFP-specific details away from the public API.
 *
 * Bit positions were checked against:
 *   Microchip.dsPIC33AK-MP_DFP.1.3.185.atpack
 *   xc16/support/dsPIC33A/h/p33AK512MPS512.h
 *
 * Keep this file small.  Add only the bits that are actually used by the
 * readable I2C driver.
 */

typedef struct {
    volatile uint32_t *CON1;
    volatile uint32_t *CON2;
    volatile uint32_t *STAT1;
    volatile uint32_t *STAT2;
    volatile uint32_t *BITO;
    volatile uint32_t *LBRG;
    volatile uint32_t *HBRG;
    volatile uint32_t *TRN;
    volatile uint32_t *RCV;
} dspic33ak_i2c_regs_t;

/* I2CxCON1 bits */
#define DSPIC33AK_I2C_CON1_SEN      (1UL << 0)   /* I2CxCON1bits.SEN */
#define DSPIC33AK_I2C_CON1_RSEN     (1UL << 1)   /* I2CxCON1bits.RSEN */
#define DSPIC33AK_I2C_CON1_PEN      (1UL << 2)   /* I2CxCON1bits.PEN */
#define DSPIC33AK_I2C_CON1_RCEN     (1UL << 3)   /* I2CxCON1bits.RCEN */
#define DSPIC33AK_I2C_CON1_ACKEN    (1UL << 4)   /* I2CxCON1bits.ACKEN */
#define DSPIC33AK_I2C_CON1_ACKDT    (1UL << 5)   /* I2CxCON1bits.ACKDT */
#define DSPIC33AK_I2C_CON1_ON       (1UL << 15)  /* I2CxCON1bits.ON */

/* I2CxCON2 bits */
#define DSPIC33AK_I2C_CON2_BITE     (1UL << 16)  /* I2CxCON2bits.BITE */

/* I2CxSTAT1 bits */
#define DSPIC33AK_I2C_STAT1_TBF     (1UL << 0)   /* I2CxSTAT1bits.TBF */
#define DSPIC33AK_I2C_STAT1_RBF     (1UL << 1)   /* I2CxSTAT1bits.RBF */
#define DSPIC33AK_I2C_STAT1_S       (1UL << 3)   /* I2CxSTAT1bits.S */
#define DSPIC33AK_I2C_STAT1_D_A     (1UL << 5)   /* I2CxSTAT1bits.D_A */
#define DSPIC33AK_I2C_STAT1_I2COV   (1UL << 6)   /* I2CxSTAT1bits.I2COV */
#define DSPIC33AK_I2C_STAT1_IWCOL   (1UL << 7)   /* I2CxSTAT1bits.IWCOL */
#define DSPIC33AK_I2C_STAT1_BCL     (1UL << 10)  /* I2CxSTAT1bits.BCL */
#define DSPIC33AK_I2C_STAT1_TRSTAT  (1UL << 14)  /* I2CxSTAT1bits.TRSTAT */
#define DSPIC33AK_I2C_STAT1_ACKSTAT (1UL << 15)  /* I2CxSTAT1bits.ACKSTAT */

/* I2CxSTAT2 bits */
#define DSPIC33AK_I2C_STAT2_ERR     (1UL << 11)  /* I2CxSTAT2bits.ERR */
#define DSPIC33AK_I2C_STAT2_CLTIF   (1UL << 12)  /* I2CxSTAT2bits.CLTIF */
#define DSPIC33AK_I2C_STAT2_HSTIF   (1UL << 13)  /* I2CxSTAT2bits.HSTIF */
#define DSPIC33AK_I2C_STAT2_STARTE  (1UL << 14)  /* I2CxSTAT2bits.STARTE */
#define DSPIC33AK_I2C_STAT2_STOPE   (1UL << 15)  /* I2CxSTAT2bits.STOPE */
#define DSPIC33AK_I2C_STAT2_NACKE   (1UL << 18)  /* I2CxSTAT2bits.NACKE */
#define DSPIC33AK_I2C_STAT2_BITO    (1UL << 20)  /* I2CxSTAT2bits.BITO */
#define DSPIC33AK_I2C_STAT2_HSTACT  (1UL << 29)  /* I2CxSTAT2bits.HSTACT */
#define DSPIC33AK_I2C_STAT2_CLTACT  (1UL << 30)  /* I2CxSTAT2bits.CLTACT */
#define DSPIC33AK_I2C_STAT2_SSPND   (1UL << 31)  /* I2CxSTAT2bits.SSPND */

/* I2CxBITO field */
#define DSPIC33AK_I2C_BITO_BITOTMR_MASK  (0x00FFFFFFUL) /* I2CxBITObits.BITOTMR */

static inline void dspic33ak_i2c_reg_set(volatile uint32_t *reg, uint32_t mask)
{
    *reg |= mask;
}

static inline void dspic33ak_i2c_reg_clear(volatile uint32_t *reg, uint32_t mask)
{
    *reg &= ~mask;
}

static inline bool dspic33ak_i2c_reg_is_set(volatile uint32_t *reg, uint32_t mask)
{
    return ((*reg & mask) != 0u);
}

static inline void dspic33ak_i2c_reg_write_field(
    volatile uint32_t *reg,
    uint32_t mask,
    uint32_t value)
{
    *reg = (*reg & ~mask) | (value & mask);
}

#endif /* DSPIC33AK_I2C_REG_H */
