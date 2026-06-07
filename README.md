# dspic33ak-i2c-hal

Small, readable blocking I2C HAL for Microchip dsPIC33AK devices.

This project is intended as a compact alternative to large generated driver code.
The goal is not to hide everything behind a framework, but to provide a simple
driver that is easy to read, test, modify, and adapt.

## Status

Current validation target:

- Device: dsPIC33AK512MPS512
- Compiler: XC-DSC
- DFP: Microchip dsPIC33AK-MP DFP 1.3.185 or compatible
- Tested I2C instances: I2C2 and I2C3
- Tested peripheral: WM8904 audio codec
- Confirmed operations:
  - Blocking write
  - Blocking read
  - Blocking write-read transaction with repeated START
  - Repeated START using RSEN
  - WM8904 device ID read on I2C2 and I2C3

## Design policy

This driver is intentionally small.

- Normal API is blocking and simple.
- Low-level API separates issue and status-check operations.
- No XC-DSC / DFP bitfield structures are exposed in the public API.
- Device-specific register symbols are isolated in the device mapping layer.
- Interrupt helper APIs are reserved, but no interrupt-driven transfer engine is forced.
- Users can modify the polling waits into RTOS waits or interrupt-flag waits if needed.

## Files

```text
src/
  dspic33ak_i2c.c
  dspic33ak_i2c.h
  dspic33ak_i2c_device.c
  dspic33ak_i2c_device.h
  dspic33ak_i2c_reg.h
```

## Basic usage

```c
#include "dspic33ak_i2c.h"

static uint32_t app_get_ms(void)
{
    return app_millisecond_tick;
}

void app_i2c_init(void)
{
    const dspic33ak_i2c_config_t cfg = {
        .fcy_hz     = 100000000u,
        .bus_hz     = 400000u,
        .timeout_ms = 10u,
        .get_ms     = app_get_ms,
    };

    (void)dspic33ak_i2c_init(DSPIC33AK_I2C_INST_1, &cfg);
}
```

If `get_ms` is `NULL`, timeout handling is disabled.  
If `timeout_ms` is `0`, timeout handling is also disabled.

## Blocking write example

```c
uint8_t tx[2] = {
    0x01u,
    0x02u,
};

(void)dspic33ak_i2c_write(DSPIC33AK_I2C_INST_1,
                          0x48u,
                          tx,
                          sizeof(tx));
```

`addr7` is a 7-bit I2C slave address. Do not pass the R/W bit.

## Blocking write-read example

Typical register read sequence:

```c
uint8_t reg = 0x00u;
uint8_t rx[2];

(void)dspic33ak_i2c_write_read(DSPIC33AK_I2C_INST_1,
                               0x1au,
                               &reg,
                               1u,
                               rx,
                               sizeof(rx));
```

The write-read API generates a repeated START between the write phase and the
read phase.

## Repeated START

The low-level repeated START primitive uses the dsPIC33AK `RSEN` request bit.

Completion is checked using both:

- `RSEN` cleared by hardware
- `STARTE` set by hardware

This keeps the API name and the dsPIC33AK host control bit aligned.

## Low-level primitive API

The blocking API is built from smaller low-level primitives such as:

- `dspic33ak_i2c_ll_start_issue()`
- `dspic33ak_i2c_ll_restart_issue()`
- `dspic33ak_i2c_ll_stop_issue()`
- `dspic33ak_i2c_ll_write_byte_issue()`
- `dspic33ak_i2c_ll_read_byte_issue()`
- `dspic33ak_i2c_ll_ack_issue()`

These functions are useful if you want to replace polling loops with your own
interrupt flags, RTOS waits, or cooperative scheduler waits.

## Interrupt helper API

The following APIs are reserved for a future small interrupt helper layer:

- `dspic33ak_i2c_irq_enable()`
- `dspic33ak_i2c_irq_disable()`
- `dspic33ak_i2c_irq_clear()`

Current implementation returns:

```c
DSPIC33AK_I2C_ERR_UNSUPPORTED
```

The driver does not currently provide a full interrupt-driven transfer state
machine.

## Device mapping

The device mapping layer is the only place that directly references symbols
such as `I2C1CON1`, `I2C2CON1`, and `I2C3CON1`.

The main driver logic accesses registers through a small pointer table.

This keeps the driver easier to adapt across dsPIC33AK variants where the
available I2C instance count may differ.

## Notes

- `LBRG` and `HBRG` are set to the same value for a simple 50% duty-cycle SCL setup.
- BRG calculation uses 64-bit arithmetic internally to avoid overflow while keeping rounded divider behavior.
- `dspic33ak_i2c_deinit()` returns `DSPIC33AK_I2C_ERR_BUSY` if the host state machine or command bits indicate an active transfer.
- This repository does not include Microchip DFP header files.

## License

MIT License.

Copyright (c) 2026 SulaoLab
