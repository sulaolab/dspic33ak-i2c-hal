# dspic33ak-i2c-hal

> Want to run it on hardware first?
> Start with [dspic33ak-hal-starter](https://github.com/sulaolab/dspic33ak-hal-starter),
> which vendors validated snapshots of the dsPIC33AK HAL repositories and
> provides a ready-to-build MPLAB X project for the dsPIC33AK Curiosity board.

Small, readable I2C HAL for Microchip dsPIC33AK devices, with both a blocking
**master** API and an interrupt-driven, callback-based **slave** API.

This project is intended as a compact alternative to large generated driver code.
The goal is not to hide everything behind a framework, but to provide a simple
driver that is easy to read, test, modify, and adapt.

The master and slave roles live in separate headers so a program includes only
what it uses:

* `dspic33ak_i2c_master.h` — bus-master API (include for master use)
* `dspic33ak_i2c_slave.h` — slave/device API (include for slave use)
* `dspic33ak_i2c.h` — shared types and lifecycle, pulled in by both

A program may include either or both.

## Status

Current validation target:

* Device: dsPIC33AK512MPS512
* Compiler: XC-DSC
* DFP: Microchip dsPIC33AK-MP DFP 1.3.185 or compatible
* Tested I2C instances: I2C2 and I2C3
* Tested peripheral: WM8904 audio codec
* Tested bus speeds: 100 kHz, 150 kHz, 200 kHz, 300 kHz, 400 kHz

Other I2C instances may be available depending on the device mapping, but the
current hardware validation has been performed on I2C2 and I2C3.

Confirmed operations on the validation target:

* Blocking write
* Blocking read
* Blocking write-read transaction with repeated START
* Repeated START using RSEN
* WM8904 device ID read on I2C2 and I2C3
* no-STOP master write transaction
* repeated START read after pending write transaction
* explicit STOP for pending transaction
* pending transaction timeout recovery
* runtime bus speed change on an initialized idle instance
* 7-bit slave: address match, master-write reception, master-read transmission,
  and STOP detection, validated with an I2C2-master ↔ I2C3-slave (0x55)
  loopback round trip (master Write then Read)

## Design policy

This driver is intentionally small.

* Normal API is blocking and simple.
* Low-level API separates issue and status-check operations.
* No XC-DSC / DFP bitfield structures are exposed in the public API.
* Device-specific register symbols are isolated in the device mapping layer.
* Interrupt helper APIs are reserved, but no interrupt-driven transfer engine is forced.
* Users can modify the polling waits into RTOS waits or interrupt-flag waits if needed.

## Files

```text
src/
  dspic33ak_i2c.h          shared types + lifecycle (is_present / is_initialized / deinit)
  dspic33ak_i2c_master.h   master config + blocking / low-level / irq API
  dspic33ak_i2c_slave.h    slave config (callbacks) + slave API
  dspic33ak_i2c_master.c   master engine
  dspic33ak_i2c_slave.c    slave interrupt engine
  dspic33ak_i2c_common.c   shared primitives (instance validation, register
                           resolution, BRG calc, presence)
  dspic33ak_i2c_common.h
  dspic33ak_i2c_device.c   device register mapping
  dspic33ak_i2c_device.h
  dspic33ak_i2c_reg.h      internal register/bit definitions
```

Every `.c` has a paired `.h`; `dspic33ak_i2c.h` and `dspic33ak_i2c_reg.h` are
header-only.

## Basic usage (master)

```c
#include "dspic33ak_i2c_master.h"

static uint32_t app_get_ms(void)
{
    return app_millisecond_tick;
}

void app_i2c_init(void)
{
    const dspic33ak_i2c_config_t cfg = {
        .fcy_hz             = 100000000u,
        .bus_hz             = 400000u,
        .timeout_ms         = 10u,
        .get_ms             = app_get_ms,
        .pending_timeout_ms = 0u,
    };

    (void)dspic33ak_i2c_init(DSPIC33AK_I2C_INST_2, &cfg);
}
```

If `get_ms` is `NULL`, timeout handling is disabled.
If `timeout_ms` is `0`, timeout handling is also disabled.
If `pending_timeout_ms` is `0`, stale pending transaction recovery is disabled.

`addr7` arguments in this HAL are always 7-bit I2C slave addresses. Do not pass
the R/W bit.

## Runtime bus speed change

The bus speed of an already initialized instance can be changed at runtime:

```c
(void)dspic33ak_i2c_set_bus_speed(DSPIC33AK_I2C_INST_2,
                                  100000000u,   /* fcy_hz */
                                  100000u);     /* bus_hz */
```

The HAL accepts an arbitrary `bus_hz` value and computes `LBRG` / `HBRG` from
`fcy_hz`. The currently validated speeds are listed in the Status section. The
resulting BRG register range is not validated, so the caller is responsible for
passing an `fcy_hz` / `bus_hz` combination the device can represent.

This API uses the same BRG calculation as `dspic33ak_i2c_init()`. The peripheral
is briefly turned off (`CON1.ON`) around the BRG write and its previous ON state
is restored.

The instance must be initialized and idle. The call returns:

* `DSPIC33AK_I2C_ERR_INVALID_ARG` if `inst` is invalid or `fcy_hz` / `bus_hz` is `0`
* `DSPIC33AK_I2C_ERR_NOT_PRESENT` if the instance does not exist on the device
* `DSPIC33AK_I2C_ERR_NOT_INITIALIZED` if the instance has not been initialized
* `DSPIC33AK_I2C_ERR_BUSY` if the host state machine is active or a no-STOP
  transaction is pending
* `DSPIC33AK_I2C_OK` on success

It does not run stale pending recovery; recovery stays the responsibility of the
read-after-restart / stop / deinit paths.

## Blocking write example

```c
uint8_t tx[2] = {
    0x01u,
    0x02u,
};

(void)dspic33ak_i2c_write(DSPIC33AK_I2C_INST_2,
                          0x48u,
                          tx,
                          sizeof(tx));
```

## Blocking write-read example

Typical register read sequence:

```c
uint8_t reg = 0x00u;
uint8_t rx[2];

(void)dspic33ak_i2c_write_read(DSPIC33AK_I2C_INST_2,
                               0x1au,
                               &reg,
                               1u,
                               rx,
                               sizeof(rx));
```

The write-read API generates a repeated START between the write phase and the
read phase.

## Pending transaction API

For register-read style transfers, the normal `dspic33ak_i2c_write_read()` API
is usually sufficient.

For applications that need explicit control over a repeated START sequence, the
driver also provides a pending transaction API:

```c
uint8_t reg = 0x00u;
uint8_t rx[2];

(void)dspic33ak_i2c_master_write_no_stop(DSPIC33AK_I2C_INST_2,
                                         0x1au,
                                         &reg,
                                         1u);

(void)dspic33ak_i2c_master_read_after_restart(DSPIC33AK_I2C_INST_2,
                                              0x1au,
                                              rx,
                                              sizeof(rx));
```

`dspic33ak_i2c_master_write_no_stop()` starts a master write transaction and
leaves the bus active without issuing STOP.

`dspic33ak_i2c_master_read_after_restart()` continues the pending write
transaction by issuing a repeated START, reading data, and then issuing STOP.

If a pending transaction must be terminated without a read phase, call:

```c
(void)dspic33ak_i2c_master_stop(DSPIC33AK_I2C_INST_2);
```

The no-STOP transaction API is useful for higher-level wrappers or application
code that needs to keep the I2C bus active across multiple phases.

## Pending transaction timeout

`pending_timeout_ms` controls stale pending transaction recovery.

If both `get_ms` and `pending_timeout_ms` are valid, a later public HAL call can
detect an expired pending transaction, attempt to issue STOP, clear the pending
state, and return `DSPIC33AK_I2C_ERR_TIMEOUT`.

If `pending_timeout_ms` is `0`, pending timeout recovery is disabled.

## Repeated START

The low-level repeated START primitive uses the dsPIC33AK `RSEN` request bit.

Completion is checked using both:

* `RSEN` cleared by hardware
* `STARTE` set by hardware

This keeps the API name and the dsPIC33AK host control bit aligned.

## STOP completion

The low-level `dspic33ak_i2c_ll_stop_done()` primitive reports the hardware
`STOPE` status directly.

The higher-level blocking STOP path additionally waits for `CON1.PEN` to clear
before returning. On dsPIC33AK, `STAT2.STOPE` can become set before the STOP
request bit (`PEN`) is fully cleared. If the next START is requested while
`PEN` is still set, the host can ignore the START request.

This behavior was reproducible at 100 kHz with back-to-back transfers, so the
blocking STOP path treats STOP as complete only after both conditions are true:

* `STAT2.STOPE` is set
* `CON1.PEN` is cleared

## I2C slave (device) mode

Include `dspic33ak_i2c_slave.h` to answer as an I2C slave. The slave is
interrupt-driven and callback-based, and supports 7-bit addressing.

```c
#include "dspic33ak_i2c_slave.h"

static uint8_t reg_file[8];
static uint8_t rx_idx, tx_idx;

static void on_addr(bool is_read) { if (is_read) tx_idx = 0; else rx_idx = 0; }
static void on_rx(uint8_t b)      { if (rx_idx < sizeof reg_file) reg_file[rx_idx++] = b; }
static uint8_t on_tx(void)        { return (tx_idx < sizeof reg_file) ? reg_file[tx_idx++] : 0xFF; }
static void on_stop(void)         { /* transaction complete */ }

void app_i2c_slave_init(void)
{
    const dspic33ak_i2c_slave_config_t cfg = {
        .addr7         = 0x55u,
        .addr_mask     = 0u,      /* 0 = exact address match */
        .clock_stretch = false,   /* STREN: hold SCL for slower callbacks */
        .on_addr_match = on_addr,
        .on_rx_byte    = on_rx,
        .on_tx_byte    = on_tx,
        .on_stop       = on_stop,
    };
    (void)dspic33ak_i2c_slave_init(DSPIC33AK_I2C_INST_3, &cfg);
}
```

The application owns the three I2C interrupt vectors and forwards each to the
matching slave handler (the same pattern used for other interrupt-driven HALs
in this family). The HAL enables the interrupt sources; the application can set
their priority through `dspic33ak_i2c_set_interrupt_priority()`:

```c
/* once, before/around dspic33ak_i2c_slave_init(): */
(void)dspic33ak_i2c_set_interrupt_priority(DSPIC33AK_I2C_INST_3, 4u);

void __attribute__((__interrupt__, __no_auto_psv__)) _I2C3Interrupt(void)
{ dspic33ak_i2c_slave_event_irq(DSPIC33AK_I2C_INST_3); }
void __attribute__((__interrupt__, __no_auto_psv__)) _I2C3RXInterrupt(void)
{ dspic33ak_i2c_slave_rx_irq(DSPIC33AK_I2C_INST_3); }
void __attribute__((__interrupt__, __no_auto_psv__)) _I2C3TXInterrupt(void)
{ dspic33ak_i2c_slave_tx_irq(DSPIC33AK_I2C_INST_3); }
```

Callback contract:

* `on_addr_match(is_read)` — the master addressed this device. `is_read` is
  true when the master will read from us (we transmit via `on_tx_byte`), false
  when it will write to us (bytes arrive at `on_rx_byte`).
* `on_rx_byte(b)` — one received byte (master write).
* `on_tx_byte()` — return the next byte to transmit (master read); `0xFF` is
  sent if this is `NULL`.
* `on_stop()` — the transaction ended.

Callbacks run in interrupt context, so keep them short. With
`clock_stretch = true` the slave holds SCL (via `STREN`/`SCLREL`) to give the
callbacks more time. 10-bit addressing and general call are not handled yet.

## Low-level primitive API

The blocking API is built from smaller low-level primitives such as:

* `dspic33ak_i2c_ll_start_issue()`
* `dspic33ak_i2c_ll_restart_issue()`
* `dspic33ak_i2c_ll_stop_issue()`
* `dspic33ak_i2c_ll_write_byte_issue()`
* `dspic33ak_i2c_ll_read_byte_issue()`
* `dspic33ak_i2c_ll_ack_issue()`

These functions are useful if you want to replace polling loops with your own
interrupt flags, RTOS waits, or cooperative scheduler waits.

The low-level primitives intentionally expose simple hardware-oriented issue /
done checks. The normal blocking API adds the extra sequencing needed for safe
back-to-back transactions.

## Interrupt helper API

The device layer provides:

* `dspic33ak_i2c_set_interrupt_priority()`

It sets the interrupt priority symbols available for the selected instance on
the target device. Some instances expose only the event priority symbol; others
also expose dedicated RX/TX priority symbols. A priority of 0 leaves the
interrupt source masked by CPU priority rules; valid CPU priorities are 0
through 7.

The following APIs are reserved for a future small interrupt helper layer:

* `dspic33ak_i2c_irq_enable()`
* `dspic33ak_i2c_irq_disable()`
* `dspic33ak_i2c_irq_clear()`

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

* `LBRG` and `HBRG` are set to the same value for a simple 50% duty-cycle SCL setup.
* BRG calculation uses 64-bit arithmetic internally to avoid overflow while keeping rounded divider behavior.
* Runtime bus speed change requires an initialized idle instance (see [Runtime bus speed change](#runtime-bus-speed-change)).
* Blocking STOP waits for `CON1.PEN` to clear, not just `STAT2.STOPE` (see [STOP completion](#stop-completion)).
* `dspic33ak_i2c_deinit()` forces the peripheral off and clears HAL state; if it
  recovers a stale pending transaction, it may return that recovery status.
* This repository does not include Microchip DFP header files.

## License

MIT No Attribution License (MIT-0). See [LICENSE](LICENSE).

Attribution is appreciated but not required.
