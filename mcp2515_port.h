#ifndef _MCP2515_PORT_H_
#define _MCP2515_PORT_H_

#include <stdint.h>
#include <stddef.h>

/*
 * Hardware abstraction for the MCP2515 driver.
 *
 * The driver itself performs no I/O: it asks this port to move bytes over SPI,
 * to drive chip-select, and to tell the time. Nothing in this header (or
 * anywhere else in this library) names a concrete peripheral, vendor HAL or
 * MCU family -- supplying that is entirely the application's job.
 *
 * Deliberately a plain C struct of function pointers rather than a C++
 * abstract class, so a backend can be written in C and linked against the C++
 * driver without the application having to compile any C++ of its own.
 */

#ifndef MCP2515_HAVE_ARDUINO
#  if defined(ARDUINO)
#    define MCP2515_HAVE_ARDUINO 1
#  elif defined(__has_include)
#    if __has_include(<Arduino.h>)
#      define MCP2515_HAVE_ARDUINO 1
#    else
#      define MCP2515_HAVE_ARDUINO 0
#    endif
#  else
#    define MCP2515_HAVE_ARDUINO 0
#  endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mcp2515_port {
    /*
     * Assert chip-select (drive CS low) and, if the bus is shared, acquire it.
     * This is the hook where a backend takes a mutex and/or applies its SPI
     * settings -- the MCP2515 needs mode 0, MSB-first, and SCK no faster than
     * 10 MHz.  Required.
     */
    void (*select)(void *ctx);

    /*
     * Release chip-select (drive CS high) and release the bus if it was
     * acquired in select().  Required.
     */
    void (*deselect)(void *ctx);

    /*
     * Full-duplex transfer of a single byte: clock out `tx`, return the byte
     * clocked in.  Called between select() and deselect().  Required.
     */
    uint8_t (*transfer)(void *ctx, uint8_t tx);

    /*
     * Optional block transfer. May be NULL, in which case the driver falls
     * back to looping over transfer() a byte at a time.
     *
     * Providing it is strongly recommended on hosts where a single-byte
     * transfer carries per-call overhead: the driver moves up to 14 bytes in
     * one go while resetting, and a full CAN frame is 13.
     *
     * `tx` may be NULL, meaning "clock out zeros". `rx` may be NULL, meaning
     * "discard what is clocked in". Both are never NULL at once.
     */
    void (*transfer_buffer)(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len);

    /*
     * Block for at least `ms` milliseconds. Used once, for the 10 ms the
     * MCP2515 needs to come out of reset. On an RTOS this should yield
     * (osDelay / vTaskDelay) rather than spin.  Required.
     */
    void (*delay_ms)(void *ctx, uint32_t ms);

    /*
     * A free-running millisecond counter. Only ever used to measure elapsed
     * time via unsigned subtraction, so it is allowed to wrap.  Required.
     */
    uint32_t (*ticks_ms)(void *ctx);

    /* Opaque backend state, passed back to every callback above. */
    void *ctx;
} mcp2515_port_t;

#ifdef __cplusplus
}
#endif

#endif /* _MCP2515_PORT_H_ */
