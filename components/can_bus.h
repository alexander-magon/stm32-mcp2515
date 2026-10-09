#ifndef CAN_BUS_H_
#define CAN_BUS_H_

/*
 * C-callable front end for the MCP2515 driver.
 *
 * The driver is C++; this component exposes it to a C application and owns the
 * single controller instance, so a C-only project never has to compile any C++
 * of its own. Frames use `struct can_frame` from can.h, following the
 * SocketCAN convention: the EFF/RTR flags live in the top bits of can_id.
 *
 * Nothing here is hardware-specific. The application supplies CanPort_Get(),
 * returning an mcp2515_port_t that says how to reach the controller on its
 * board; everything below is board-independent.
 *
 * Layering:
 *   components/can_bus.*   - this API, owns the MCP2515 instance
 *   mcp2515.{h,cpp}        - protocol driver, no hardware knowledge
 *   mcp2515_port.h         - the contract the application implements
 *   <application>          - CanPort_Get(), backed by its own SPI and GPIO
 */

#include "can.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Crystal fitted to the MCP2515 module. Getting this wrong does not fail
 * loudly -- it silently produces the wrong bit rate, and the node just never
 * ACKs. The common blue "MCP2515 + TJA1050" breakout boards ship an 8 MHz can.
 * Check the part marking on yours before trusting this default.
 */
#ifndef CAN_BUS_MCP_CLOCK
#define CAN_BUS_MCP_CLOCK MCP_8MHZ
#endif

/* Bus bit rate. Must match every other node on the bus. */
#ifndef CAN_BUS_BITRATE
#define CAN_BUS_BITRATE CAN_500KBPS
#endif

typedef enum {
    CAN_BUS_OK = 0,
    CAN_BUS_ERR_INIT,    /* reset / bitrate / mode change failed */
    CAN_BUS_ERR_TX,      /* all TX buffers busy, or transmit failed */
    CAN_BUS_ERR_NO_MSG,  /* CanBus_Receive found nothing pending */
    CAN_BUS_ERR_FAIL,    /* catch-all */
} can_bus_status_t;

/*
 * Reset the controller, apply CAN_BUS_BITRATE and enter normal mode.
 * Call once, after MX_SPI1_Init() and MX_GPIO_Init(), and before any other
 * function here. Safe to call again; subsequent calls re-initialise.
 */
can_bus_status_t CanBus_Init(void);

/*
 * Put the controller in loopback instead of normal mode. Nothing reaches the
 * wire and no ACK is required, so this exercises the whole SPI path with no
 * second node attached -- the quickest way to tell a wiring fault from a bus
 * configuration fault. Call after CanBus_Init().
 */
can_bus_status_t CanBus_SetLoopback(void);

/*
 * Listen-only mode: receive everything, never transmit, never ACK, never emit
 * error frames.
 *
 * USE THIS WHEN SNIFFING A VEHICLE. In normal mode this node ACKs every frame
 * it accepts and signals error frames when it disagrees with what it sees --
 * so a wrong bit rate or a half-working transceiver turns a passive tap into
 * something that actively corrupts the car's bus. Listen-only makes that
 * impossible at the controller level.
 *
 * The trade-off is that nothing can be transmitted, including the ACK a
 * diagnostic tester would need, so OBD request/response flows need normal mode.
 */
can_bus_status_t CanBus_SetListenOnly(void);

/*
 * Restrict which frames reach the RX buffers, using the controller's hardware
 * acceptance filters. Frames rejected here never consume a buffer and never
 * reach the CPU.
 *
 * On a live vehicle bus this is not optional: the controller has exactly two
 * RX buffers, and a VW powertrain bus can run well past 1000 frames/s, so an
 * unfiltered sniffer overflows continuously and prints an arbitrary subset.
 *
 * `mask` selects which ID bits must match (1 = must match, 0 = don't care),
 * `id` is the value those bits are compared against. Applies to both RX
 * buffers and all six filters, so it is a single global accept rule.
 *
 *   CanBus_SetIdFilter(0x7FF, 0x7E8, false)  -> only that one 11-bit ID
 *   CanBus_SetIdFilter(0x7F0, 0x7E0, false)  -> the 0x7E0..0x7EF OBD range
 */
can_bus_status_t CanBus_SetIdFilter(uint32_t mask, uint32_t id, bool extended);

/* Undo CanBus_SetIdFilter: accept every ID again (the post-Init default). */
can_bus_status_t CanBus_AcceptAll(void);

/* Queue a frame into the first free TX buffer. */
can_bus_status_t CanBus_Send(const struct can_frame *frame);

/* Pop one frame from whichever RX buffer holds it. */
can_bus_status_t CanBus_Receive(struct can_frame *frame);

/* True when at least one RX buffer is occupied. */
bool CanBus_HasPending(void);

/* Raw EFLG register, for diagnostics (bus-off, error-passive, overflow). */
uint8_t CanBus_GetErrorFlags(void);

/*
 * Clear the RX overflow flags (EFLG.RX0OVR / RX1OVR).
 *
 * Reading a message clears only that buffer's interrupt flag; the overflow
 * bits latch until cleared explicitly. Without this they stay set forever
 * after the first overflow, so the condition cannot be observed a second time.
 */
void CanBus_ClearOverflow(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_BUS_H_ */
