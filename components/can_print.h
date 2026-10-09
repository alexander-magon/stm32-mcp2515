#ifndef CAN_PRINT_H_
#define CAN_PRINT_H_

/*
 * Human-readable tracing of received CAN frames.
 *
 * Output is one line per frame, candump-flavoured so it reads the same way as
 * SocketCAN logs:
 *
 *   CAN      7E8 [8] 06 41 0C 1A F8 00 00 00 |.A......| OBD2.resp ECU0 SF:6 UDS+ 01
 *   CAN 18DAF110 [8] E 03 7F 22 31 00 00 00 00 |..".....| diag.phys tgt=F1 src=10 ...
 *   CAN      280 [8] 00 00 1A F8 00 00 00 00 |........|
 *
 * The annotations after the ASCII gutter are decoded only from ISO standards
 * (ISO 15765-2/-4 addressing and ISO-TP, ISO 14229 / SAE J1979 services), which
 * hold for any vehicle. Manufacturer-specific broadcast IDs are NOT decoded --
 * see can_print_known_ids in can_print.c for how to name the ones on your car.
 *
 * Besides plain tracing this implements change detection, which is what makes
 * an unknown control findable on a live vehicle bus. See can_print_mode_t.
 *
 * Platform-independent: the only thing it needs from the outside is a
 * millisecond clock, installed with CanPrint_SetClock(). Frames arrive through
 * CanPrint_Feed(), so how they are collected -- polled, interrupt-driven,
 * replayed from a file in a unit test -- is entirely the caller's business.
 */

#include "can.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Longest line the formatter can emit, including the NUL. */
#define CAN_PRINT_LINE_MAX 160

/*
 * Where trace output goes. Defaults to printf; override to route it to a
 * ring buffer, a second UART, or a host-side capture in tests.
 */
#ifndef CAN_PRINT_PRINTF
#include <stdio.h>
#define CAN_PRINT_PRINTF printf
#endif

/* Distinct identifiers tracked in change mode. 95 were seen on a stationary
 * MQB-platform car, so this leaves headroom. 16 bytes of RAM each. */
#ifndef CAN_PRINT_MAX_IDS
#define CAN_PRINT_MAX_IDS 128U
#endif

/* Default length of the learn window, in milliseconds. */
#ifndef CAN_PRINT_LEARN_MS
#define CAN_PRINT_LEARN_MS 5000U
#endif

/*
 * How many change lines one identifier may print before it is muted.
 *
 * This is what makes the output fall silent by itself. Anything still moving
 * after the learn window -- a slow counter, a temperature, a wheel speed --
 * spends its budget within a second or two and goes quiet, leaving only
 * identifiers that have not moved at all. Press a button then and its frame
 * still has its full budget, so it prints.
 *
 * 2 shows a press and its release before muting. Raise it to watch a control
 * for longer, or set 0 for no muting at all.
 */
#ifndef CAN_PRINT_CHANGE_BUDGET
#define CAN_PRINT_CHANGE_BUDGET 2U
#endif

typedef enum {
    /* Every frame, fully decoded. Useful for diagnostics, far too fast to
     * read on a live vehicle bus. */
    CAN_PRINT_MODE_RAW = 0,

    /*
     * Change detection, for reverse-engineering a control whose frame is
     * unknown. Runs in two phases:
     *
     *   learn  - every identifier is announced once (the device inventory),
     *            and every byte seen to change is recorded as volatile.
     *            Leave the car alone and touch nothing during this.
     *   watch  - only bytes that stayed constant through the learn window are
     *            reported when they change.
     *
     * The learn phase is what makes this usable. Roughly half the identifiers
     * on a VW carry a rolling counter and checksum in bytes 0-1 that change on
     * every single frame; without masking them, a plain payload diff still
     * floods the output and buries the button.
     */
    CAN_PRINT_MODE_CHANGES,
} can_print_mode_t;

/* Select what the monitor task prints. Switching to CHANGES restarts the
 * learn window and discards any previously learned state. */
void CanPrint_SetMode(can_print_mode_t mode);

/*
 * Restart the learn window, keeping the identifier inventory but clearing all
 * volatile-byte masks. Use when a stray press during learning masked the very
 * byte you were looking for. Pass 0 for CAN_PRINT_LEARN_MS.
 */
void CanPrint_Relearn(uint32_t learn_ms);

/*
 * True while the learn window is still open.
 *
 * Exposed so a caller can align its own bookkeeping with the same window --
 * e.g. accumulating per-identifier frame rates during learning and reporting
 * them once it closes. Polling this for the true->false edge is more robust
 * than recomputing the deadline from CAN_PRINT_LEARN_MS, which CanPrint_Relearn()
 * may have overridden.
 */
bool CanPrint_IsLearning(void);

/*
 * Software identifier filter, applied before anything is printed.
 *
 * With an empty watch list every identifier is considered; adding even one
 * restricts output to the listed identifiers. The ignore list always wins.
 * This filters output rather than reception, which is the right lever here --
 * the bottleneck is the debug UART, not the SPI link. Once the identifier is
 * known, CanBus_SetIdFilter() moves the same decision into hardware.
 */
bool CanPrint_WatchId(uint32_t id, bool extended);
bool CanPrint_IgnoreId(uint32_t id, bool extended);
void CanPrint_ClearFilters(void);

/*
 * A watched identifier is exempt from the change budget and never mutes
 * itself. Once a candidate has been spotted, CanPrint_WatchId() on it is the
 * way to study press and release repeatedly without it falling silent.
 */

/*
 * Mute every identifier that has changed at all so far, in one go, regardless
 * of remaining budget. The manual equivalent of waiting for the budget to
 * drain -- call it when the bus is idle and the output goes quiet immediately.
 */
void CanPrint_MuteChanging(void);

/* Restore every muted identifier and refill all budgets. */
void CanPrint_UnmuteAll(void);

/* Override CAN_PRINT_CHANGE_BUDGET at runtime. 0 disables muting. */
void CanPrint_SetChangeBudget(uint8_t budget);

/*
 * Verbose change lines (default on). Applies only to frames caught after the
 * learn window -- the inventory printed during learning stays compact.
 *
 *   CHG  5BF STD[4] +48ms  15 00 01 A0  |....|  b0 15>00  b2 01>00
 *
 * Carries the identifier and its format, the DLC, the complete payload rather
 * than only the bytes that moved, the ASCII rendering, the gap since that
 * identifier was last seen, and the named byte deltas. For 29-bit diagnostic
 * identifiers the ISO 15765-4 target and source addresses are shown too.
 *
 * The inter-frame gap is the useful part when characterising a control: it is
 * what distinguishes a repeat counter ticking on a timer from one counting
 * discrete events.
 */
void CanPrint_SetVerbose(bool verbose);

/* Print every tracked identifier with its last payload and volatile mask. */
void CanPrint_DumpTable(void);

/*
 * Render `frame` into `buf`. Returns the number of characters written
 * (excluding the NUL), or a negative value on bad arguments.
 *
 * No RTOS or I/O dependencies, so this is also the piece to unit-test.
 */
int CanPrint_FormatFrame(char *buf, size_t size, const struct can_frame *frame);

/* Format `frame` and emit it through APP_DBG_MSG. */
void CanPrint_Frame(const struct can_frame *frame);

/*
 * Install the millisecond clock. Must be called before CanPrint_Feed(); the
 * change detector uses it for the learn window and for inter-frame gaps. The
 * source may wrap -- elapsed time is always computed by unsigned subtraction.
 *
 * On a bare-metal target HAL_GetTick (or equivalent) is the obvious choice;
 * the same tick source already backing mcp2515_port_t::ticks_ms will do.
 */
void CanPrint_SetClock(uint32_t (*now_ms)(void));

/*
 * Hand one received frame to the tracer. Prints according to the current mode.
 *
 * Collection strategy is the caller's: poll the controller from a task, drain
 * it from an INT-pin interrupt, or replay a capture in a host test.
 *
 * Be aware of the throughput ceiling when driving this from a live bus. The
 * controller has two RX buffers and a busy powertrain bus can exceed 1000
 * frames/s, while a 115200-baud UART fits roughly 160 trace lines per second.
 * Without a filter you are sampling the bus, not capturing it -- which is what
 * CAN_PRINT_MODE_CHANGES and the hardware acceptance filters exist to fix.
 */
void CanPrint_Feed(const struct can_frame *frame);

#ifdef __cplusplus
}
#endif

#endif /* CAN_PRINT_H_ */
