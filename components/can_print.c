/*
 * CAN frame tracing.
 *
 * Scope of the decoding below, deliberately: everything annotated here comes
 * from published standards that apply to any vehicle --
 *
 *   ISO 15765-4  diagnostic CAN identifiers (11-bit 0x7DF/0x7E0..0x7EF,
 *                29-bit 0x18DAxxyy / 0x18DBxxyy)
 *   ISO 15765-2  ISO-TP framing (single / first / consecutive / flow control)
 *   ISO 14229-1  UDS service identifiers and negative response codes
 *   SAE J1979    OBD-II service (mode) identifiers
 *
 * Manufacturer broadcast frames -- the 0x280/0x320/0x5A0-style IDs carrying
 * engine speed, wheel speeds and so on -- are intentionally NOT decoded. Those
 * assignments are specific to a platform and model year, and guessing at them
 * produces confident, wrong output. Fill in can_print_known_ids[] below once
 * you have confirmed IDs for your own car.
 */

#include "can_print.h"

#include <stdarg.h>   /* va_list in can_print_append; not implied by stdio.h */
#include <stdio.h>
#include <string.h>

/*
 * Millisecond clock, installed by the application. Until it is, elapsed-time
 * logic degrades gracefully: the learn window never expires, so nothing is
 * silently misreported as a change.
 */
static uint32_t (*can_print_clock)(void);

static uint32_t can_print_now(void)
{
    return (can_print_clock != NULL) ? can_print_clock() : 0U;
}

/* CanPrint_SetClock() is defined further down, next to the learn-window state
 * it has to re-base. */

/* Set to 0 to drop the ISO-TP/UDS tables and annotations (~1 kB of flash). */
#ifndef CAN_PRINT_DECODE_DIAGNOSTICS
#define CAN_PRINT_DECODE_DIAGNOSTICS 1
#endif


/* ------------------------------------------------------------------------- */
/* Vehicle-specific identifiers                                              */
/* ------------------------------------------------------------------------- */

/*
 * Names for broadcast IDs on YOUR vehicle. Empty on purpose -- see the file
 * header. To populate it, sniff with no filter, note which IDs appear, then
 * correlate each against something you can physically change (blinker, brake,
 * throttle) before naming it.
 *
 * Note that a VW has several buses behind the OBD-II port's gateway, running
 * at different rates (powertrain at 500 kbit/s, comfort/infotainment commonly
 * at 100 kbit/s), and the same numeric ID means different things on each. Keep
 * CAN_BUS_BITRATE and this table consistent with the bus actually tapped.
 */
typedef struct {
    uint32_t    id;
    bool        extended;
    const char *name;
} can_print_known_id_t;

static const can_print_known_id_t can_print_known_ids[] = {
    /* { 0x280, false, "ENGINE" }, */
};

static const char *can_print_lookup_id(uint32_t id, bool extended)
{
    for (size_t i = 0; i < sizeof(can_print_known_ids) / sizeof(can_print_known_ids[0]); i++) {
        if (can_print_known_ids[i].id == id &&
            can_print_known_ids[i].extended == extended) {
            return can_print_known_ids[i].name;
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------------- */
/* Standard diagnostic decoding                                              */
/* ------------------------------------------------------------------------- */

#if CAN_PRINT_DECODE_DIAGNOSTICS

/* ISO 15765-4 29-bit addressing: prio | 0xDA/0xDB | target | source */
#define ISO_TP_EFF_PHYSICAL   0xDAU
#define ISO_TP_EFF_FUNCTIONAL 0xDBU

typedef struct {
    uint8_t     id;
    const char *name;
} can_print_code_t;

/* SAE J1979 modes (0x01..0x0A) and ISO 14229 services. */
static const can_print_code_t can_print_services[] = {
    {0x01, "CurrentData"},   {0x02, "FreezeFrame"},   {0x03, "ReadDTC"},
    {0x04, "ClearDTC"},      {0x06, "MonitorResult"}, {0x07, "PendingDTC"},
    {0x09, "VehicleInfo"},   {0x0A, "PermanentDTC"},  {0x10, "Session"},
    {0x11, "ECUReset"},      {0x14, "ClearDiagInfo"}, {0x19, "ReadDTCInfo"},
    {0x22, "ReadDataByID"},  {0x23, "ReadMemByAddr"}, {0x27, "SecurityAccess"},
    {0x28, "CommControl"},   {0x2E, "WriteDataByID"}, {0x2F, "IOControl"},
    {0x31, "RoutineCtrl"},   {0x34, "ReqDownload"},   {0x36, "TransferData"},
    {0x37, "TransferExit"},  {0x3E, "TesterPresent"}, {0x85, "CtrlDTCSetting"},
};

/* ISO 14229-1 negative response codes, common subset. */
static const can_print_code_t can_print_nrcs[] = {
    {0x10, "generalReject"},     {0x11, "svcNotSupported"},
    {0x12, "subFuncNotSupported"}, {0x13, "badLength"},
    {0x21, "busyRepeatReq"},     {0x22, "condsNotCorrect"},
    {0x24, "reqSequenceError"},  {0x31, "reqOutOfRange"},
    {0x33, "securityDenied"},    {0x35, "invalidKey"},
    {0x36, "tooManyAttempts"},   {0x78, "responsePending"},
    {0x7E, "subFuncWrongSession"}, {0x7F, "svcWrongSession"},
};

static const char *can_print_lookup_code(const can_print_code_t *table, size_t n, uint8_t id)
{
    for (size_t i = 0; i < n; i++) {
        if (table[i].id == id) {
            return table[i].name;
        }
    }
    return NULL;
}

/*
 * True when this identifier is a standardised diagnostic address. The ISO-TP
 * and UDS decoding below is only valid for such frames -- applying it to an
 * arbitrary broadcast frame whose first byte happens to look like a PCI byte
 * would invent structure that is not there.
 */
static bool can_print_is_diag_id(const struct can_frame *frame, bool extended)
{
    const uint32_t id = frame->can_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK);

    if (extended) {
        const uint8_t format = (uint8_t) ((id >> 16) & 0xFFU);
        return format == ISO_TP_EFF_PHYSICAL || format == ISO_TP_EFF_FUNCTIONAL;
    }
    return id == 0x7DFU || (id >= 0x7E0U && id <= 0x7EFU);
}

#endif /* CAN_PRINT_DECODE_DIAGNOSTICS */

/* ------------------------------------------------------------------------- */
/* Formatting                                                                */
/* ------------------------------------------------------------------------- */

/* Append to buf, keeping *off within [0, size). Silently truncates. */
static void can_print_append(char *buf, size_t size, int *off, const char *fmt, ...)
{
    va_list args;
    int written;

    if (*off < 0 || (size_t) *off >= size) {
        return;
    }
    va_start(args, fmt);
    written = vsnprintf(buf + *off, size - (size_t) *off, fmt, args);
    va_end(args);

    if (written > 0) {
        *off += written;
        if ((size_t) *off >= size) {
            *off = (int) size - 1;
        }
    }
}

int CanPrint_FormatFrame(char *buf, size_t size, const struct can_frame *frame)
{
    int off = 0;

    if (buf == NULL || frame == NULL || size == 0U) {
        return -1;
    }

    const bool     extended = (frame->can_id & CAN_EFF_FLAG) != 0U;
    const bool     rtr      = (frame->can_id & CAN_RTR_FLAG) != 0U;
    const uint32_t id       = frame->can_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    /* A corrupt DLC must not walk off the end of data[]. */
    const uint8_t  dlc      = (frame->can_dlc > CAN_MAX_DLEN) ? CAN_MAX_DLEN : frame->can_dlc;

    /* Identifier, right-aligned so 11- and 29-bit IDs form columns. */
    can_print_append(buf, size, &off, extended ? "%08lX" : "%8lX", (unsigned long) id);
    can_print_append(buf, size, &off, " [%u]%s", (unsigned) frame->can_dlc, extended ? " E" : "");
    if (rtr) {
        can_print_append(buf, size, &off, " R");
    }

    /* Payload. An RTR frame carries none by definition, whatever the DLC says. */
    if (!rtr) {
        for (uint8_t i = 0; i < dlc; i++) {
            can_print_append(buf, size, &off, " %02X", frame->data[i]);
        }
        for (uint8_t i = dlc; i < CAN_MAX_DLEN; i++) {
            can_print_append(buf, size, &off, "   ");
        }

        can_print_append(buf, size, &off, " |");
        for (uint8_t i = 0; i < dlc; i++) {
            const uint8_t c = frame->data[i];
            can_print_append(buf, size, &off, "%c", (c >= 0x20U && c < 0x7FU) ? (char) c : '.');
        }
        can_print_append(buf, size, &off, "|");
    }

    const char *known = can_print_lookup_id(id, extended);
    if (known != NULL) {
        can_print_append(buf, size, &off, " %s", known);
    }

#if CAN_PRINT_DECODE_DIAGNOSTICS
    if (rtr || !can_print_is_diag_id(frame, extended)) {
        return off;
    }

    /* Addressing */
    if (extended) {
        const uint8_t format = (uint8_t) ((id >> 16) & 0xFFU);
        can_print_append(buf, size, &off, " diag.%s tgt=%02lX src=%02lX",
                         format == ISO_TP_EFF_PHYSICAL ? "phys" : "func",
                         (unsigned long) ((id >> 8) & 0xFFU),
                         (unsigned long) (id & 0xFFU));
    } else if (id == 0x7DFU) {
        can_print_append(buf, size, &off, " OBD2.func");
    } else if (id <= 0x7E7U) {
        can_print_append(buf, size, &off, " OBD2.req ECU%lu", (unsigned long) (id - 0x7E0U));
    } else {
        can_print_append(buf, size, &off, " OBD2.resp ECU%lu", (unsigned long) (id - 0x7E8U));
    }

    if (dlc == 0U) {
        return off;
    }

    /* ISO-TP protocol control information, and the service inside it. */
    const uint8_t pci     = (uint8_t) (frame->data[0] >> 4);
    uint8_t       sid_idx = 0U;   /* 0 = no service byte in this frame */

    switch (pci) {
        case 0x0U:
            can_print_append(buf, size, &off, " SF:%u", (unsigned) (frame->data[0] & 0x0FU));
            sid_idx = 1U;
            break;
        case 0x1U:
            if (dlc >= 2U) {
                can_print_append(buf, size, &off, " FF:%u",
                                 (unsigned) (((frame->data[0] & 0x0FU) << 8) | frame->data[1]));
                sid_idx = 2U;
            }
            break;
        case 0x2U:
            can_print_append(buf, size, &off, " CF#%u", (unsigned) (frame->data[0] & 0x0FU));
            break;
        case 0x3U: {
            static const char *const fs[] = {"CTS", "WAIT", "OVFLW"};
            const uint8_t flow = frame->data[0] & 0x0FU;
            can_print_append(buf, size, &off, " FC:%s",
                             flow < 3U ? fs[flow] : "?");
            break;
        }
        default:
            break;
    }

    if (sid_idx == 0U || dlc <= sid_idx) {
        return off;
    }

    const uint8_t sid = frame->data[sid_idx];

    if (sid == 0x7FU) {
        /* Negative response: [7F] [requested SID] [NRC] */
        const uint8_t req = (dlc > sid_idx + 1U) ? frame->data[sid_idx + 1U] : 0U;
        const uint8_t nrc = (dlc > sid_idx + 2U) ? frame->data[sid_idx + 2U] : 0U;
        const char   *req_name =
            can_print_lookup_code(can_print_services,
                                  sizeof(can_print_services) / sizeof(can_print_services[0]), req);
        const char   *nrc_name =
            can_print_lookup_code(can_print_nrcs,
                                  sizeof(can_print_nrcs) / sizeof(can_print_nrcs[0]), nrc);

        can_print_append(buf, size, &off, " UDS- %02X", req);
        if (req_name != NULL) {
            can_print_append(buf, size, &off, ":%s", req_name);
        }
        can_print_append(buf, size, &off, " nrc=%02X", nrc);
        if (nrc_name != NULL) {
            can_print_append(buf, size, &off, ":%s", nrc_name);
        }
        return off;
    }

    /* A positive response echoes the request SID with bit 6 set. */
    const bool    positive = (sid >= 0x40U && sid <= 0xBFU);
    const uint8_t base     = positive ? (uint8_t) (sid - 0x40U) : sid;
    const char   *name =
        can_print_lookup_code(can_print_services,
                              sizeof(can_print_services) / sizeof(can_print_services[0]), base);

    can_print_append(buf, size, &off, " UDS%s %02X", positive ? "+" : ">", base);
    if (name != NULL) {
        can_print_append(buf, size, &off, ":%s", name);
    }
#endif /* CAN_PRINT_DECODE_DIAGNOSTICS */

    return off;
}

/*
 * Trace output goes through printf directly rather than APP_DBG_MSG.
 *
 * PRINT_MESG_DBG prefixes every call with "\r\n [file][function][line] ",
 * which measured 38 characters against 51 characters of actual frame on the
 * first capture from this board -- the prefix and its leading newline were
 * costing more UART bandwidth than the data. At this line rate that directly
 * translates into dropped frames, so the prefix is not worth paying for here.
 */
#define CAN_OUT(...) do { CAN_PRINT_PRINTF(__VA_ARGS__); } while (0)

void CanPrint_Frame(const struct can_frame *frame)
{
    char line[CAN_PRINT_LINE_MAX];

    if (CanPrint_FormatFrame(line, sizeof(line), frame) < 0) {
        return;
    }
    CAN_OUT("%s\r\n", line);
}

/* ------------------------------------------------------------------------- */
/* Change detection                                                          */
/* ------------------------------------------------------------------------- */

typedef struct {
    uint32_t id;        /* identifier, flag bits stripped */
    uint8_t  data[CAN_MAX_DLEN];
    uint8_t  dlc;
    uint8_t  vol_mask;  /* bit i: byte i changed during the learn window */
    uint8_t  extended;
    uint8_t  used;
    uint8_t  spent;     /* change lines printed so far */
    uint8_t  muted;     /* budget exhausted, or muted explicitly */
    uint32_t last_tick; /* when this identifier was last seen */
} can_print_entry_t;

static can_print_entry_t can_print_table[CAN_PRINT_MAX_IDS];
static uint16_t          can_print_table_used;

/* Filter lists. id == 0 with used == 0 marks a free slot. */
typedef struct {
    uint32_t id;
    uint8_t  extended;
    uint8_t  used;
} can_print_filter_t;

static can_print_filter_t can_print_watch[16];
static can_print_filter_t can_print_ignore[16];

static can_print_mode_t can_print_mode  = CAN_PRINT_MODE_RAW;
static uint32_t         can_print_learn_until;
static bool             can_print_learning;
static uint8_t          can_print_budget = CAN_PRINT_CHANGE_BUDGET;
static bool             can_print_verbose_on = true;

/* Length of the current learn window, kept so CanPrint_SetClock() can re-base it. */
static uint32_t         can_print_learn_window = CAN_PRINT_LEARN_MS;

void CanPrint_SetClock(uint32_t (*now_ms)(void))
{
    can_print_clock = now_ms;

    /*
     * Re-base a learn window that was armed before a clock existed.
     *
     * CanPrint_SetMode() may legitimately run before the clock is installed,
     * in which case can_print_now() returned 0 and the deadline became an
     * absolute tick rather than an offset from now -- truncating the window,
     * or expiring it outright if the system had already been up that long.
     * Re-basing here makes the two calls order-independent.
     */
    if (can_print_learning && now_ms != NULL) {
        can_print_learn_until = now_ms() + can_print_learn_window;
    }
}

static bool can_print_list_has(const can_print_filter_t *list, size_t n,
                               uint32_t id, bool extended)
{
    for (size_t i = 0; i < n; i++) {
        if (list[i].used && list[i].id == id && list[i].extended == (uint8_t) extended) {
            return true;
        }
    }
    return false;
}

static bool can_print_list_add(can_print_filter_t *list, size_t n,
                               uint32_t id, bool extended)
{
    if (can_print_list_has(list, n, id, extended)) {
        return true;
    }
    for (size_t i = 0; i < n; i++) {
        if (!list[i].used) {
            list[i].id       = id;
            list[i].extended = (uint8_t) extended;
            list[i].used     = 1U;
            return true;
        }
    }
    return false;
}

static bool can_print_list_empty(const can_print_filter_t *list, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if (list[i].used) {
            return false;
        }
    }
    return true;
}

static bool can_print_accepts(uint32_t id, bool extended)
{
    const size_t n = sizeof(can_print_watch) / sizeof(can_print_watch[0]);

    if (can_print_list_has(can_print_ignore, n, id, extended)) {
        return false;
    }
    if (can_print_list_empty(can_print_watch, n)) {
        return true;
    }
    return can_print_list_has(can_print_watch, n, id, extended);
}

/*
 * Linear scan. With ~95 live identifiers and a 128-entry table this is a few
 * hundred comparisons per frame, which is nothing next to the SPI transfer
 * that delivered the frame.
 */
static can_print_entry_t *can_print_find(uint32_t id, bool extended)
{
    for (uint16_t i = 0; i < can_print_table_used; i++) {
        if (can_print_table[i].id == id &&
            can_print_table[i].extended == (uint8_t) extended) {
            return &can_print_table[i];
        }
    }
    return NULL;
}

static void can_print_id_field(uint32_t id, bool extended)
{
    CAN_OUT(extended ? "%08lX" : "%8lX", (unsigned long) id);
}

/*
 * Full context for one frame caught after the learn window.
 *
 * A note on "address": on a broadcast frame such as 0x5BF there is no separate
 * controller and device address to report. The 11-bit identifier IS the
 * address -- it names the message, and by convention the module that owns it.
 * Only the diagnostic identifiers carry explicit target/source bytes, decoded
 * below when present.
 */
static void can_print_verbose(const can_print_entry_t *entry,
                              const struct can_frame *frame,
                              uint32_t id, bool extended, uint8_t dlc,
                              uint8_t report, uint32_t now)
{
    can_print_id_field(id, extended);
    CAN_OUT(" %s[%u]", extended ? "EXT" : "STD", (unsigned) dlc);

    /* Gap since this identifier was last seen; blank the first time. */
    if (entry->last_tick != 0U) {
        CAN_OUT(" +%lums", (unsigned long) (now - entry->last_tick));
    } else {
        CAN_OUT(" ------");
    }

#if CAN_PRINT_DECODE_DIAGNOSTICS
    if (extended) {
        const uint8_t format = (uint8_t) ((id >> 16) & 0xFFU);
        if (format == ISO_TP_EFF_PHYSICAL || format == ISO_TP_EFF_FUNCTIONAL) {
            CAN_OUT(" %s tgt=%02lX src=%02lX",
                    format == ISO_TP_EFF_PHYSICAL ? "phys" : "func",
                    (unsigned long) ((id >> 8) & 0xFFU),
                    (unsigned long) (id & 0xFFU));
        }
    }
#endif

    /* Complete payload, not just what moved. */
    CAN_OUT(" ");
    for (uint8_t i = 0; i < dlc; i++) {
        CAN_OUT(" %02X", frame->data[i]);
    }
    CAN_OUT("  |");
    for (uint8_t i = 0; i < dlc; i++) {
        const uint8_t c = frame->data[i];
        CAN_OUT("%c", (c >= 0x20U && c < 0x7FU) ? (char) c : '.');
    }
    CAN_OUT("|");

    /* Named deltas, so the moving byte is unambiguous. */
    for (uint8_t i = 0; i < dlc; i++) {
        if ((report & (uint8_t) (1U << i)) != 0U) {
            CAN_OUT("  b%u %02X>%02X", (unsigned) i, entry->data[i], frame->data[i]);
        }
    }
    CAN_OUT("\r\n");
}

static void can_print_process(const struct can_frame *frame)
{
    const bool     extended = (frame->can_id & CAN_EFF_FLAG) != 0U;
    const bool     rtr      = (frame->can_id & CAN_RTR_FLAG) != 0U;
    const uint32_t id       = frame->can_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    const uint8_t  dlc      = (frame->can_dlc > CAN_MAX_DLEN) ? CAN_MAX_DLEN : frame->can_dlc;

    if (rtr || !can_print_accepts(id, extended)) {
        return;
    }

    /* Close the learn window on the first frame seen after it expires. */
    if (can_print_learning && (can_print_now() - can_print_learn_until) < 0x80000000U) {
        uint16_t with_volatile = 0U;
        uint16_t masked_bytes  = 0U;

        can_print_learning = false;
        for (uint16_t i = 0; i < can_print_table_used; i++) {
            const uint8_t m = can_print_table[i].vol_mask;
            if (m != 0U) {
                with_volatile++;
                for (uint8_t b = 0; b < CAN_MAX_DLEN; b++) {
                    if ((m & (uint8_t) (1U << b)) != 0U) {
                        masked_bytes++;
                    }
                }
            }
        }
        CAN_OUT("-- learn done: %u ids, %u with churn, %u bytes masked --\r\n"
                "-- now press the control you are looking for --\r\n",
                (unsigned) can_print_table_used, (unsigned) with_volatile,
                (unsigned) masked_bytes);
    }

    can_print_entry_t *entry = can_print_find(id, extended);

    if (entry == NULL) {
        if (can_print_table_used >= CAN_PRINT_MAX_IDS) {
            return; /* table full; reported once by the monitor task */
        }
        entry            = &can_print_table[can_print_table_used++];
        entry->id        = id;
        entry->extended  = (uint8_t) extended;
        entry->dlc       = dlc;
        entry->vol_mask  = 0U;
        entry->used      = 1U;
        entry->spent     = 0U;
        entry->muted     = 0U;
        entry->last_tick = 0U;
        memcpy(entry->data, frame->data, CAN_MAX_DLEN);

        /* Announce every identifier once: this is the device inventory. */
        CAN_OUT("NEW ");
        if (can_print_verbose_on && !can_print_learning) {
            /* An identifier appearing after the learn window is itself a
             * catch, so give it the same detail as a change. */
            can_print_verbose(entry, frame, id, extended, dlc, 0U, can_print_now());
        } else {
            can_print_id_field(id, extended);
            CAN_OUT(" [%u]", (unsigned) dlc);
            for (uint8_t i = 0; i < dlc; i++) {
                CAN_OUT(" %02X", frame->data[i]);
            }
            CAN_OUT("\r\n");
        }
        entry->last_tick = can_print_now();
        return;
    }

    /* Which bytes differ from what we last saw for this identifier. */
    uint8_t changed = 0U;
    for (uint8_t i = 0; i < dlc; i++) {
        if (frame->data[i] != entry->data[i]) {
            changed |= (uint8_t) (1U << i);
        }
    }

    if (can_print_learning) {
        /* Record the churn, print nothing. */
        entry->vol_mask |= changed;
        memcpy(entry->data, frame->data, CAN_MAX_DLEN);
        entry->dlc = dlc;
        return;
    }

    const uint8_t report = changed & (uint8_t) ~entry->vol_mask;

    /*
     * An explicitly watched identifier ignores the budget entirely -- that is
     * the point of putting it on the watch list.
     */
    const bool exempt = !can_print_list_empty(can_print_watch,
                                              sizeof(can_print_watch) /
                                              sizeof(can_print_watch[0])) &&
                        can_print_list_has(can_print_watch,
                                           sizeof(can_print_watch) /
                                           sizeof(can_print_watch[0]), id, extended);

    if (report != 0U && (!entry->muted || exempt)) {
        CAN_OUT("CHG ");

        if (can_print_verbose_on) {
            can_print_verbose(entry, frame, id, extended, dlc, report, can_print_now());
        } else {
            can_print_id_field(id, extended);
            for (uint8_t i = 0; i < dlc; i++) {
                if ((report & (uint8_t) (1U << i)) != 0U) {
                    CAN_OUT(" [%u]%02X>%02X", (unsigned) i, entry->data[i], frame->data[i]);
                }
            }
            CAN_OUT("\r\n");
        }

        if (!exempt && can_print_budget != 0U) {
            entry->spent++;
            if (entry->spent >= can_print_budget) {
                entry->muted = 1U;
                CAN_OUT("MUTE ");
                can_print_id_field(id, extended);
                CAN_OUT(" - keeps changing, silenced\r\n");
            }
        }
    }

    /* Store every byte, including volatile ones, so the next diff is correct. */
    memcpy(entry->data, frame->data, CAN_MAX_DLEN);
    entry->dlc       = dlc;
    entry->last_tick = can_print_now();
}

void CanPrint_SetVerbose(bool verbose)
{
    can_print_verbose_on = verbose;
}

bool CanPrint_IsLearning(void)
{
    return can_print_learning;
}

void CanPrint_SetMode(can_print_mode_t mode)
{
    can_print_mode = mode;
    if (mode == CAN_PRINT_MODE_CHANGES) {
        memset(can_print_table, 0, sizeof(can_print_table));
        can_print_table_used = 0U;
        CanPrint_Relearn(0U);
    }
}

void CanPrint_Relearn(uint32_t learn_ms)
{
    const uint32_t window = (learn_ms != 0U) ? learn_ms : CAN_PRINT_LEARN_MS;

    for (uint16_t i = 0; i < can_print_table_used; i++) {
        can_print_table[i].vol_mask = 0U;
        can_print_table[i].spent    = 0U;
        can_print_table[i].muted    = 0U;
    }
    can_print_learn_window = window;
    can_print_learn_until  = can_print_now() + window;
    can_print_learning     = true;

    CAN_OUT("-- learning for %lu ms: leave the car alone --\r\n",
            (unsigned long) window);
}

bool CanPrint_WatchId(uint32_t id, bool extended)
{
    return can_print_list_add(can_print_watch,
                              sizeof(can_print_watch) / sizeof(can_print_watch[0]),
                              id, extended);
}

bool CanPrint_IgnoreId(uint32_t id, bool extended)
{
    return can_print_list_add(can_print_ignore,
                              sizeof(can_print_ignore) / sizeof(can_print_ignore[0]),
                              id, extended);
}

void CanPrint_ClearFilters(void)
{
    memset(can_print_watch, 0, sizeof(can_print_watch));
    memset(can_print_ignore, 0, sizeof(can_print_ignore));
}

void CanPrint_MuteChanging(void)
{
    uint16_t muted = 0U;

    for (uint16_t i = 0; i < can_print_table_used; i++) {
        /* "Has moved at all": either it spent budget after the learn window,
         * or the learn window itself caught a byte moving. */
        if (can_print_table[i].muted == 0U &&
            (can_print_table[i].spent != 0U || can_print_table[i].vol_mask != 0U)) {
            can_print_table[i].muted = 1U;
            muted++;
        }
    }
    CAN_OUT("-- muted %u changing ids, %u still silent --\r\n",
            (unsigned) muted, (unsigned) (can_print_table_used - muted));
}

void CanPrint_UnmuteAll(void)
{
    for (uint16_t i = 0; i < can_print_table_used; i++) {
        can_print_table[i].muted = 0U;
        can_print_table[i].spent = 0U;
    }
    CAN_OUT("-- unmuted %u ids --\r\n", (unsigned) can_print_table_used);
}

void CanPrint_SetChangeBudget(uint8_t budget)
{
    can_print_budget = budget;
}

void CanPrint_DumpTable(void)
{
    CAN_OUT("-- %u identifiers tracked --\r\n", (unsigned) can_print_table_used);
    for (uint16_t i = 0; i < can_print_table_used; i++) {
        const can_print_entry_t *e = &can_print_table[i];

        CAN_OUT("    ");
        can_print_id_field(e->id, e->extended != 0U);
        CAN_OUT(" [%u]", (unsigned) e->dlc);
        for (uint8_t b = 0; b < e->dlc; b++) {
            CAN_OUT(" %02X", e->data[b]);
        }
        CAN_OUT("  churn:");
        for (uint8_t b = 0; b < CAN_MAX_DLEN; b++) {
            CAN_OUT("%c", ((e->vol_mask & (uint8_t) (1U << b)) != 0U) ? 'V' : '.');
        }
        CAN_OUT("%s\r\n", (e->muted != 0U) ? "  MUTED" : "");
    }
}

/* ------------------------------------------------------------------------- */
/* Entry point                                                               */
/* ------------------------------------------------------------------------- */

void CanPrint_Feed(const struct can_frame *frame)
{
    if (frame == NULL) {
        return;
    }
    if (can_print_mode == CAN_PRINT_MODE_CHANGES) {
        can_print_process(frame);
    } else {
        CanPrint_Frame(frame);
    }
}
