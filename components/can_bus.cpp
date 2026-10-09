/*
 * Owns the MCP2515 instance and exposes it to the C application.
 *
 * The only C++ translation unit in this project. It exists purely so the
 * driver's C++ API can be reached from C; all hardware specifics live in
 * can_port.c and all protocol logic lives in the fetched driver.
 */

#include "can_bus.h"

#include "mcp2515.h"
#include "mcp2515_port.h"

#include <new>

extern "C" {
/*
 * Supplied by the application: returns a fully populated port describing how
 * to reach the controller on this board. Nothing here knows or cares what is
 * behind it.
 */
const mcp2515_port_t *CanPort_Get(void);
}

namespace {

/*
 * Constructed with placement new into static storage rather than declared as a
 * file-scope object, so no dynamic static initialiser is emitted. That keeps
 * the firmware free of .init_array entries, which matters here because the
 * executable is linked with a custom startup and linker script.
 */
alignas(MCP2515) uint8_t g_storage[sizeof(MCP2515)];
MCP2515 *g_mcp = nullptr;

can_bus_status_t map_error(MCP2515::ERROR err)
{
    switch (err) {
        case MCP2515::ERROR_OK:        return CAN_BUS_OK;
        case MCP2515::ERROR_NOMSG:     return CAN_BUS_ERR_NO_MSG;
        case MCP2515::ERROR_ALLTXBUSY:
        case MCP2515::ERROR_FAILTX:    return CAN_BUS_ERR_TX;
        case MCP2515::ERROR_FAILINIT:  return CAN_BUS_ERR_INIT;
        default:                       return CAN_BUS_ERR_FAIL;
    }
}

}  // namespace

extern "C" can_bus_status_t CanBus_Init(void)
{
    if (g_mcp == nullptr) {
        g_mcp = new (g_storage) MCP2515(CanPort_Get());
    }

    if (g_mcp->reset() != MCP2515::ERROR_OK) {
        return CAN_BUS_ERR_INIT;
    }
    if (g_mcp->setBitrate(CAN_BUS_BITRATE, CAN_BUS_MCP_CLOCK) != MCP2515::ERROR_OK) {
        return CAN_BUS_ERR_INIT;
    }
    if (g_mcp->setNormalMode() != MCP2515::ERROR_OK) {
        return CAN_BUS_ERR_INIT;
    }
    return CAN_BUS_OK;
}

extern "C" can_bus_status_t CanBus_SetLoopback(void)
{
    if (g_mcp == nullptr) {
        return CAN_BUS_ERR_INIT;
    }
    return map_error(g_mcp->setLoopbackMode());
}

extern "C" can_bus_status_t CanBus_SetListenOnly(void)
{
    if (g_mcp == nullptr) {
        return CAN_BUS_ERR_INIT;
    }
    return map_error(g_mcp->setListenOnlyMode());
}

extern "C" can_bus_status_t CanBus_SetIdFilter(uint32_t mask, uint32_t id, bool extended)
{
    if (g_mcp == nullptr) {
        return CAN_BUS_ERR_INIT;
    }

    /*
     * Filters only take effect in configuration mode, and the controller must
     * be returned to a running mode afterwards. Preserve listen-only across
     * the change rather than silently dropping back to normal, which on a
     * vehicle bus would start ACKing traffic the caller meant only to observe.
     */
    const bool was_listen_only =
        (g_mcp->getControlRegister() & 0xE0) == 0x60; /* CANCTRL REQOP = listen-only */

    if (g_mcp->setConfigMode() != MCP2515::ERROR_OK) {
        return CAN_BUS_ERR_FAIL;
    }

    const MCP2515::MASK masks[] = {MCP2515::MASK0, MCP2515::MASK1};
    for (MCP2515::MASK m : masks) {
        if (g_mcp->setFilterMask(m, extended, mask) != MCP2515::ERROR_OK) {
            return CAN_BUS_ERR_FAIL;
        }
    }

    const MCP2515::RXF filters[] = {MCP2515::RXF0, MCP2515::RXF1, MCP2515::RXF2,
                                    MCP2515::RXF3, MCP2515::RXF4, MCP2515::RXF5};
    for (MCP2515::RXF f : filters) {
        if (g_mcp->setFilter(f, extended, id) != MCP2515::ERROR_OK) {
            return CAN_BUS_ERR_FAIL;
        }
    }

    return map_error(was_listen_only ? g_mcp->setListenOnlyMode()
                                     : g_mcp->setNormalMode());
}

extern "C" can_bus_status_t CanBus_SetFilters(const can_bus_filter_t *filters, uint8_t count)
{
    if (g_mcp == nullptr || filters == nullptr ||
        count == 0U || count > CAN_BUS_MAX_FILTERS) {
        return CAN_BUS_ERR_FAIL;
    }

    /* One mask serves filters 0-1 and the other 2-5, and the two mask layouts
     * differ between 11- and 29-bit matching, so the set has to be uniform. */
    for (uint8_t i = 1U; i < count; i++) {
        if (filters[i].extended != filters[0].extended) {
            return CAN_BUS_ERR_FAIL;
        }
    }

    const bool     extended = filters[0].extended;
    const uint32_t mask     = extended ? 0x1FFFFFFFUL : 0x7FFUL;

    const bool was_listen_only =
        (g_mcp->getControlRegister() & 0xE0) == 0x60; /* CANCTRL REQOP = listen-only */

    if (g_mcp->setConfigMode() != MCP2515::ERROR_OK) {
        return CAN_BUS_ERR_FAIL;
    }

    /* Full mask: every identifier bit must match, so each filter is an exact
     * identifier rather than a range. */
    const MCP2515::MASK masks[] = {MCP2515::MASK0, MCP2515::MASK1};
    for (MCP2515::MASK m : masks) {
        if (g_mcp->setFilterMask(m, extended, mask) != MCP2515::ERROR_OK) {
            return CAN_BUS_ERR_FAIL;
        }
    }

    const MCP2515::RXF slots[] = {MCP2515::RXF0, MCP2515::RXF1, MCP2515::RXF2,
                                  MCP2515::RXF3, MCP2515::RXF4, MCP2515::RXF5};
    for (uint8_t i = 0U; i < CAN_BUS_MAX_FILTERS; i++) {
        /* Pad the spare slots by repeating the first entry. Leaving them at
         * zero under a full mask would quietly accept identifier 0x000. */
        const uint32_t id = (i < count) ? filters[i].id : filters[0].id;

        if (g_mcp->setFilter(slots[i], extended, id) != MCP2515::ERROR_OK) {
            return CAN_BUS_ERR_FAIL;
        }
    }

    return map_error(was_listen_only ? g_mcp->setListenOnlyMode()
                                     : g_mcp->setNormalMode());
}

extern "C" can_bus_status_t CanBus_AcceptAll(void)
{
    /* An all-zero mask makes every ID bit a don't-care. */
    return CanBus_SetIdFilter(0U, 0U, false);
}

extern "C" can_bus_status_t CanBus_Send(const struct can_frame *frame)
{
    if (g_mcp == nullptr || frame == nullptr) {
        return CAN_BUS_ERR_FAIL;
    }
    return map_error(g_mcp->sendMessage(frame));
}

extern "C" can_bus_status_t CanBus_Receive(struct can_frame *frame)
{
    if (g_mcp == nullptr || frame == nullptr) {
        return CAN_BUS_ERR_FAIL;
    }
    return map_error(g_mcp->readMessage(frame));
}

extern "C" bool CanBus_HasPending(void)
{
    return g_mcp != nullptr && g_mcp->checkReceive();
}

extern "C" uint8_t CanBus_GetErrorFlags(void)
{
    return g_mcp != nullptr ? g_mcp->getErrorFlags() : 0U;
}

extern "C" void CanBus_ClearOverflow(void)
{
    if (g_mcp != nullptr) {
        g_mcp->clearRXnOVRFlags();
    }
}
