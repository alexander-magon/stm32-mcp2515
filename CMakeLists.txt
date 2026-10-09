cmake_minimum_required(VERSION 3.16)

project(mcp2515
    VERSION     1.3.1
    DESCRIPTION "Portable MCP2515 CAN controller driver"
    LANGUAGES   CXX
)

#
# The driver is hardware-agnostic: it performs all I/O through the
# mcp2515_port_t callback struct declared in mcp2515_port.h, which the
# consuming application fills in. Nothing here pulls in a vendor HAL, so the
# same target builds for any MCU (or for the host, for unit tests).
#
# Consumers only need:
#     target_link_libraries(my_app PRIVATE mcp2515)
#
# When this is consumed via add_subdirectory/FetchContent from a C-only
# project, remember that the parent must enable CXX -- e.g.
#     enable_language(CXX)
# -- because this library is C++ even though its port interface is plain C.
#

option(MCP2515_BUILD_AS_OBJECT
    "Build as an OBJECT library instead of STATIC (useful when the consumer \
links a custom startup/linker setup and wants the objects forwarded directly)"
    OFF)

#
# Optional components, under components/. They are hardware-independent like
# the driver itself, but unlike the driver they are not needed to talk to an
# MCP2515, so each can be switched off.
#
# They live in a subdirectory on purpose: the Arduino 1.0 library layout
# compiles only the root (and utility/), so a sketch is unaffected by their
# presence no matter what these options are set to.
#
option(MCP2515_BUILD_C_API
    "Build components/can_bus: a C facade owning the controller instance, so \
a C-only application never has to compile C++ of its own"
    ON)

option(MCP2515_BUILD_TRACE
    "Build components/can_print: frame formatting, ISO-TP/UDS decoding and \
change detection for reverse-engineering an unknown bus"
    OFF)

set(MCP2515_SOURCES mcp2515.cpp)

if(MCP2515_BUILD_C_API)
    list(APPEND MCP2515_SOURCES components/can_bus.cpp)
endif()

if(MCP2515_BUILD_TRACE)
    list(APPEND MCP2515_SOURCES components/can_print.c)
    # can_print.c is C; the driver and facade are C++.
    enable_language(C)
endif()

if(MCP2515_BUILD_AS_OBJECT)
    add_library(mcp2515 OBJECT ${MCP2515_SOURCES})
else()
    add_library(mcp2515 STATIC ${MCP2515_SOURCES})
endif()

add_library(mcp2515::mcp2515 ALIAS mcp2515)

target_include_directories(mcp2515 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/components>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(mcp2515 PUBLIC cxx_std_11)

# Freestanding targets have no exceptions/RTTI runtime worth paying for, and
# the driver uses neither. Only applied when we are clearly cross-compiling to
# a bare-metal target, so a host unit-test build keeps project defaults.
if(CMAKE_CROSSCOMPILING AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Guarded per-language: -fno-rtti and -fno-threadsafe-statics are C++-only,
    # and GCC warns on every C translation unit if they are applied globally.
    target_compile_options(mcp2515 PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
    )
endif()

set_target_properties(mcp2515 PROPERTIES
    POSITION_INDEPENDENT_CODE OFF
    CXX_EXTENSIONS            OFF
)

# Installation is only wired up for a standalone build; when embedded via
# add_subdirectory/FetchContent the consumer owns its own install rules.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    include(GNUInstallDirs)

    install(TARGETS mcp2515
        EXPORT  mcp2515Targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
    set(MCP2515_HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/mcp2515.h
        ${CMAKE_CURRENT_SOURCE_DIR}/mcp2515_port.h
        ${CMAKE_CURRENT_SOURCE_DIR}/can.h
    )
    if(MCP2515_BUILD_C_API)
        list(APPEND MCP2515_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/components/can_bus.h)
    endif()
    if(MCP2515_BUILD_TRACE)
        list(APPEND MCP2515_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/components/can_print.h)
    endif()
    install(FILES ${MCP2515_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    install(EXPORT mcp2515Targets
        FILE        mcp2515Targets.cmake
        NAMESPACE   mcp2515::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mcp2515
    )
endif()
