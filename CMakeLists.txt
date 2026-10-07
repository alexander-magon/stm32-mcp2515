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

if(MCP2515_BUILD_AS_OBJECT)
    add_library(mcp2515 OBJECT mcp2515.cpp)
else()
    add_library(mcp2515 STATIC mcp2515.cpp)
endif()

add_library(mcp2515::mcp2515 ALIAS mcp2515)

target_include_directories(mcp2515 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(mcp2515 PUBLIC cxx_std_11)

# Freestanding targets have no exceptions/RTTI runtime worth paying for, and
# the driver uses neither. Only applied when we are clearly cross-compiling to
# a bare-metal target, so a host unit-test build keeps project defaults.
if(CMAKE_CROSSCOMPILING AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mcp2515 PRIVATE
        -fno-rtti
        -fno-exceptions
        -fno-threadsafe-statics
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
    install(FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/mcp2515.h
        ${CMAKE_CURRENT_SOURCE_DIR}/mcp2515_port.h
        ${CMAKE_CURRENT_SOURCE_DIR}/can.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
    install(EXPORT mcp2515Targets
        FILE        mcp2515Targets.cmake
        NAMESPACE   mcp2515::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mcp2515
    )
endif()
