
cmake_minimum_required(VERSION 2.8.12)

include(GNUInstallDirs)

project(olas-cli)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake")

#===============================================================================
#= User Options
#-------------------------------------------------------------------------------

option(DISABLE_WERROR "Build without -Werror" FALSE)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Debug CACHE STRING
	"Choose the type of build (None, Debug, Release, RelWithDebInfo, MinSizeRel)."
	FORCE)
endif()

#===============================================================================
#= Dependencies
#-------------------------------------------------------------------------------

#list(APPEND PKGDEPS glib-2.0>=2.28.0)
#list(APPEND PKGDEPS glibmm-2.4>=2.28.0)
#list(APPEND PKGDEPS libsigrokcxx>=0.6.0)

if(ENABLE_DECODE)
	list(APPEND PKGDEPS libsigrokdecode>=0.6.0)
endif()


# Find the platform's thread library (needed for C++11 threads).
# This will set ${CMAKE_THREAD_LIBS_INIT} to the correct, OS-specific value.
find_package(Threads REQUIRED)

list(APPEND PKGDEPS glib-2.0>=2.28.0)

#===============================================================================
#= Sources
#-------------------------------------------------------------------------------

include_directories(
#sigrok-cli
/usr/include/glib-2.0
/usr/lib/glib-2.0/include
/usr/include/libusb-1.0/
libsigrok/src/
/usr/include/tirpc
libsigrok
libsigrok/include/libsigrok/
/usr/include/libftdi1
)

set(sigrok_SOURCES
libsigrok/src/transform/invert.c
libsigrok/src/transform/nop.c
libsigrok/src/transform/scale.c
libsigrok/src/transform/transform.c
libsigrok/src/analog.c
libsigrok/src/backend.c
libsigrok/src/conversion.c
libsigrok/src/device.c
libsigrok/src/drivers.c
libsigrok/src/error.c
libsigrok/src/ezusb.c
libsigrok/src/fallback.c
libsigrok/src/hwdriver.c
libsigrok/src/log.c
libsigrok/src/resource.c
libsigrok/src/serial.c
libsigrok/src/serial_hid.c
libsigrok/src/serial_libsp.c
# If you do not have bluez, remove these
libsigrok/src/serial_bt.c
libsigrok/src/bt/bt_bluez.c

libsigrok/src/session.c
libsigrok/src/session_driver.c
libsigrok/src/session_file.c
libsigrok/src/soft-trigger.c
libsigrok/src/std.c
libsigrok/src/strutil.c
libsigrok/src/sw_limits.c
libsigrok/src/trigger.c
libsigrok/src/usb.c
libsigrok/src/version.c
libsigrok/src/input/binary.c
libsigrok/src/input/chronovu_la8.c
libsigrok/src/input/csv.c
libsigrok/src/input/input.c
libsigrok/src/input/logicport.c
libsigrok/src/input/raw_analog.c
libsigrok/src/input/trace32_ad.c
libsigrok/src/input/wav.c
libsigrok/src/input/vcd.c
libsigrok/src/input/null.c
#libsigrok/src/scpi/helpers.c
libsigrok/src/scpi/scpi.c
#libsigrok/src/scpi/scpi_libgpib.c
libsigrok/src/scpi/scpi_serial.c
libsigrok/src/scpi/scpi_tcp.c
libsigrok/src/scpi/scpi_usbtmc_libusb.c
#libsigrok/src/scpi/scpi_visa.c
libsigrok/src/scpi/scpi_vxi.c
libsigrok/src/scpi/vxi_clnt.c
libsigrok/src/scpi/vxi_xdr.c
libsigrok/src/output/analog.c
libsigrok/src/output/ascii.c
libsigrok/src/output/binary.c
libsigrok/src/output/bits.c
libsigrok/src/output/chronovu_la8.c
libsigrok/src/output/csv.c
libsigrok/src/output/hex.c
libsigrok/src/output/ols.c
libsigrok/src/output/wavedrom.c
libsigrok/src/output/output.c
libsigrok/src/output/srzip.c
libsigrok/src/output/wav.c
libsigrok/src/output/vcd.c
libsigrok/src/output/null.c
libsigrok/src/modbus/modbus.c
libsigrok/src/modbus/modbus_serial_rtu.c
libsigrok/src/driver_list_start.c
libsigrok/src/hardware/openbench-logic-sniffer/api.c
libsigrok/src/hardware/openbench-logic-sniffer/protocol.c
libsigrok/src/hardware/rigol-ds/api.c
libsigrok/src/hardware/rigol-ds/protocol.c
libsigrok/src/hardware/pipistrello-ols/api.c
libsigrok/src/hardware/pipistrello-ols/protocol.c
libsigrok/src/driver_list_stop.c
)

set(cli_SOURCES
 sigrok-cli/anykey.c
 sigrok-cli/decode.c
 sigrok-cli/device.c
 sigrok-cli/input.c
 sigrok-cli/main.c
 sigrok-cli/options.c
 sigrok-cli/parsers.c
 sigrok-cli/session.c
  sigrok-cli/output.c
 sigrok-cli/show.c
)


#===============================================================================
#= Global Definitions
#-------------------------------------------------------------------------------

add_definitions(-DQT_NO_KEYWORDS)
add_definitions(-D__STDC_LIMIT_MACROS)
add_definitions(-DSC_PACKAGE_VERSION_STRING="test ver")
add_definitions(-Wall -Wextra)
#add_definitions(-std=c++11)

if(ENABLE_DECODE)
	add_definitions(-DENABLE_DECODE)
endif()

#if(NOT DISABLE_WERROR)
#	add_definitions(-Werror)
#endif()

if(ENABLE_SIGNALS)
	add_definitions(-DENABLE_SIGNALS)
endif()

#===============================================================================
#= Global Include Directories
#-------------------------------------------------------------------------------

#include_directories(
#	${CMAKE_CURRENT_BINARY_DIR}
#	${CMAKE_CURRENT_SOURCE_DIR}
#)

add_executable(olas-cli
${cli_SOURCES}
${sigrok_SOURCES}
)

target_link_libraries(olas-cli rt tirpc pthread usb-1.0 serialport glib-2.0 m zip ftdi1)




