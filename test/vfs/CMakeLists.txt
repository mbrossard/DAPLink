cmake_minimum_required(VERSION 3.15)

project(vfs_test VERSION 1.0 LANGUAGES C)

add_compile_definitions(DAPLINK_DEBUG DAPLINK_DEBUG_STDOUT
    DEBUG_FLASH_MANAGER=1 DEBUG_INTELHEX=1 DEBUG_IAP_FLASH=1
    DAPLINK_DISABLE_STATIC_CHECKS)

set(IF_DEFS DAPLINK_IF DAPLINK_BUILD_KEY=DAPLINK_BUILD_KEY_IF)
set(BL_DEFS DAPLINK_BL DAPLINK_BUILD_KEY=DAPLINK_BUILD_KEY_BL)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../source/daplink
    ${CMAKE_CURRENT_SOURCE_DIR}/../../source/daplink/drag-n-drop
    ${CMAKE_CURRENT_SOURCE_DIR}/../../source/daplink/settings
    ${CMAKE_CURRENT_SOURCE_DIR}/../../source/target
    ${CMAKE_CURRENT_SOURCE_DIR}/../../source/cmsis-core
    ${CMAKE_CURRENT_SOURCE_DIR}/../../source/rtos_none
    ${CMAKE_CURRENT_SOURCE_DIR}/../../source/hic_hal)

add_compile_options(-Wall)

set(SOURCES main.c intf_flash.c util.c
    ../../source/daplink/crc32.c
    ../../source/daplink/daplink_debug.c
    ../../source/daplink/drag-n-drop/iap_flash_intf.c
    ../../source/daplink/drag-n-drop/intelhex.c
    ../../source/daplink/drag-n-drop/file_stream.c
    ../../source/daplink/drag-n-drop/flash_decoder.c
    ../../source/daplink/drag-n-drop/flash_intf.c
    ../../source/daplink/drag-n-drop/flash_manager.c
    ../../source/daplink/drag-n-drop/virtual_fs.c
    ../../source/daplink/error.c
    ../../source/daplink/validation.c
    ../../source/target/target_default.c)

set(IF_SRC ../../source/daplink/interface/daplink.c target_flash.c)
set(BL_SRC ../../source/daplink/bootloader/daplink.c)

add_executable(vfs_test_nrf52820_if ${SOURCES} ${IF_SRC} target_nrf52833.c)
target_compile_definitions(vfs_test_nrf52820_if PUBLIC -DINTERFACE_NRF52820 ${IF_DEFS})

add_executable(vfs_test_nrf52820_bl ${SOURCES} ${BL_SRC})
target_compile_definitions(vfs_test_nrf52820_bl PUBLIC -DINTERFACE_NRF52820 ${BL_DEFS})

add_executable(vfs_test_kl27z_if ${SOURCES} ${IF_SRC} target_nrf52833.c)
target_compile_definitions(vfs_test_kl27z_if PUBLIC -DINTERFACE_KL27Z ${IF_DEFS})

add_executable(vfs_test_kl27z_bl ${SOURCES} ${BL_SRC})
target_compile_definitions(vfs_test_kl27z_bl PUBLIC -DINTERFACE_KL27Z ${BL_DEFS})

add_executable(vfs_test_kl26z_if ${SOURCES} ${IF_SRC} target_nrf51822.c)
target_compile_definitions(vfs_test_kl26z_if PUBLIC -DINTERFACE_KL26Z ${IF_DEFS})
