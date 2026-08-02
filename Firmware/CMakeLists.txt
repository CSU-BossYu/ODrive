cmake_minimum_required(VERSION 3.20)

option(BUILD_FIRMWARE "Build the STM32 firmware target" ON)
if(BUILD_FIRMWARE)
    project(ODriveFirmware C CXX ASM)
else()
    project(ODriveFirmware CXX)
endif()

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

set(BOARD_VERSION "v3.6-56V" CACHE STRING "ODrive board version")
set_property(CACHE BOARD_VERSION PROPERTY STRINGS
    v3.6-24V v3.6-56V
)

option(CONFIG_DEBUG "Build with debug optimization" OFF)
option(CONFIG_STRICT "Treat warnings as errors" OFF)
option(CONFIG_USE_LTO "Enable link-time optimization" OFF)
option(CONFIG_NO_DRM "Disable DRM support" OFF)
option(CONFIG_ENABLE_DISASM "Generate disassembly listing" OFF)
option(BUILD_NATIVE_TESTS "Build and register native C++ tests" OFF)
option(HOST_SANITIZERS "Enable ASan/UBSan for native tests when supported" ON)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(AUTOGEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/autogen")
file(MAKE_DIRECTORY "${AUTOGEN_DIR}")

set(ROOT_INTERFACE ODrive3)
set(VERSION_C "${AUTOGEN_DIR}/version.c")
set(AUTOGEN_INTERFACES "${AUTOGEN_DIR}/interfaces.hpp")

add_custom_command(
    OUTPUT "${VERSION_C}"
    COMMAND "${Python3_EXECUTABLE}" -B
            "${CMAKE_CURRENT_SOURCE_DIR}/../tools/odrive/version.py"
            --output "${VERSION_C}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/../tools/odrive/version.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    VERBATIM
)

add_custom_command(
    OUTPUT "${AUTOGEN_INTERFACES}"
    COMMAND "${Python3_EXECUTABLE}" -B
            "${CMAKE_CURRENT_SOURCE_DIR}/interface_generator_stub.py"
            --definitions "${CMAKE_CURRENT_SOURCE_DIR}/odrive-interface.yaml"
            --template "${CMAKE_CURRENT_SOURCE_DIR}/interface/interfaces_template.j2"
            --output "${AUTOGEN_INTERFACES}"
    DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/interface_generator_stub.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/odrive-interface.yaml"
        "${CMAKE_CURRENT_SOURCE_DIR}/interface/interfaces_template.j2"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    VERBATIM
)

set(FIRMWARE_SOURCES
    syscalls.c
    MotorControl/fault_manager.cpp
    MotorControl/platform_stm32_ports.cpp
    MotorControl/crash_recorder.cpp
    MotorControl/realtime_event_ring.cpp
    MotorControl/realtime_snapshot.cpp
    MotorControl/scope_capture.cpp
    MotorControl/safety_supervisor.cpp
    MotorControl/trace_rings.cpp
    USB/usb_debug_protocol.cpp
    USB/usb_debug_transport.cpp
    MotorControl/utils.cpp
    MotorControl/arm_sin_f32.c
    MotorControl/arm_cos_f32.c
    MotorControl/low_level.cpp
    MotorControl/axis.cpp
    MotorControl/motor.cpp
    MotorControl/thermistor.cpp
    MotorControl/encoder.cpp
    MotorControl/controller.cpp
    MotorControl/control_timeout.cpp
    MotorControl/foc.cpp
    MotorControl/open_loop_controller.cpp
    MotorControl/trapTraj.cpp
    MotorControl/main.cpp
    Drivers/STM32/stm32_system.cpp
    Drivers/STM32/stm32_gpio.cpp
    Drivers/STM32/stm32_nvm.c
    Drivers/STM32/stm32_spi_arbiter.cpp
    Drivers/MT6826S/mt6826s_spi.cpp
    Drivers/MT6826S/vernier_resolver.cpp
    Drivers/MT6826S/lz5710_position_estimator.cpp
    MotorControl/debug_counters.cpp
    communication/can/can_simple.cpp
    communication/can/odrive_can.cpp
    communication/communication.cpp
    communication/interface_usb.cpp
    communication/calibration_transport.cpp
    FreeRTOS-openocd.c
    "${VERSION_C}"
)

set(FREERTOS_SOURCES
    ThirdParty/FreeRTOS/Source/croutine.c
    ThirdParty/FreeRTOS/Source/event_groups.c
    ThirdParty/FreeRTOS/Source/list.c
    ThirdParty/FreeRTOS/Source/queue.c
    ThirdParty/FreeRTOS/Source/stream_buffer.c
    ThirdParty/FreeRTOS/Source/tasks.c
    ThirdParty/FreeRTOS/Source/timers.c
    ThirdParty/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    ThirdParty/FreeRTOS/Source/portable/MemMang/heap_4.c
    ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)

set(USB_DEVICE_SOURCES
    ThirdParty/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ThirdParty/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ThirdParty/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ThirdParty/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
)

set(STM32_HAL_SOURCES
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
)

set(BOARD_V3_SOURCES
    Board/v3/startup_stm32f405xx.s
    Drivers/DRV8301/drv8301.cpp
    Board/v3/board.cpp
    Board/v3/Src/stm32f4xx_hal_timebase_TIM.c
    Board/v3/Src/tim.c
    Board/v3/Src/dma.c
    Board/v3/Src/freertos.c
    Board/v3/Src/main.c
    Board/v3/Src/usbd_conf.c
    Board/v3/Src/spi.c
    Board/v3/Src/usbd_cdc_if.c
    Board/v3/Src/adc.c
    Board/v3/Src/stm32f4xx_hal_msp.c
    Board/v3/Src/usbd_desc.c
    Board/v3/Src/stm32f4xx_it.c
    Board/v3/Src/usb_device.c
    Board/v3/Src/can.c
    Board/v3/Src/system_stm32f4xx.c
    Board/v3/Src/gpio.c
)

if(BOARD_VERSION STREQUAL "v3.6-24V")
    set(HW_VERSION_MINOR 6)
    set(HW_VERSION_VOLTAGE 24)
elseif(BOARD_VERSION STREQUAL "v3.6-56V")
    set(HW_VERSION_MINOR 6)
    set(HW_VERSION_VOLTAGE 56)
else()
    message(FATAL_ERROR "Unsupported BOARD_VERSION '${BOARD_VERSION}'. Production firmware supports ODrive v3.6 only.")
endif()

set(PROTOCOL_VERSION "0x0000010C" CACHE STRING "Product CAN protocol version" FORCE)
set(USB_DEBUG_PROTOCOL_VERSION "4.5" CACHE STRING "USB Debug/Test protocol version" FORCE)
set(MANIFEST_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/../tools/generate_build_manifest.py")
set(BUILD_MANIFEST "${CMAKE_CURRENT_BINARY_DIR}/build_manifest.json")
set(FAULT_SCHEMA "${CMAKE_CURRENT_SOURCE_DIR}/../docs/fault_schema.json")
set(FAULT_SCHEMA_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/../tools/generate_fault_schema.py")
set(FAULT_CODES_GENERATED "${CMAKE_CURRENT_BINARY_DIR}/autogen/fault_codes_generated.hpp")
set(FAULT_CODES_PYTHON "${CMAKE_CURRENT_SOURCE_DIR}/../foc_ui/backend/odrive_can/fault_codes.py")
set(FAULT_CODES_TYPESCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/../foc_ui/frontend/src/fault_codes.ts")
set(LEGACY_ERROR_GATE "${CMAKE_CURRENT_SOURCE_DIR}/../tools/check_legacy_error_writes.py")
set(PHASE2_STATIC_GATE "${CMAKE_CURRENT_SOURCE_DIR}/../tools/check_phase2_static_gate.py")
set(PHASE3_STATIC_GATE "${CMAKE_CURRENT_SOURCE_DIR}/../tools/check_phase3_static_gate.py")
set(PHASE4_STATIC_GATE "${CMAKE_CURRENT_SOURCE_DIR}/../tools/check_phase4_static_gate.py")
set(PHASE5_STATIC_GATE "${CMAKE_CURRENT_SOURCE_DIR}/../tools/check_phase5_static_gate.py")
set(PHASE6_STATIC_GATE "${CMAKE_CURRENT_SOURCE_DIR}/../tools/check_phase6_static_gate.py")
set(PHASE7_STATIC_GATE "${CMAKE_CURRENT_SOURCE_DIR}/../tools/check_phase7_static_gate.py")
set(PHASE8_STATIC_GATE "${CMAKE_CURRENT_SOURCE_DIR}/../tools/check_phase8_static_gate.py")
set(USB_PROTOCOL_SCHEMA "${CMAKE_CURRENT_SOURCE_DIR}/../docs/usb_debug_protocol_schema.json")
set(SCOPE_SCHEMA "${CMAKE_CURRENT_SOURCE_DIR}/../docs/scope_channel_schema.json")
set(SCOPE_SCHEMA_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/../tools/generate_scope_schema.py")
set(SCOPE_CPP_GENERATED "${CMAKE_CURRENT_SOURCE_DIR}/MotorControl/scope_channels_generated.hpp")
set(SCOPE_PYTHON_GENERATED "${CMAKE_CURRENT_SOURCE_DIR}/../foc_ui/backend/odrive_usb/scope_channels_generated.py")
set(SCOPE_TYPESCRIPT_GENERATED "${CMAKE_CURRENT_SOURCE_DIR}/../foc_ui/frontend/src/scope_channels_generated.ts")
set(CAN_PRODUCT_SCHEMA "${CMAKE_CURRENT_SOURCE_DIR}/../docs/can_product_protocol_schema.json")
set(CAN_PRODUCT_SCHEMA_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/../tools/generate_can_product_schema.py")
set(CAN_PRODUCT_CPP_GENERATED "${CMAKE_CURRENT_SOURCE_DIR}/communication/can/can_product_schema_generated.hpp")
set(CAN_PRODUCT_PYTHON_GENERATED "${CMAKE_CURRENT_SOURCE_DIR}/../foc_ui/backend/odrive_can/can_product_schema_generated.py")
set(CAN_PRODUCT_TEST_PYTHON_GENERATED "${CMAKE_CURRENT_SOURCE_DIR}/Tests/odrive_hil/can_product_schema_generated.py")
set(CAN_PRODUCT_TYPESCRIPT_GENERATED "${CMAKE_CURRENT_SOURCE_DIR}/../foc_ui/frontend/src/can_product_schema_generated.ts")
set(USB_IDENTITY_HEADER "${CMAKE_CURRENT_BINARY_DIR}/autogen/usb_debug_identity.hpp")
if(CONFIG_DEBUG)
    set(MANIFEST_OPTIMIZATION "-Og -g3")
    set(MANIFEST_CONFIGURATION "firmware-debug")
else()
    set(MANIFEST_OPTIMIZATION "-O2")
    set(MANIFEST_CONFIGURATION "firmware-release")
endif()
if(NOT BUILD_FIRMWARE)
    set(MANIFEST_CONFIGURATION "host-debug")
endif()

add_custom_target(build_manifest ALL
    COMMAND "${Python3_EXECUTABLE}" -B "${MANIFEST_SCRIPT}"
            --output "${BUILD_MANIFEST}"
            --configuration "${MANIFEST_CONFIGURATION}"
            --board "${BOARD_VERSION}"
            --protocol-version "${PROTOCOL_VERSION}"
            --usb-protocol-version "${USB_DEBUG_PROTOCOL_VERSION}"
            --compiler-id "${CMAKE_CXX_COMPILER_ID}"
            --compiler-version "${CMAKE_CXX_COMPILER_VERSION}"
            --compiler-path "${CMAKE_CXX_COMPILER}"
            --fault-schema "${FAULT_SCHEMA}"
            --protocol-schema "${USB_PROTOCOL_SCHEMA}"
            --scope-schema "${SCOPE_SCHEMA}"
            --can-schema "${CAN_PRODUCT_SCHEMA}"
            --identity-header "${USB_IDENTITY_HEADER}"
            --build-firmware "${BUILD_FIRMWARE}"
            --build-native-tests "${BUILD_NATIVE_TESTS}"
            "--compile-option=${MANIFEST_OPTIMIZATION}"
    BYPRODUCTS "${BUILD_MANIFEST}"
    VERBATIM
)

add_custom_command(
    OUTPUT "${FAULT_CODES_GENERATED}" "${FAULT_CODES_PYTHON}" "${FAULT_CODES_TYPESCRIPT}"
    COMMAND "${Python3_EXECUTABLE}" -B "${FAULT_SCHEMA_SCRIPT}"
            --schema "${FAULT_SCHEMA}"
            --cpp-output "${FAULT_CODES_GENERATED}"
            --python-output "foc_ui/backend/odrive_can/fault_codes.py"
            --typescript-output "foc_ui/frontend/src/fault_codes.ts"
    DEPENDS "${FAULT_SCHEMA_SCRIPT}" "${FAULT_SCHEMA}"
    VERBATIM
)
add_custom_target(generate_fault_schema DEPENDS
    "${FAULT_CODES_GENERATED}" "${FAULT_CODES_PYTHON}" "${FAULT_CODES_TYPESCRIPT}")
add_custom_target(fault_write_gate ALL
    COMMAND "${Python3_EXECUTABLE}" -B "${LEGACY_ERROR_GATE}"
    DEPENDS "${LEGACY_ERROR_GATE}"
    VERBATIM
)
add_custom_target(phase2_static_gate ALL
    COMMAND "${Python3_EXECUTABLE}" -B "${PHASE2_STATIC_GATE}"
    DEPENDS "${PHASE2_STATIC_GATE}"
    VERBATIM
)
add_custom_target(phase3_static_gate ALL
    COMMAND "${Python3_EXECUTABLE}" -B "${PHASE3_STATIC_GATE}"
    DEPENDS "${PHASE3_STATIC_GATE}"
    VERBATIM
)
add_custom_target(phase4_static_gate ALL
    COMMAND "${Python3_EXECUTABLE}" -B "${PHASE4_STATIC_GATE}"
    DEPENDS "${PHASE4_STATIC_GATE}" "${USB_PROTOCOL_SCHEMA}"
    VERBATIM
)
add_custom_command(
    OUTPUT "${SCOPE_CPP_GENERATED}" "${SCOPE_PYTHON_GENERATED}" "${SCOPE_TYPESCRIPT_GENERATED}"
    COMMAND "${Python3_EXECUTABLE}" -B "${SCOPE_SCHEMA_SCRIPT}"
            --schema "${SCOPE_SCHEMA}"
            --cpp-output "${SCOPE_CPP_GENERATED}"
            --python-output "${SCOPE_PYTHON_GENERATED}"
            --typescript-output "${SCOPE_TYPESCRIPT_GENERATED}"
    DEPENDS "${SCOPE_SCHEMA_SCRIPT}" "${SCOPE_SCHEMA}"
    VERBATIM
)
add_custom_target(generate_scope_schema
    DEPENDS "${SCOPE_CPP_GENERATED}" "${SCOPE_PYTHON_GENERATED}" "${SCOPE_TYPESCRIPT_GENERATED}")
add_custom_target(phase5_static_gate ALL
    COMMAND "${Python3_EXECUTABLE}" -B "${PHASE5_STATIC_GATE}"
    DEPENDS "${PHASE5_STATIC_GATE}" "${SCOPE_SCHEMA}" "${USB_PROTOCOL_SCHEMA}"
    VERBATIM
)
add_custom_target(phase6_static_gate ALL
    COMMAND "${Python3_EXECUTABLE}" -B "${PHASE6_STATIC_GATE}"
    DEPENDS "${PHASE6_STATIC_GATE}" "${USB_PROTOCOL_SCHEMA}"
    VERBATIM
)
add_custom_command(
    OUTPUT "${CAN_PRODUCT_CPP_GENERATED}" "${CAN_PRODUCT_PYTHON_GENERATED}"
           "${CAN_PRODUCT_TEST_PYTHON_GENERATED}"
           "${CAN_PRODUCT_TYPESCRIPT_GENERATED}"
    COMMAND "${Python3_EXECUTABLE}" -B "${CAN_PRODUCT_SCHEMA_SCRIPT}"
            --schema "${CAN_PRODUCT_SCHEMA}"
            --cpp-output "${CAN_PRODUCT_CPP_GENERATED}"
            --python-output "${CAN_PRODUCT_PYTHON_GENERATED}"
            --test-python-output "${CAN_PRODUCT_TEST_PYTHON_GENERATED}"
            --typescript-output "${CAN_PRODUCT_TYPESCRIPT_GENERATED}"
    DEPENDS "${CAN_PRODUCT_SCHEMA_SCRIPT}" "${CAN_PRODUCT_SCHEMA}"
    VERBATIM
)
add_custom_target(generate_can_product_schema
    DEPENDS "${CAN_PRODUCT_CPP_GENERATED}" "${CAN_PRODUCT_PYTHON_GENERATED}"
            "${CAN_PRODUCT_TYPESCRIPT_GENERATED}"
            "${CAN_PRODUCT_TEST_PYTHON_GENERATED}")
add_custom_target(phase7_static_gate ALL
    COMMAND "${Python3_EXECUTABLE}" -B "${PHASE7_STATIC_GATE}"
    DEPENDS "${PHASE7_STATIC_GATE}" "${CAN_PRODUCT_SCHEMA}"
    VERBATIM
)
add_dependencies(phase7_static_gate generate_can_product_schema)
add_custom_target(phase8_static_gate ALL
    COMMAND "${Python3_EXECUTABLE}" -B "${PHASE8_STATIC_GATE}"
    DEPENDS "${PHASE8_STATIC_GATE}" "${CAN_PRODUCT_TEST_PYTHON_GENERATED}"
    VERBATIM
)
add_dependencies(phase8_static_gate generate_can_product_schema)

if(NOT BUILD_FIRMWARE)
    if(BUILD_NATIVE_TESTS)
        enable_testing()
        add_subdirectory(Tests)
    endif()
    return()
endif()

add_executable(ODriveFirmware
    ${FIRMWARE_SOURCES}
    ${FREERTOS_SOURCES}
    ${USB_DEVICE_SOURCES}
    ${STM32_HAL_SOURCES}
    ${BOARD_V3_SOURCES}
    "${AUTOGEN_INTERFACES}"
)
add_dependencies(ODriveFirmware build_manifest generate_fault_schema
                 generate_scope_schema phase2_static_gate phase3_static_gate
                 generate_can_product_schema phase4_static_gate phase5_static_gate
                 phase6_static_gate phase7_static_gate phase8_static_gate)

set_source_files_properties(
    "${AUTOGEN_INTERFACES}"
    PROPERTIES GENERATED TRUE HEADER_FILE_ONLY TRUE
)

target_include_directories(ODriveFirmware PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/autogen"
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/MotorControl"
    "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/STM32F4xx_HAL_Driver/Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/FreeRTOS/Source/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/FreeRTOS/Source/CMSIS_RTOS"
    "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/CMSIS/Include"
    "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/CMSIS/Device/ST/STM32F4xx/Include"
    "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/STM32_USB_Device_Library/Core/Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/STM32_USB_Device_Library/Class/CDC/Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/Board/v3/Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F"
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/DRV8301"
)

target_compile_definitions(ODriveFirmware PRIVATE
    USE_HAL_DRIVER
    FIBRE_ENABLE_SERVER=0
    FIBRE_ENABLE_CLIENT=0
    FIBRE_ENABLE_EVENT_LOOP=0
    FIBRE_ALLOW_HEAP=0
    FIBRE_MAX_LOG_VERBOSITY=0
    FIBRE_DEFAULT_LOG_VERBOSITY=2
    FIBRE_ENABLE_LIBUSB_BACKEND=0
    FIBRE_ENABLE_TCP_SERVER_BACKEND=0
    FIBRE_ENABLE_TCP_CLIENT_BACKEND=0
    ODRIVE_PRODUCTION_SINGLE_AXIS=1
    ODRIVE_PRODUCTION_CAN_BAUD=1000000UL
    ODRIVE_PRODUCTION_CAN_NODE_ID=0UL
    ARM_MATH_CM4
    FPU_FPV4
    STM32F405xx
    HW_VERSION_MAJOR=3
    HW_VERSION_MINOR=${HW_VERSION_MINOR}
    HW_VERSION_VOLTAGE=${HW_VERSION_VOLTAGE}
)

if(CONFIG_NO_DRM)
    target_compile_definitions(ODriveFirmware PRIVATE NO_DRM)
endif()

set(COMMON_CPU_FLAGS
    -mthumb
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

target_compile_options(ODriveFirmware PRIVATE
    ${COMMON_CPU_FLAGS}
    "-D__weak=__attribute__((weak))"
    "-D__packed=__attribute__((__packed__))"
    -Wno-psabi
    -Wall
    -Wdouble-promotion
    -Wfloat-conversion
    -fdata-sections
    -ffunction-sections
    -g
    -Wno-nonnull
)

target_compile_options(ODriveFirmware PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-register>
)

if(CONFIG_DEBUG)
    target_compile_options(ODriveFirmware PRIVATE -g3 -Og)
else()
    target_compile_options(ODriveFirmware PRIVATE -O2)
endif()

if(CONFIG_STRICT)
    target_compile_options(ODriveFirmware PRIVATE -Werror)
endif()

if(CONFIG_USE_LTO)
    target_compile_options(ODriveFirmware PRIVATE -flto)
    target_link_options(ODriveFirmware PRIVATE -flto)
endif()

target_link_directories(ODriveFirmware PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/CMSIS/Lib/GCC"
)

target_link_options(ODriveFirmware PRIVATE
    ${COMMON_CPU_FLAGS}
    -specs=nosys.specs
    -specs=nano.specs
    -u,_printf_float
    -u,_scanf_float
    -Wl,--cref
    -Wl,--gc-sections
    -Wl,--undefined=uxTopUsedPriority
    -Wl,-Map=$<TARGET_FILE_DIR:ODriveFirmware>/ODriveFirmware.map
    -T${CMAKE_CURRENT_SOURCE_DIR}/Board/v3/STM32F405RGTx_FLASH.ld
)

target_link_libraries(ODriveFirmware PRIVATE
    c
    m
    nosys
    arm_cortexM4lf_math
)

add_custom_command(TARGET ODriveFirmware POST_BUILD
    COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:ODriveFirmware>"
    COMMAND "${CMAKE_OBJCOPY}" -O ihex "$<TARGET_FILE:ODriveFirmware>" "$<TARGET_FILE_DIR:ODriveFirmware>/ODriveFirmware.hex"
    COMMAND "${CMAKE_OBJCOPY}" -O binary -S "$<TARGET_FILE:ODriveFirmware>" "$<TARGET_FILE_DIR:ODriveFirmware>/ODriveFirmware.bin"
    COMMAND "${Python3_EXECUTABLE}" -B
            "${CMAKE_CURRENT_SOURCE_DIR}/../tools/archive_firmware_artifacts.py"
            --manifest "${BUILD_MANIFEST}"
            --elf "$<TARGET_FILE:ODriveFirmware>"
            --map "$<TARGET_FILE_DIR:ODriveFirmware>/ODriveFirmware.map"
            --archive-root "${CMAKE_CURRENT_BINARY_DIR}/artifacts"
            --schema "${FAULT_SCHEMA}"
            --schema "${USB_PROTOCOL_SCHEMA}"
            --schema "${SCOPE_SCHEMA}"
            --schema "${CAN_PRODUCT_SCHEMA}"
    VERBATIM
)

if(CONFIG_ENABLE_DISASM)
    add_custom_command(TARGET ODriveFirmware POST_BUILD
        COMMAND "${CMAKE_OBJDUMP}" "$<TARGET_FILE:ODriveFirmware>" -dSC > "$<TARGET_FILE_DIR:ODriveFirmware>/ODriveFirmware.asm"
        VERBATIM
    )
endif()

if(BUILD_NATIVE_TESTS)
    enable_testing()
    add_subdirectory(Tests)
endif()
