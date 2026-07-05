cmake_minimum_required(VERSION 3.20)

project(ODriveFirmware C CXX ASM)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

set(BOARD_VERSION "v3.6-56V" CACHE STRING "ODrive board version")
set_property(CACHE BOARD_VERSION PROPERTY STRINGS
    v3.1 v3.2 v3.3
    v3.4-24V v3.4-48V
    v3.5-24V v3.5-48V
    v3.6-24V v3.6-56V
)

option(CONFIG_DEBUG "Build with debug optimization" OFF)
option(CONFIG_STRICT "Treat warnings as errors" OFF)
option(CONFIG_USE_LTO "Enable link-time optimization" OFF)
option(CONFIG_NO_DRM "Disable DRM support" OFF)
option(CONFIG_ENABLE_DISASM "Generate disassembly listing" OFF)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(AUTOGEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/autogen")
file(MAKE_DIRECTORY "${AUTOGEN_DIR}")

set(ROOT_INTERFACE ODrive3)
set(VERSION_C "${AUTOGEN_DIR}/version.c")
set(AUTOGEN_INTERFACES "${AUTOGEN_DIR}/interfaces.hpp")
set(AUTOGEN_FUNCTION_STUBS "${AUTOGEN_DIR}/function_stubs.hpp")
set(AUTOGEN_ENDPOINTS "${AUTOGEN_DIR}/endpoints.hpp")
set(AUTOGEN_TYPE_INFO "${AUTOGEN_DIR}/type_info.hpp")

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
            --template "${CMAKE_CURRENT_SOURCE_DIR}/fibre-cpp/interfaces_template.j2"
            --output "${AUTOGEN_INTERFACES}"
    DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/interface_generator_stub.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/odrive-interface.yaml"
        "${CMAKE_CURRENT_SOURCE_DIR}/fibre-cpp/interfaces_template.j2"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    VERBATIM
)

add_custom_command(
    OUTPUT "${AUTOGEN_FUNCTION_STUBS}"
    COMMAND "${Python3_EXECUTABLE}" -B
            "${CMAKE_CURRENT_SOURCE_DIR}/interface_generator_stub.py"
            --definitions "${CMAKE_CURRENT_SOURCE_DIR}/odrive-interface.yaml"
            --template "${CMAKE_CURRENT_SOURCE_DIR}/fibre-cpp/function_stubs_template.j2"
            --output "${AUTOGEN_FUNCTION_STUBS}"
    DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/interface_generator_stub.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/odrive-interface.yaml"
        "${CMAKE_CURRENT_SOURCE_DIR}/fibre-cpp/function_stubs_template.j2"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    VERBATIM
)

add_custom_command(
    OUTPUT "${AUTOGEN_ENDPOINTS}"
    COMMAND "${Python3_EXECUTABLE}" -B
            "${CMAKE_CURRENT_SOURCE_DIR}/interface_generator_stub.py"
            --definitions "${CMAKE_CURRENT_SOURCE_DIR}/odrive-interface.yaml"
            --generate-endpoints "${ROOT_INTERFACE}"
            --template "${CMAKE_CURRENT_SOURCE_DIR}/fibre-cpp/endpoints_template.j2"
            --output "${AUTOGEN_ENDPOINTS}"
    DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/interface_generator_stub.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/odrive-interface.yaml"
        "${CMAKE_CURRENT_SOURCE_DIR}/fibre-cpp/endpoints_template.j2"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    VERBATIM
)

add_custom_command(
    OUTPUT "${AUTOGEN_TYPE_INFO}"
    COMMAND "${Python3_EXECUTABLE}" -B
            "${CMAKE_CURRENT_SOURCE_DIR}/interface_generator_stub.py"
            --definitions "${CMAKE_CURRENT_SOURCE_DIR}/odrive-interface.yaml"
            --template "${CMAKE_CURRENT_SOURCE_DIR}/fibre-cpp/type_info_template.j2"
            --output "${AUTOGEN_TYPE_INFO}"
    DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/interface_generator_stub.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/odrive-interface.yaml"
        "${CMAKE_CURRENT_SOURCE_DIR}/fibre-cpp/type_info_template.j2"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    VERBATIM
)

set(FIRMWARE_SOURCES
    syscalls.c
    MotorControl/utils.cpp
    MotorControl/arm_sin_f32.c
    MotorControl/arm_cos_f32.c
    MotorControl/low_level.cpp
    MotorControl/axis.cpp
    MotorControl/motor.cpp
    MotorControl/thermistor.cpp
    MotorControl/encoder.cpp
    MotorControl/endstop.cpp
    MotorControl/acim_estimator.cpp
    MotorControl/mechanical_brake.cpp
    MotorControl/controller.cpp
    MotorControl/control_timeout.cpp
    MotorControl/foc.cpp
    MotorControl/open_loop_controller.cpp
    MotorControl/oscilloscope.cpp
    MotorControl/trapTraj.cpp
    MotorControl/pwm_input.cpp
    MotorControl/main.cpp
    Drivers/STM32/stm32_system.cpp
    Drivers/STM32/stm32_gpio.cpp
    Drivers/STM32/stm32_nvm.c
    Drivers/STM32/stm32_spi_arbiter.cpp
    Drivers/MT6826S/mt6826s_spi.cpp
    Drivers/MT6826S/vernier_resolver.cpp
    MotorControl/debug_counters.cpp
    communication/can/can_simple.cpp
    communication/can/odrive_can.cpp
    communication/communication.cpp
    communication/ascii_protocol.cpp
    communication/interface_uart.cpp
    communication/interface_usb.cpp
    communication/interface_i2c.cpp
    FreeRTOS-openocd.c
    "${VERSION_C}"
)

set(FIBRE_SOURCES
    fibre-cpp/fibre.cpp
    fibre-cpp/channel_discoverer.cpp
    fibre-cpp/legacy_protocol.cpp
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
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
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
    Board/v3/Src/usart.c
    Board/v3/Src/usbd_cdc_if.c
    Board/v3/Src/adc.c
    Board/v3/Src/stm32f4xx_hal_msp.c
    Board/v3/Src/usbd_desc.c
    Board/v3/Src/stm32f4xx_it.c
    Board/v3/Src/usb_device.c
    Board/v3/Src/can.c
    Board/v3/Src/system_stm32f4xx.c
    Board/v3/Src/gpio.c
    Board/v3/Src/i2c.c
)

if(BOARD_VERSION STREQUAL "v3.1")
    set(HW_VERSION_MINOR 1)
    set(HW_VERSION_VOLTAGE 24)
elseif(BOARD_VERSION STREQUAL "v3.2")
    set(HW_VERSION_MINOR 2)
    set(HW_VERSION_VOLTAGE 24)
elseif(BOARD_VERSION STREQUAL "v3.3")
    set(HW_VERSION_MINOR 3)
    set(HW_VERSION_VOLTAGE 24)
elseif(BOARD_VERSION STREQUAL "v3.4-24V")
    set(HW_VERSION_MINOR 4)
    set(HW_VERSION_VOLTAGE 24)
elseif(BOARD_VERSION STREQUAL "v3.4-48V")
    set(HW_VERSION_MINOR 4)
    set(HW_VERSION_VOLTAGE 48)
elseif(BOARD_VERSION STREQUAL "v3.5-24V")
    set(HW_VERSION_MINOR 5)
    set(HW_VERSION_VOLTAGE 24)
elseif(BOARD_VERSION STREQUAL "v3.5-48V")
    set(HW_VERSION_MINOR 5)
    set(HW_VERSION_VOLTAGE 48)
elseif(BOARD_VERSION STREQUAL "v3.6-24V")
    set(HW_VERSION_MINOR 6)
    set(HW_VERSION_VOLTAGE 24)
elseif(BOARD_VERSION STREQUAL "v3.6-56V")
    set(HW_VERSION_MINOR 6)
    set(HW_VERSION_VOLTAGE 56)
else()
    message(FATAL_ERROR "Unsupported BOARD_VERSION '${BOARD_VERSION}'. This CMake build currently supports v3 boards.")
endif()

add_executable(ODriveFirmware
    ${FIRMWARE_SOURCES}
    ${FIBRE_SOURCES}
    ${FREERTOS_SOURCES}
    ${USB_DEVICE_SOURCES}
    ${STM32_HAL_SOURCES}
    ${BOARD_V3_SOURCES}
    "${AUTOGEN_INTERFACES}"
    "${AUTOGEN_FUNCTION_STUBS}"
    "${AUTOGEN_ENDPOINTS}"
    "${AUTOGEN_TYPE_INFO}"
)

set_source_files_properties(
    "${AUTOGEN_INTERFACES}"
    "${AUTOGEN_FUNCTION_STUBS}"
    "${AUTOGEN_ENDPOINTS}"
    "${AUTOGEN_TYPE_INFO}"
    PROPERTIES GENERATED TRUE HEADER_FILE_ONLY TRUE
)

target_include_directories(ODriveFirmware PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/MotorControl"
    "${CMAKE_CURRENT_SOURCE_DIR}/fibre-cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/fibre-cpp/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/STM32F4xx_HAL_Driver/Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/FreeRTOS/Source/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/FreeRTOS/Source/CMSIS_RTOS"
    "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/CMSIS/Include"
    "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/CMSIS/Device/ST/STM32F7xx/Include"
    "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/CMSIS/Device/ST/STM32F4xx/Include"
    "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/STM32_USB_Device_Library/Core/Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/STM32_USB_Device_Library/Class/CDC/Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/Board/v3/Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F"
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/DRV8301"
)

target_compile_definitions(ODriveFirmware PRIVATE
    USE_HAL_DRIVER
    FIBRE_ENABLE_SERVER=1
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
    target_compile_options(ODriveFirmware PRIVATE -gdwarf-2 -Og)
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
    VERBATIM
)

if(CONFIG_ENABLE_DISASM)
    add_custom_command(TARGET ODriveFirmware POST_BUILD
        COMMAND "${CMAKE_OBJDUMP}" "$<TARGET_FILE:ODriveFirmware>" -dSC > "$<TARGET_FILE_DIR:ODriveFirmware>/ODriveFirmware.asm"
        VERBATIM
    )
endif()
