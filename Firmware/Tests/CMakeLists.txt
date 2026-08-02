set(NATIVE_TEST_SOURCES
    ../MotorControl/fault_manager.cpp
    ../MotorControl/native_scenario_c_api.cpp
    ../MotorControl/realtime_event_ring.cpp
    ../MotorControl/realtime_snapshot.cpp
    ../MotorControl/scope_capture.cpp
    ../MotorControl/safety_supervisor.cpp
    ../MotorControl/trace_rings.cpp
    ../USB/usb_debug_protocol.cpp
    ../USB/usb_debug_transport.cpp
    test_runner.cpp
    native_crash_recorder_stub.cpp
    test_can.cpp
    test_rotate.cpp
    test_timer.cpp
    test_trap_traj.cpp
    test_fault_manager.cpp
    test_safety_supervisor.cpp
    test_realtime_snapshot.cpp
    test_trace_rings.cpp
    test_usb_debug_protocol.cpp
    test_scope_capture.cpp
    test_usb_scope_protocol.cpp
    test_can_product_protocol.cpp
    test_native_scenario_c_api.cpp
    test_platform_ports.cpp
)

add_executable(odrive_native_tests ${NATIVE_TEST_SOURCES})
target_include_directories(odrive_native_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/.."
    "${CMAKE_CURRENT_SOURCE_DIR}/../doctest"
    "${CMAKE_BINARY_DIR}"
    "${CMAKE_BINARY_DIR}/autogen"
)
add_dependencies(odrive_native_tests generate_fault_schema)
add_dependencies(odrive_native_tests phase2_static_gate)
add_dependencies(odrive_native_tests phase3_static_gate)
add_dependencies(odrive_native_tests build_manifest)
add_dependencies(odrive_native_tests phase4_static_gate)
add_dependencies(odrive_native_tests generate_scope_schema)
add_dependencies(odrive_native_tests phase5_static_gate)
add_dependencies(odrive_native_tests generate_can_product_schema phase7_static_gate)
add_dependencies(odrive_native_tests phase8_static_gate)
target_compile_features(odrive_native_tests PRIVATE cxx_std_17)
if(MSVC)
    target_compile_options(odrive_native_tests PRIVATE /W4 /permissive-)
else()
    target_compile_options(odrive_native_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(HOST_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang" AND NOT MINGW)
    target_compile_options(odrive_native_tests PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
    target_link_options(odrive_native_tests PRIVATE -fsanitize=address,undefined)
elseif(HOST_SANITIZERS AND MINGW)
    message(WARNING "Host MinGW toolchain has no ASan/UBSan runtime; native tests remain enabled without sanitizers")
endif()

add_test(NAME odrive_native_tests COMMAND odrive_native_tests)
add_test(NAME odrive_native_python
    COMMAND "${CMAKE_COMMAND}" -E env
            "ODRIVE_SCENARIO_LIBRARY=$<TARGET_FILE:odrive_scenario>"
            "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/native_scenario_smoke.py")
set_tests_properties(odrive_native_tests PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)
set_tests_properties(odrive_native_python PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
if(WIN32)
    get_filename_component(ODRIVE_HOST_RUNTIME_DIR
        "${CMAKE_CXX_COMPILER}" DIRECTORY)
    set_tests_properties(odrive_native_tests odrive_native_python PROPERTIES
        ENVIRONMENT_MODIFICATION
        "PATH=path_list_prepend:${ODRIVE_HOST_RUNTIME_DIR}"
    )
endif()

add_library(odrive_scenario SHARED
    ../MotorControl/fault_manager.cpp
    ../MotorControl/native_scenario_c_api.cpp
    ../MotorControl/realtime_event_ring.cpp
    ../MotorControl/safety_supervisor.cpp
)
target_include_directories(odrive_scenario PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/.."
    "${CMAKE_BINARY_DIR}"
    "${CMAKE_BINARY_DIR}/autogen"
)
target_compile_definitions(odrive_scenario PRIVATE ODRIVE_SCENARIO_BUILD=1)
target_compile_features(odrive_scenario PRIVATE cxx_std_17)
add_dependencies(odrive_scenario generate_fault_schema phase8_static_gate)
set_target_properties(odrive_scenario PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
)
if(MSVC)
    target_compile_options(odrive_scenario PRIVATE /W4 /permissive-)
else()
    target_compile_options(odrive_scenario PRIVATE -Wall -Wextra -Wpedantic)
endif()
# The executable above compiles the same core sources with ASan/UBSan. Keep the
# ctypes library unsanitized so a stock Python process can load it without
# platform-specific LD_PRELOAD ordering requirements.
