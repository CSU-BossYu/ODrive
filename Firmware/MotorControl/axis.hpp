#ifndef __AXIS_HPP
#define __AXIS_HPP

#include <functional>

class Axis;

#include "encoder.hpp"
#include "controller.hpp"
#include "open_loop_controller.hpp"
#include "trapTraj.hpp"
#include "low_level.h"
#include "utils.hpp"
#include "task_timer.hpp"
#include "calibration_session.hpp"
#include "calibration_buffer.hpp"
#include "calibration_result.hpp"
#include "calibration_geometry_fitter.hpp"
#include "calibration_flux_fitter.hpp"
#include "calibration_mechanical_fitter.hpp"
#include "calibration_delay_fitter.hpp"
#include <production_config.h>

#include <array>

class Axis : public ODriveIntf::AxisIntf {
public:
    struct LockinConfig_t {
        float current = ODRIVE_PRODUCTION_AXIS_CALIBRATION_CURRENT;           // [A]
        float ramp_time = 0.4f;          // [s]
        float ramp_distance = 1 * M_PI;  // [rad]
        float accel = 20.0f;     // [rad/s^2]
        float vel = 40.0f; // [rad/s]
        float finish_distance = 100.0f;  // [rad]
        bool finish_on_vel = false;
        bool finish_on_distance = false;
    };

    struct TaskTimes {
        TaskTimer thermistor_update;
        TaskTimer encoder_update;
        TaskTimer can_heartbeat;
        TaskTimer controller_update;
        TaskTimer open_loop_controller_update;
        TaskTimer motor_update;
        TaskTimer current_controller_update;
        TaskTimer dc_calib;
        TaskTimer current_sense;
        TaskTimer pwm_update;
    };

    static LockinConfig_t default_calibration();
    static LockinConfig_t default_lockin();

    struct CANConfig_t {
        uint32_t node_id = ODRIVE_PRODUCTION_CAN_NODE_ID;
        bool is_extended = false;
        uint32_t heartbeat_rate_ms = 100;
        // Command/heartbeat watchdog timeouts (persisted). 0 = disabled.
        // can_watchdog_timeout_ms: fed by motion command frames, triggers
        // controller.config.timeout_action on expiry.
        // heartbeat_timeout_ms: fed by master heartbeat/NMT frames.
        uint32_t can_watchdog_timeout_ms = 300;
        uint32_t heartbeat_timeout_ms = 0;
        uint32_t encoder_rate_ms = 10;
        uint32_t motor_error_rate_ms = 0;
        uint32_t encoder_error_rate_ms = 0;
        uint32_t controller_error_rate_ms = 0;
        uint32_t encoder_count_rate_ms = 0;
        uint32_t iq_rate_ms = 0;
        uint32_t bus_vi_rate_ms = 0;
    };

    struct Config_t {
        bool startup_motor_calibration = false;   //<! run motor calibration at startup, skip otherwise
        bool startup_encoder_offset_calibration = false; //<! run encoder offset calibration after startup, skip otherwise
        bool startup_closed_loop_control = false; //<! enable closed loop control after calibration/startup
        float watchdog_timeout = 0.0f; // [s]
        bool enable_watchdog = false;

        LockinConfig_t calibration_lockin = default_calibration();
        LockinConfig_t general_lockin;

        CANConfig_t can;

        Axis* parent = nullptr;
    };

    struct CAN_t {
        uint32_t last_heartbeat = 0;
        uint32_t last_encoder = 0;
        uint32_t last_motor_error = 0;
        uint32_t last_encoder_error = 0;
        uint32_t last_controller_error = 0;
        uint32_t last_encoder_count = 0;
        uint32_t last_iq = 0;
        uint32_t last_bus_vi = 0;
    };

    Axis(int axis_num,
            osPriority thread_priority,
            Encoder& encoder,
            Controller& controller,
            Motor& motor,
            TrapezoidalTrajectory& trap);

    bool apply_config();
    void clear_config();

    void start_thread();
    bool wait_for_control_iteration();

    bool do_checks(uint32_t timestamp);

    void watchdog_feed();
    bool watchdog_check();

    // True if there are no errors
    bool inline check_for_errors() {
        return error_ == ERROR_NONE;
    }

    bool start_closed_loop_control();
    bool stop_closed_loop_control();
    bool run_lockin_spin(const LockinConfig_t &lockin_config, bool remain_armed,
                std::function<bool(bool)> loop_cb = {} );
    bool run_closed_loop_control_loop();
    bool run_idle_loop();

    constexpr uint32_t get_watchdog_reset() {
        return static_cast<uint32_t>(std::clamp<float>(config_.watchdog_timeout, 0, UINT32_MAX / (current_meas_hz + 1)) * current_meas_hz);
    }

    void run_state_machine_loop();
    bool start_calibration_session(uint32_t request_options);
    bool abort_calibration_session();
    void capture_calibration_electrical_sample(
        uint32_t output_timestamp, const float (&pwm_timings)[3],
        bool pwm_valid);
    bool capture_calibration_full_sample(CalibrationSampleV1* captured = nullptr);

    // hardware config
    int axis_num_;
    osPriority thread_priority_;
    Config_t config_;

    Encoder& encoder_;
    Controller& controller_;
    OpenLoopController open_loop_controller_;
    Motor& motor_;
    TrapezoidalTrajectory& trap_traj_;
    TaskTimes task_times_;
    CalibrationSession calibration_session_;
    // About 3.2 KiB per axis. Raw samples are drained by the calibration
    // transport; long-term storage remains host-side.
    CalibrationRecordBuffer<32> calibration_record_buffer_;
    CalibrationPendingResult calibration_pending_result_;
    CalibrationGeometryFitter calibration_geometry_fitter_;
    CalibrationFluxFitter calibration_flux_fitter_;
    CalibrationMechanicalFitter calibration_mechanical_fitter_;
    CalibrationDelayFitter calibration_delay_fitter_;
    volatile bool calibration_start_pending_ = false;
    volatile bool calibration_capture_enabled_ = false;
    uint32_t calibration_sample_sequence_ = 0;

    osThreadId thread_id_ = 0;
    const uint32_t stack_size_ = 2048; // Bytes
    volatile bool thread_id_valid_ = false;

    // variables exposed on protocol
    Error error_ = ERROR_NONE;
    uint32_t last_drv_fault_ = 0;

    AxisState requested_state_ = AXIS_STATE_STARTUP_SEQUENCE;
    std::array<AxisState, 10> task_chain_ = { AXIS_STATE_UNDEFINED };
    AxisState& current_state_ = task_chain_.front();
    CAN_t can_;


    // watchdog
    uint32_t watchdog_current_value_= 0;

private:
    void service_calibration_session_start();
    bool run_calibration_geometry_scan();
    bool run_calibration_flux_scan();
    bool run_calibration_encoder_alignment();
    bool finalize_vernier_offset_candidate();
    bool run_calibration_mechanical_scan();
    bool run_calibration_delay_scan();
    bool validate_calibration_candidate() const;
    bool commit_calibration_candidate();
};


#endif /* __AXIS_HPP */
