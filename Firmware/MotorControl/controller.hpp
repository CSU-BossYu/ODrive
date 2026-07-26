#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

#include <optional>

class Controller : public ODriveIntf::ControllerIntf {
public:
    struct Autotuning_t {
        float frequency = 0.0f;
        float pos_amplitude = 0.0f;
        float vel_amplitude = 0.0f;
        float torque_amplitude = 0.0f;
    };

    struct Config_t {
        ControlMode control_mode = CONTROL_MODE_POSITION_CONTROL;  //see: ControlMode_t
        InputMode input_mode = INPUT_MODE_PASSTHROUGH;             //see: InputMode_t
        float pos_gain = 7.0f;                   // [(turn/s) / turn], SguanFOC position Kp
        float pos_integrator_gain = 0.0f;        // [(turn/s) / (turn * s)]
        float vel_gain = 10.0f;                  // [Nm/(turn/s)], output-shaft PI
        float vel_integrator_gain = 1.5f;        // [Nm/(turn/s * s)]
        float vel_limit = 2.0f;                  // [turn/s] Infinity to disable.
        float vel_limit_tolerance = 1.2f;        // ratio to vel_lim. Infinity to disable.
        float vel_integrator_limit = INFINITY;   // Vel. integrator clamping value. Infinity to disable.
        float vel_ramp_rate = 1.0f;              // [(turn/s) / s]
        float torque_ramp_rate = 0.01f;          // Nm / sec
        // Command/heartbeat watchdog control (persisted). Consumed by
        // ControlTimeout and the VEL_RAMP input mode in update().
        float velocity_accel_limit = 0.5f;       // [turn/s²] VEL_RAMP acceleration
        float velocity_decel_limit = 0.5f;       // [turn/s²] VEL_RAMP deceleration
        float quick_stop_decel_limit = 1.0f;     // [turn/s²] quick-stop ramp on timeout
        TimeoutAction timeout_action = TIMEOUT_ACTION_QUICK_STOP_AND_HOLD; // action on watchdog expiry
        bool circular_setpoints = false;
        float circular_setpoint_range = 1.0f;    // Circular range when circular_setpoints is true. [turn]
        uint32_t steps_per_circular_range = 1024;
        float inertia = 0.0f;                    // [Nm/(turn/s^2)]
        float input_filter_bandwidth = 2.0f;     // [1/s]
        float gain_scheduling_width = 10.0f;
        bool enable_gain_scheduling = false;
        bool enable_vel_limit = true;
        bool enable_overspeed_error = true;
        bool enable_torque_mode_vel_limit = true;  // enable velocity limit in current control mode (requires a valid velocity estimator)
        float mechanical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for mechanical power for spinout detction
        float electrical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for electrical power for spinout detection
        float spinout_electrical_power_threshold = 10.0f; // [W] electrical power threshold for spinout detection
        float spinout_mechanical_power_threshold = -10.0f; // [W] mechanical power threshold for spinout detection

        // SguanFOC v3.0.1 super-twisting speed controller at 2 kHz.
        // Opt-in until its current-domain gains are converted for this plant.
        bool enable_sta = false;

        // Legacy friction-identification results retained as calibration data.
        // The Sguan-aligned realtime controller does not inject this model.
        bool enable_friction_compensation = false;      // legacy calibration data only
        bool enable_mit_friction_compensation = false;  // legacy calibration data only
        float friction_pos_deadband = 0.0f;        // [turn], output shaft
        float friction_vel_deadband = 0.001f;      // [turn/s], output shaft
        float friction_stribeck_vel = 0.01f;       // [turn/s], output shaft
        float friction_static_pos = 0.0f;          // [Nm], output shaft
        float friction_static_neg = 0.0f;          // [Nm], output shaft
        float friction_coulomb_pos = 0.0f;         // [Nm], output shaft
        float friction_coulomb_neg = 0.0f;         // [Nm], output shaft
        float friction_viscous_pos = 0.0f;         // [Nm/(turn/s)]
        float friction_viscous_neg = 0.0f;         // [Nm/(turn/s)]
        float friction_max_torque = INFINITY;      // [Nm], output shaft. Infinity = disabled.
        float friction_torque_slew_rate = INFINITY;// [Nm/s], output shaft. Infinity = disabled.

        // custom setters
        Controller* parent;
        void set_input_filter_bandwidth(float value) { input_filter_bandwidth = value; parent->update_filter_gains(); }
        void set_steps_per_circular_range(uint32_t value) { steps_per_circular_range = value > 0 ? value : steps_per_circular_range; }
        void set_circular_setpoints(bool) { circular_setpoints = false; }
        void set_circular_setpoint_range(float) { circular_setpoint_range = 1.0f; }
        void set_control_mode(ControlMode value) { control_mode = value; parent->control_mode_updated(); }
    };

    
    bool apply_config();

    void reset();
    void set_error(Error error);

    void input_pos_updated() {
        input_pos_updated_ = true;
        pos_integrator_vel_ = 0.0f;
        reset_sta();
    }
    bool control_mode_updated();
    void set_input_pos_and_steps(float pos);

    // Accept a decoded MIT-style packed control frame. Only stores the values
    // (with finite/clamp sanity checks); the torque is computed later in
    // update() under INPUT_MODE_MIT. Safe to call from the CAN thread.
    void set_mit_input(float pos_rad, float vel_rad_per_s, float kp, float kd, float torque_ff);

    // Trajectory-Planned control
    void move_to_pos(float goal_point);
    void move_incremental(float displacement, bool from_goal_point);

    void update_filter_gains();
    void reset_sta();
    float update_sta_velocity(float vel_estimate, float vel_des,
                              float torque_feedforward);
    bool update(float update_period = current_meas_period,
                bool run_position_step = true,
                float position_step_period = 0.0f);
    void publish_held_torque();

    Config_t config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    struct OverspeedSnapshot {
        bool valid = false;
        uint32_t control_loop_count = 0;
        float timestamp = 0.0f;
        float vel_estimate = 0.0f;
        float vel_limit = 0.0f;
        float vel_limit_tolerance = 0.0f;
        float pos_estimate_linear = 0.0f;
        float pos_estimate_circular = 0.0f;
        float pos_wrap = 0.0f;
        float pos_setpoint = 0.0f;
        float vel_setpoint = 0.0f;
        float torque_setpoint = 0.0f;
        float input_pos = 0.0f;
        float input_vel = 0.0f;
        float input_torque = 0.0f;
        uint32_t input_mode = 0;
        uint32_t control_mode = 0;
        uint32_t resolver_state = 0;
        uint32_t resolver_valid = 0;
        uint32_t resolver_locked = 0;
        uint32_t resolver_accepted_aux = 0;
        uint32_t resolver_degraded = 0;
        float resolver_position_turns = 0.0f;
        float resolver_residual = 0.0f;
        float encoder_pos_estimate = 0.0f;
        float encoder_vel_estimate = 0.0f;
        float encoder_pos_circular = 0.0f;
        uint32_t pair_sequence = 0;
        uint32_t pair_valid = 0;
        uint32_t output_estimate_valid = 0;
        float output_pos_estimate = 0.0f;
        float output_vel_estimate = 0.0f;
        float output_sample_dt = 0.0f;
        uint32_t output_pair_sequence = 0;
    };

    void clear_overspeed_snapshot();
    void capture_overspeed_snapshot(float vel_estimate,
                                    const std::optional<float>& pos_estimate_linear,
                                    const std::optional<float>& pos_estimate_circular,
                                    const std::optional<float>& pos_wrap);
    const OverspeedSnapshot& get_overspeed_snapshot() const { return overspeed_snapshot_; }

    Error error_ = ERROR_NONE;
    float last_error_time_ = 0.0f;

    // Inputs
    InputPort<float> pos_estimate_linear_src_;
    InputPort<float> pos_estimate_circular_src_;
    InputPort<float> vel_estimate_src_;
    InputPort<float> pos_wrap_src_; 

    float pos_setpoint_ = 0.0f; // [turns]
    float vel_setpoint_ = 0.0f; // [turn/s]
    float pos_integrator_vel_ = 0.0f;       // [turn/s]
    float vel_integrator_torque_ = 0.0f;    // [Nm]
    float torque_setpoint_ = 0.0f;  // [Nm], output-shaft Nm in vernier mode
    float held_motor_torque_ = 0.0f; // [Nm], republished on divided outer-loop ticks
    float update_period_ = current_meas_period;
    float position_step_period_ = current_meas_period;
    float held_position_vel_des_ = 0.0f;
    float held_position_gain_multiplier_ = 1.0f;
    float held_position_error_ = 0.0f;
    bool position_step_valid_ = false;
    float sta_integral_current_ = 0.0f; // Sguan STA integral term [Aq]
    bool sta_integral_frozen_ = false;

    float input_pos_ = 0.0f;     // [turns]
    float input_vel_ = 0.0f;     // [turn/s]
    float input_torque_ = 0.0f;  // [Nm], output-shaft Nm in vernier mode
    float input_filter_kp_ = 0.0f;
    float input_filter_ki_ = 0.0f;

    // MIT-style packed control input (set from CAN layer, consumed in update()).
    // Position/velocity are in [rad] / [rad/s]; kp in [Nm/rad], kd in [Nm/(rad/s)],
    // torque feed-forward in [Nm]. These are only used when input_mode == INPUT_MODE_MIT.
    float mit_pos_rad_ = 0.0f;
    float mit_vel_rad_per_s_ = 0.0f;
    float mit_kp_ = 0.0f;        // [Nm / rad]
    float mit_kd_ = 0.0f;        // [Nm / (rad/s)]
    float mit_torque_ff_ = 0.0f; // [Nm]

    Autotuning_t autotuning_;
    float autotuning_phase_ = 0.0f;
    
    bool input_pos_updated_ = false;
    
    bool trajectory_done_ = true;

    float mechanical_power_ = 0.0f; // [W]
    float electrical_power_ = 0.0f; // [W]
    float overspeed_time_ = 0.0f; // [s]

    // Outputs
    OutputPort<float> torque_output_ = 0.0f;

    // custom setters
    void set_input_pos(float value) { set_input_pos_and_steps(value); input_pos_updated(); }

private:
    OverspeedSnapshot overspeed_snapshot_;
};

#endif // __CONTROLLER_HPP
