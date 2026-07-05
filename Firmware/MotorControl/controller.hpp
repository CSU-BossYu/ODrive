#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

#include <optional>

class Controller : public ODriveIntf::ControllerIntf {
public:
    struct Anticogging_t {
        uint32_t index = 0;
        float cogging_map[3600];
        bool pre_calibrated = false;
        bool calib_anticogging = false;
        float calib_pos_threshold = 1.0f;
        float calib_vel_threshold = 1.0f;
        float cogging_ratio = 1.0f;
        bool anticogging_enabled = true;
    };

    struct Autotuning_t {
        float frequency = 0.0f;
        float pos_amplitude = 0.0f;
        float vel_amplitude = 0.0f;
        float torque_amplitude = 0.0f;
    };

    struct Config_t {
        ControlMode control_mode = CONTROL_MODE_POSITION_CONTROL;  //see: ControlMode_t
        InputMode input_mode = INPUT_MODE_PASSTHROUGH;             //see: InputMode_t
        float pos_gain = 20.0f;                  // [(turn/s) / turn]
        float vel_gain = 1.0f / 6.0f;            // [Nm/(turn/s)]
        float vel_integrator_gain = 2.0f / 6.0f; // [Nm/(turn/s * s)]
        float vel_limit = 2.0f;                  // [turn/s] Infinity to disable.
        float vel_limit_tolerance = 1.2f;        // ratio to vel_lim. Infinity to disable.
        float vel_integrator_limit = INFINITY;   // Vel. integrator clamping value. Infinity to disable.
        float vel_ramp_rate = 1.0f;              // [(turn/s) / s]
        float torque_ramp_rate = 0.01f;          // Nm / sec
        bool circular_setpoints = false;
        float circular_setpoint_range = 1.0f;    // Circular range when circular_setpoints is true. [turn]
        uint32_t steps_per_circular_range = 1024;
        float inertia = 0.0f;                    // [Nm/(turn/s^2)]
        float input_filter_bandwidth = 2.0f;     // [1/s]
        float homing_speed = 0.25f;              // [turn/s]
        Anticogging_t anticogging;
        float gain_scheduling_width = 10.0f;
        bool enable_gain_scheduling = false;
        bool enable_vel_limit = true;
        bool enable_overspeed_error = true;
        bool enable_torque_mode_vel_limit = true;  // enable velocity limit in current control mode (requires a valid velocity estimator)
        float mechanical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for mechanical power for spinout detction
        float electrical_power_bandwidth = 20.0f; // [rad/s] filter cutoff for electrical power for spinout detection
        float spinout_electrical_power_threshold = 10.0f; // [W] electrical power threshold for spinout detection
        float spinout_mechanical_power_threshold = -10.0f; // [W] mechanical power threshold for spinout detection

        // custom setters
        Controller* parent;
        void set_input_filter_bandwidth(float value) { input_filter_bandwidth = value; parent->update_filter_gains(); }
        void set_steps_per_circular_range(uint32_t value) { steps_per_circular_range = value > 0 ? value : steps_per_circular_range; }
        void set_control_mode(ControlMode value) { control_mode = value; parent->control_mode_updated(); }
    };

    
    bool apply_config();

    void reset();
    void set_error(Error error);

    void input_pos_updated() {
        input_pos_updated_ = true;
        pos_integrator_vel_ = 0.0f;
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
    
    // TODO: make this more similar to other calibration loops
    void start_anticogging_calibration();
    float remove_anticogging_bias();
    bool anticogging_calibration(float pos_estimate, float vel_estimate);
    
    float get_anticogging_value(uint32_t index) {
        return (index < 3600) ? config_.anticogging.cogging_map[index] : 0.0f;
    }

    void update_filter_gains();
    bool update();

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
    float pos_integrator_gain_ = 0.0f;      // [(turn/s) / (turn * s)]
    float pos_integrator_vel_ = 0.0f;       // [turn/s]
    float vel_integrator_torque_ = 0.0f;    // [Nm]
    float torque_setpoint_ = 0.0f;  // [Nm], output-shaft Nm in vernier mode

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

    bool anticogging_valid_ = false;
    bool anticogging_calibration_initialized_ = false;
    uint32_t anticogging_start_index_ = 0;
    float anticogging_start_pos_ = 0.0f;
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
