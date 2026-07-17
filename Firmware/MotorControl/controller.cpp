
#include "odrive_main.h"
#include "control_timeout.hpp"
#include <algorithm>
#include <cmath>

// Position deadband (hardcoded, not a config item). Zeros pos_err within this
// many turns so the position loop doesn't hunt on sub-count jitter at the
// target. Sized for a 6-axis arm with ~0.1mm end-effector accuracy:
//   - MT6826S 32768 cpr * 42:1 gear = 1,376,256 counts/output-rev
//     => 1 count ≈ 1.4μm at a 300mm link, ≈ 2.7μm at 600mm.
//   - 3 counts ≈ 4-8μm at the tip: ~5-8% of the 0.1mm budget (and ~15-20% of
//     the per-joint RSS budget 0.1mm/√6), while covering a few LSB of
//     magnetic-encoder noise. Calibrate up if jitter, down if accuracy sags.
constexpr float kPosDeadbandTurns = 3.0f / (32768.0f * 42.0f);  // ≈ 2.18e-6 turn

bool Controller::apply_config() {
    config_.parent = this;
    update_filter_gains();
    return true;
}

void Controller::reset() {
    // pos_setpoint is initialized in start_closed_loop_control
    vel_setpoint_ = 0.0f;
    pos_integrator_vel_ = 0.0f;
    vel_integrator_torque_ = 0.0f;
    torque_setpoint_ = 0.0f;
    held_motor_torque_ = 0.0f;
    torque_output_ = 0.0f;
    held_position_vel_des_ = 0.0f;
    held_position_gain_multiplier_ = 1.0f;
    held_position_error_ = 0.0f;
    position_step_valid_ = false;
    friction_torque_ = 0.0f;
    friction_dir_ = 0;
    reset_adrc();
    mechanical_power_ = 0.0f;
    electrical_power_ = 0.0f;
    overspeed_time_ = 0.0f;
    clear_overspeed_snapshot();
}

void Controller::set_error(Error error) {
    error_ |= error;
    last_error_time_ = odrv.n_evt_control_loop_ * current_meas_period;
}

void Controller::clear_overspeed_snapshot() {
    overspeed_snapshot_ = {};
}

void Controller::capture_overspeed_snapshot(float vel_estimate,
                                            const std::optional<float>& pos_estimate_linear,
                                            const std::optional<float>& pos_estimate_circular,
                                            const std::optional<float>& pos_wrap) {
    if (overspeed_snapshot_.valid) {
        return;
    }

    Encoder::VernierDiagnosticsSnapshot vernier = {};
    axis_->encoder_.get_vernier_diagnostics_snapshot(&vernier);

    overspeed_snapshot_.valid = true;
    overspeed_snapshot_.control_loop_count = odrv.n_evt_control_loop_;
    overspeed_snapshot_.timestamp = odrv.n_evt_control_loop_ * current_meas_period;
    overspeed_snapshot_.vel_estimate = vel_estimate;
    overspeed_snapshot_.vel_limit = config_.vel_limit;
    overspeed_snapshot_.vel_limit_tolerance = config_.vel_limit_tolerance;
    overspeed_snapshot_.pos_estimate_linear = pos_estimate_linear.value_or(0.0f);
    overspeed_snapshot_.pos_estimate_circular = pos_estimate_circular.value_or(0.0f);
    overspeed_snapshot_.pos_wrap = pos_wrap.value_or(0.0f);
    overspeed_snapshot_.pos_setpoint = pos_setpoint_;
    overspeed_snapshot_.vel_setpoint = vel_setpoint_;
    overspeed_snapshot_.torque_setpoint = torque_setpoint_;
    overspeed_snapshot_.input_pos = input_pos_;
    overspeed_snapshot_.input_vel = input_vel_;
    overspeed_snapshot_.input_torque = input_torque_;
    overspeed_snapshot_.input_mode = static_cast<uint32_t>(config_.input_mode);
    overspeed_snapshot_.control_mode = static_cast<uint32_t>(config_.control_mode);
    overspeed_snapshot_.resolver_state = vernier.state;
    overspeed_snapshot_.resolver_valid = vernier.resolver_valid ? 1u : 0u;
    overspeed_snapshot_.resolver_locked = vernier.resolver_locked ? 1u : 0u;
    overspeed_snapshot_.resolver_accepted_aux = vernier.resolver_accepted_aux ? 1u : 0u;
    overspeed_snapshot_.resolver_degraded = vernier.resolver_degraded ? 1u : 0u;
    overspeed_snapshot_.resolver_position_turns = vernier.position_turns;
    overspeed_snapshot_.resolver_residual = vernier.residual;
    overspeed_snapshot_.encoder_pos_estimate = vernier.encoder_pos_estimate;
    overspeed_snapshot_.encoder_vel_estimate = vernier.encoder_vel_estimate;
    overspeed_snapshot_.encoder_pos_circular = vernier.encoder_pos_circular;
    overspeed_snapshot_.pair_sequence = vernier.pair_count;
    overspeed_snapshot_.pair_valid = vernier.pair_valid ? 1u : 0u;
    overspeed_snapshot_.output_estimate_valid = vernier.output_estimate_valid ? 1u : 0u;
    overspeed_snapshot_.output_pos_estimate = vernier.output_pos_estimate;
    overspeed_snapshot_.output_vel_estimate = vernier.output_vel_estimate;
    overspeed_snapshot_.output_sample_dt = vernier.output_sample_dt;
    overspeed_snapshot_.output_pair_sequence = vernier.output_pair_sequence;
}

//--------------------------------
// Command Handling
//--------------------------------


void Controller::move_to_pos(float goal_point) {
    float profile_vel_limit = axis_->trap_traj_.config_.vel_limit;
    if (config_.enable_vel_limit &&
        std::isfinite(config_.vel_limit) &&
        config_.vel_limit > 0.0f) {
        profile_vel_limit = std::min(profile_vel_limit, config_.vel_limit);
    }
    axis_->trap_traj_.planTrapezoidal(goal_point, pos_setpoint_, vel_setpoint_,
                                 profile_vel_limit,
                                 axis_->trap_traj_.config_.accel_limit,
                                 axis_->trap_traj_.config_.decel_limit);
    axis_->trap_traj_.t_ = 0.0f;
    trajectory_done_ = false;
    ControlTimeout::mark_trajectory_active(*axis_);
}

void Controller::move_incremental(float displacement, bool from_input_pos = true){
    if(from_input_pos){
        input_pos_ += displacement;
    } else{
        input_pos_ = pos_setpoint_ + displacement;
    }

    input_pos_updated();
}

void Controller::set_input_pos_and_steps(float const pos) {
    input_pos_ = pos;
}

void Controller::set_mit_input(float pos_rad, float vel_rad_per_s, float kp, float kd, float torque_ff) {
    // Reject non-finite frames rather than poisoning the control loop.
    if (!std::isfinite(pos_rad) || !std::isfinite(vel_rad_per_s) ||
        !std::isfinite(kp) || !std::isfinite(kd) || !std::isfinite(torque_ff)) {
        return;
    }
    // kp / kd are gains and must not be negative.
    CRITICAL_SECTION() {
        mit_pos_rad_ = pos_rad;
        mit_vel_rad_per_s_ = vel_rad_per_s;
        mit_kp_ = std::max(kp, 0.0f);
        mit_kd_ = std::max(kd, 0.0f);
        mit_torque_ff_ = torque_ff;
    }
}

bool Controller::control_mode_updated() {
    position_step_valid_ = false;
    if (config_.control_mode >= CONTROL_MODE_POSITION_CONTROL) {
        InputPort<float>& estimate_src = config_.circular_setpoints ?
                                pos_estimate_circular_src_ :
                                pos_estimate_linear_src_;
        std::optional<float> estimate =
            axis_->encoder_.mode_ == Encoder::MODE_SPI_ABS_MT6826S_VERNIER ?
            estimate_src.present() :
            estimate_src.any();
        if (!estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }

        pos_setpoint_ = *estimate;
        set_input_pos_and_steps(*estimate);
        pos_integrator_vel_ = 0.0f;
    }
    return true;
}


void Controller::update_filter_gains() {
    float bandwidth = std::min(config_.input_filter_bandwidth, 0.25f * current_meas_hz);
    input_filter_ki_ = 2.0f * bandwidth;  // basic conversion to discrete time
    input_filter_kp_ = 0.25f * (input_filter_ki_ * input_filter_ki_); // Critically damped
}

void Controller::reset_adrc() {
    adrc_initialized_ = false;
    adrc_z1_ = 0.0f;
    adrc_z2_ = 0.0f;
    adrc_z3_ = 0.0f;
    adrc_last_torque_ = 0.0f;
    adrc_trim_torque_ = 0.0f;
}

float Controller::update_adrc_torque(float pos_estimate, float vel_estimate,
                                     float torque_cmd) {
    if (!adrc_initialized_) {
        adrc_z1_ = pos_estimate;
        adrc_z2_ = vel_estimate;
        adrc_z3_ = 0.0f;
        adrc_last_torque_ = 0.0f;
        adrc_trim_torque_ = 0.0f;
        adrc_initialized_ = true;
    }

    const float update_hz = 1.0f / std::max(update_period_, 1.0e-6f);
    const float wo = std::clamp(adrc_bandwidth_, 1.0f, 0.25f * update_hz);
    const float beta1 = 3.0f * wo;
    const float beta2 = 3.0f * wo * wo;
    const float beta3 = wo * wo * wo;
    const float b0 = std::max(adrc_b0_, 1.0e-6f);

    const float e = adrc_z1_ - pos_estimate;
    adrc_z1_ += update_period_ * (adrc_z2_ - beta1 * e);
    adrc_z2_ += update_period_ * (adrc_z3_ - beta2 * e + b0 * adrc_last_torque_);
    adrc_z3_ += update_period_ * (-beta3 * e);

    if (std::isfinite(adrc_disturbance_limit_)) {
        const float lim = std::abs(adrc_disturbance_limit_);
        adrc_z3_ = std::clamp(adrc_z3_, -lim, lim);
    }

    return torque_cmd - adrc_z3_ / b0;
}

float Controller::update_adrc_trim(float pos_estimate, float vel_estimate,
                                   float trim_limit) {
    float target = update_adrc_torque(pos_estimate, vel_estimate, 0.0f);
    if (std::isfinite(trim_limit)) {
        const float lim = std::abs(trim_limit);
        target = std::clamp(target, -lim, lim);
    }

    if (std::isfinite(config_.adrc_trim_slew_rate)) {
        const float step = std::abs(config_.adrc_trim_slew_rate) * update_period_;
        target = adrc_trim_torque_ + std::clamp(target - adrc_trim_torque_, -step, step);
    }

    adrc_trim_torque_ = target;
    return adrc_trim_torque_;
}

float Controller::update_friction_compensation(bool enabled,
                                               float pos_err,
                                               float vel_des,
                                               float vel_estimate,
                                               bool position_control_active) {
    // Direction selection. Friction opposes ACTUAL motion, so when the shaft
    // is moving the comp must follow sign(vel_estimate) — otherwise during
    // overshoot/reversal (intended direction flips before actual velocity
    // does) the Coulomb term lands in the same direction as friction and
    // amplifies it instead of canceling. Only when essentially stationary do
    // we fall back to the intended direction (vel_des, then pos_err) so the
    // comp can break static friction in the direction we want to go.
    int8_t dir = 0;
    const float abs_v = std::abs(vel_estimate);
    if (enabled) {
        if (abs_v > config_.friction_vel_deadband) {
            dir = vel_estimate > 0.0f ? 1 : -1;
        } else if (std::abs(vel_des) > config_.friction_vel_deadband) {
            dir = vel_des > 0.0f ? 1 : -1;
        } else if (position_control_active &&
                   std::abs(pos_err) > std::max(config_.friction_pos_deadband,
                                                4.0f * kPosDeadbandTurns)) {
            dir = pos_err > 0.0f ? 1 : -1;
        }
    }

    if (dir == 0) {
        // No motion, or comp disabled: slew back to zero. Don't hard-zero —
        // enabled can flip mid-motion because set_controller_mode (0x00B) is
        // not armed-guarded (unlike the enable flags 0x70/0x7C), and a sudden
        // friction_torque_ -> 0 step would jerk the motor. With slew_rate
        // = +inf (the default / disabled) this still collapses to zero in one
        // step, matching the old behavior.
        friction_dir_ = 0;
        float target = 0.0f;
        if (std::isfinite(config_.friction_torque_slew_rate)) {
            const float step = std::abs(config_.friction_torque_slew_rate) * update_period_;
            target = friction_torque_ + std::clamp(0.0f - friction_torque_, -step, step);
        }
        friction_torque_ = target;
        return friction_torque_;
    }

    friction_dir_ = dir;
    const bool positive = dir > 0;
    const float Ts = positive ? config_.friction_static_pos : config_.friction_static_neg;
    const float Tc = positive ? config_.friction_coulomb_pos : config_.friction_coulomb_neg;
    const float B  = positive ? config_.friction_viscous_pos : config_.friction_viscous_neg;

    // Gaussian Stribeck: Ts at v=0, decays to Tc above the Stribeck velocity.
    // Floor to Ts within the vel deadband so breakaway torque is available
    // when starting from rest.
    const float vs = std::max(config_.friction_stribeck_vel, 1.0e-6f);
    const float x = abs_v / vs;
    float mag = Tc + (Ts - Tc) * expf(-(x * x));
    if (abs_v <= config_.friction_vel_deadband) {
        mag = std::max(mag, Ts);
    }

    // Compensation acts in the direction of motion (counteracts friction
    // which opposes motion). Now that dir follows actual velocity when
    // moving, the Coulomb term (dir*mag) and the viscous term (B*vel_estimate)
    // share the same sign convention.
    float target = dir * mag + B * vel_estimate;

    if (std::isfinite(config_.friction_max_torque)) {
        const float lim = std::abs(config_.friction_max_torque);
        target = std::clamp(target, -lim, lim);
    }
    if (std::isfinite(config_.friction_torque_slew_rate)) {
        const float step = std::abs(config_.friction_torque_slew_rate) * update_period_;
        target = friction_torque_ + std::clamp(target - friction_torque_, -step, step);
    }

    friction_torque_ = target;
    return friction_torque_;
}

static float limitVel(const float vel_limit, const float vel_estimate, const float vel_gain, const float torque) {
    float Tmax = (vel_limit - vel_estimate) * vel_gain;
    float Tmin = (-vel_limit - vel_estimate) * vel_gain;
    return std::clamp(torque, Tmin, Tmax);
}

bool Controller::update(float update_period, bool run_position_step,
                        float position_step_period) {
    update_period_ = std::max(update_period, 1.0e-6f);
    position_step_period_ = position_step_period > 0.0f
        ? position_step_period
        : update_period_;
    ControlTimeout::mark_running(*axis_);

    std::optional<float> pos_estimate_linear = pos_estimate_linear_src_.present();
    std::optional<float> pos_estimate_circular = pos_estimate_circular_src_.present();
    std::optional<float> pos_wrap = pos_wrap_src_.present();
    std::optional<float> vel_estimate = vel_estimate_src_.present();

    float pos_err_for_friction = 0.0f;
    bool position_control_active_for_friction = false;

    // TODO also enable circular deltas for 2nd order filter, etc.
    if (config_.circular_setpoints) {
        if (!pos_wrap.has_value()) {
            set_error(ERROR_INVALID_CIRCULAR_RANGE);
            return false;
        }
        input_pos_ = fmodf_pos(input_pos_, *pos_wrap);
    }

    // Update inputs
    switch (config_.input_mode) {
        case INPUT_MODE_INACTIVE: {
            // do nothing
        } break;
        case INPUT_MODE_PASSTHROUGH: {
            pos_setpoint_ = input_pos_;
            vel_setpoint_ = input_vel_;
            torque_setpoint_ = input_torque_; 
        } break;
        case INPUT_MODE_VEL_RAMP: {
            float target_vel = ControlTimeout::quick_stop_active(*axis_) ? 0.0f : input_vel_;
            float rate = ControlTimeout::quick_stop_active(*axis_)
                ? config_.quick_stop_decel_limit
                : (std::abs(target_vel) < std::abs(vel_setpoint_)
                       ? config_.velocity_decel_limit
                       : config_.velocity_accel_limit);
            float max_step_size = std::abs(update_period_ * rate);
            float full_step = target_vel - vel_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            vel_setpoint_ += step;
            torque_setpoint_ = (step / update_period_) * config_.inertia;
            if (ControlTimeout::quick_stop_active(*axis_) && std::abs(vel_setpoint_) < 1e-3f) {
                vel_setpoint_ = 0.0f;
                ControlTimeout::mark_holding(*axis_);
            }
        } break;
        case INPUT_MODE_TORQUE_RAMP: {
            float max_step_size = std::abs(update_period_ * config_.torque_ramp_rate);
            float full_step = input_torque_ - torque_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            torque_setpoint_ += step;
        } break;
        case INPUT_MODE_POS_FILTER: {
            if (config_.control_mode >= CONTROL_MODE_POSITION_CONTROL &&
                !run_position_step && position_step_valid_) {
                break;
            }
            // 2nd order pos tracking filter
            float delta_pos = input_pos_ - pos_setpoint_; // Pos error
            if (config_.circular_setpoints) {
                if (!pos_wrap.has_value()) {
                    set_error(ERROR_INVALID_CIRCULAR_RANGE);
                    return false;
                }
                delta_pos = wrap_pm(delta_pos, *pos_wrap);
            }
            float delta_vel = input_vel_ - vel_setpoint_; // Vel error
            float accel = input_filter_kp_*delta_pos + input_filter_ki_*delta_vel; // Feedback
            torque_setpoint_ = accel * config_.inertia; // Accel
            const float filter_dt = config_.control_mode >= CONTROL_MODE_POSITION_CONTROL
                ? position_step_period_ : update_period_;
            vel_setpoint_ += filter_dt * accel; // delta vel
            pos_setpoint_ += filter_dt * vel_setpoint_; // Delta pos
        } break;
        case INPUT_MODE_TRAP_TRAJ: {
            if (!run_position_step && position_step_valid_) {
                break;
            }
            if(input_pos_updated_){
                move_to_pos(input_pos_);
                input_pos_updated_ = false;
            }
            // Avoid updating uninitialized trajectory
            if (trajectory_done_)
                break;
            
            if (axis_->trap_traj_.t_ > axis_->trap_traj_.Tf_) {
                // Drop into position control mode when done to avoid problems on loop counter delta overflow
                config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
                pos_setpoint_ = axis_->trap_traj_.Xf_;
                vel_setpoint_ = 0.0f;
                torque_setpoint_ = 0.0f;
                trajectory_done_ = true;
                ControlTimeout::mark_trajectory_done(*axis_);
            } else {
                TrapezoidalTrajectory::Step_t traj_step = axis_->trap_traj_.eval(axis_->trap_traj_.t_);
                pos_setpoint_ = traj_step.Y;
                vel_setpoint_ = traj_step.Yd;
                torque_setpoint_ = traj_step.Ydd * config_.inertia;
                axis_->trap_traj_.t_ += position_step_period_;
            }
        } break;
        case INPUT_MODE_TUNING: {
            autotuning_phase_ = wrap_pm_pi(autotuning_phase_ + (2.0f * M_PI * autotuning_.frequency * update_period_));
            float c = our_arm_cos_f32(autotuning_phase_);
            float s = our_arm_sin_f32(autotuning_phase_);
            pos_setpoint_ = input_pos_ + autotuning_.pos_amplitude * s; // + pos_amp_c * c
            vel_setpoint_ = input_vel_ + autotuning_.vel_amplitude * c;
            torque_setpoint_ = input_torque_ + autotuning_.torque_amplitude * -s;
        } break;
        case INPUT_MODE_MIT: {
            // MIT-style packed control: compute torque directly from the
            // per-frame kp/kd/t_ff against the current encoder estimates.
            // Optional actuator-side friction compensation is added later, but
            // the velocity PI / position loop must not re-process the MIT
            // torque.
            if (config_.control_mode != CONTROL_MODE_TORQUE_CONTROL) {
                set_error(ERROR_INVALID_INPUT_MODE);
                return false;
            }
            if (!pos_estimate_linear.has_value() || !vel_estimate.has_value()) {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }

            float pos_estimate_rad = *pos_estimate_linear * 2.0f * M_PI;
            float vel_estimate_rad = *vel_estimate * 2.0f * M_PI;

            float pos_err = mit_pos_rad_ - pos_estimate_rad;
            if (std::abs(pos_err) < kPosDeadbandTurns * 2.0f * M_PI) {
                pos_err = 0.0f;  // position deadband: suppress sub-count jitter
            }
            float vel_err = mit_vel_rad_per_s_ - vel_estimate_rad;

            torque_setpoint_ =
                mit_kp_ * pos_err +
                mit_kd_ * vel_err +
                mit_torque_ff_;

            // Mirror the command into the setpoints (in turns) for telemetry /
            // downstream code that inspects them. They are not re-processed
            // when control_mode == CONTROL_MODE_TORQUE_CONTROL.
            pos_setpoint_ = mit_pos_rad_ / (2.0f * M_PI);
            vel_setpoint_ = mit_vel_rad_per_s_ / (2.0f * M_PI);
            pos_err_for_friction = pos_err / (2.0f * M_PI);
            position_control_active_for_friction = true;
        } break;
        default: {
            set_error(ERROR_INVALID_INPUT_MODE);
            return false;
        }
        
    }

    // Never command a setpoint beyond its limit
    if(config_.enable_vel_limit) {
        vel_setpoint_ = std::clamp(vel_setpoint_, -config_.vel_limit, config_.vel_limit);
    }
    const float controller_to_motor_torque = axis_->encoder_.controller_torque_to_motor_torque_scale();
    const float Tlim = axis_->motor_.max_available_torque() / controller_to_motor_torque;
    torque_setpoint_ = std::clamp(torque_setpoint_, -Tlim, Tlim);

    const bool mit_adrc_active = config_.enable_adrc &&
        config_.control_mode == CONTROL_MODE_TORQUE_CONTROL &&
        config_.input_mode == INPUT_MODE_MIT;

    // Position control
    // TODO Decide if we want to use encoder or pll position here
    float gain_scheduling_multiplier = 1.0f;
    float vel_des = vel_setpoint_;
    bool adrc_active = config_.enable_adrc && config_.control_mode >= CONTROL_MODE_VELOCITY_CONTROL;
    float adrc_pos_estimate = 0.0f;
    float adrc_vel_estimate = 0.0f;
    bool adrc_measurement_valid = false;
    if (config_.control_mode >= CONTROL_MODE_POSITION_CONTROL) {
        float pos_err;

        if (config_.circular_setpoints) {
            if (!pos_estimate_circular.has_value() || !pos_wrap.has_value()) {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            // Keep pos setpoint from drifting
            pos_setpoint_ = fmodf_pos(pos_setpoint_, *pos_wrap);
            // Circular delta
            pos_err = pos_setpoint_ - *pos_estimate_circular;
            pos_err = wrap_pm(pos_err, *pos_wrap);
            adrc_active = false;
        } else {
            if (!pos_estimate_linear.has_value()) {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            pos_err = pos_setpoint_ - *pos_estimate_linear;
            if (!vel_estimate.has_value()) {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            adrc_pos_estimate = *pos_estimate_linear;
            adrc_vel_estimate = *vel_estimate;
            adrc_measurement_valid = true;
        }

        // Position deadband: zeros pos_err within kPosDeadbandTurns so the
        // position P+I and gain scheduling see zero error at the target.
        if (std::abs(pos_err) < kPosDeadbandTurns) {
            pos_err = 0.0f;
        }
        if (run_position_step || !position_step_valid_) {
            if (config_.pos_integrator_gain > 0.0f) {
                pos_integrator_vel_ +=
                    config_.pos_integrator_gain * position_step_period_ * pos_err;
                const float pos_integrator_limit = std::abs(config_.vel_limit);
                if (std::isfinite(pos_integrator_limit)) {
                    pos_integrator_vel_ = std::clamp(
                        pos_integrator_vel_, -pos_integrator_limit, pos_integrator_limit);
                }
            } else {
                pos_integrator_vel_ = 0.0f;
            }

            held_position_vel_des_ =
                vel_setpoint_ + config_.pos_gain * pos_err + pos_integrator_vel_;
            held_position_gain_multiplier_ = 1.0f;
            const float abs_pos_err = std::abs(pos_err);
            if (config_.enable_gain_scheduling &&
                abs_pos_err <= config_.gain_scheduling_width) {
                held_position_gain_multiplier_ =
                    abs_pos_err / config_.gain_scheduling_width;
            }
            held_position_error_ = pos_err;
            position_step_valid_ = true;
        }

        vel_des = held_position_vel_des_;
        gain_scheduling_multiplier = held_position_gain_multiplier_;
        pos_err_for_friction = held_position_error_;
        position_control_active_for_friction = true;
    } else {
        pos_integrator_vel_ = 0.0f;
        position_step_valid_ = false;
    }

    if (adrc_active && !adrc_measurement_valid) {
        if (!pos_estimate_linear.has_value() || !vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        adrc_pos_estimate = *pos_estimate_linear;
        adrc_vel_estimate = *vel_estimate;
        adrc_measurement_valid = true;
    }

    if (mit_adrc_active && !adrc_measurement_valid) {
        if (!pos_estimate_linear.has_value() || !vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        adrc_pos_estimate = *pos_estimate_linear;
        adrc_vel_estimate = *vel_estimate;
        adrc_measurement_valid = true;
    }

    if (!adrc_active && !mit_adrc_active) {
        reset_adrc();
    }

    // Velocity limiting
    float vel_lim = config_.vel_limit;
    if (config_.enable_vel_limit) {
        vel_des = std::clamp(vel_des, -vel_lim, vel_lim);
    }

    // Check for overspeed fault (done in this module (controller) for cohesion with vel_lim)
    if (config_.enable_overspeed_error) {  // 0.0f to disable
        if (!vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        if (std::abs(*vel_estimate) > config_.vel_limit_tolerance * vel_lim) {
            overspeed_time_ += update_period_;
        } else {
            overspeed_time_ = 0.0f;
        }

        // Require a short, continuous violation before latching OVERSPEED. This
        // filters one-sample estimator/controller handoff transients without
        // changing the configured velocity limit.
        if (overspeed_time_ >= 0.002f) {
            capture_overspeed_snapshot(*vel_estimate, pos_estimate_linear,
                                       pos_estimate_circular, pos_wrap);
            set_error(ERROR_OVERSPEED);
            return false;
        }
    }

    float vel_gain = config_.vel_gain;
    float vel_integrator_gain = config_.vel_integrator_gain;

    // Velocity control
    float torque = torque_setpoint_;

    float v_err = 0.0f;
    if (config_.control_mode >= CONTROL_MODE_VELOCITY_CONTROL) {
        if (!vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }

        v_err = vel_des - *vel_estimate;
        torque += (vel_gain * gain_scheduling_multiplier) * v_err;

        // Velocity integral action before limiting
        torque += vel_integrator_torque_;
    }

    const float adrc_trim_limit = std::max(config_.adrc_trim_torque_limit, 0.0f);
    if (mit_adrc_active) {
        if (!adrc_measurement_valid) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        torque += update_adrc_trim(adrc_pos_estimate, adrc_vel_estimate,
                                   adrc_trim_limit);
        vel_integrator_torque_ = 0.0f;
    } else if (adrc_active) {
        if (!adrc_measurement_valid) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        torque += update_adrc_trim(adrc_pos_estimate, adrc_vel_estimate,
                                   adrc_trim_limit);
    }

    const bool friction_enabled =
        (config_.control_mode >= CONTROL_MODE_VELOCITY_CONTROL &&
         config_.enable_friction_compensation) ||
        (config_.control_mode == CONTROL_MODE_TORQUE_CONTROL &&
         config_.input_mode == INPUT_MODE_MIT &&
         config_.enable_mit_friction_compensation);

    // Friction compensation (output-shaft Nm); added before velocity/Tlim clamps.
    if (vel_estimate.has_value()) {
        torque += update_friction_compensation(
            friction_enabled, pos_err_for_friction, vel_des, *vel_estimate,
            position_control_active_for_friction);
    } else {
        friction_torque_ = 0.0f;
        friction_dir_ = 0;
    }

    // Velocity limiting in current mode
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL &&
            config_.enable_vel_limit &&
            config_.enable_torque_mode_vel_limit &&
            std::isfinite(config_.vel_limit) &&
            config_.vel_limit > 0.0f) {
        if (!vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        torque = limitVel(config_.vel_limit, *vel_estimate, vel_gain, torque);
    }

    // Torque limiting
    bool limited = false;
    if (torque > Tlim) {
        limited = true;
        torque = Tlim;
    }
    if (torque < -Tlim) {
        limited = true;
        torque = -Tlim;
    }
    if (adrc_active || mit_adrc_active) {
        adrc_last_torque_ = torque;
    }

    // Velocity integrator (behaviour dependent on limiting)
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL || mit_adrc_active) {
        // reset integral if not in use
        vel_integrator_torque_ = 0.0f;
    } else {
        if (limited) {
            // TODO make decayfactor configurable
            vel_integrator_torque_ *= 0.99f;
        } else {
            vel_integrator_torque_ += ((vel_integrator_gain * gain_scheduling_multiplier) * update_period_) * v_err;
        }
        // integrator limiting to prevent windup 
        vel_integrator_torque_ = std::clamp(vel_integrator_torque_, -config_.vel_integrator_limit, config_.vel_integrator_limit);
    }

    float ideal_electrical_power = axis_->motor_.current_control_.power_ - \
        SQ(axis_->motor_.current_control_.Iq_measured_) * 1.5f * axis_->motor_.config_.phase_resistance - \
        SQ(axis_->motor_.current_control_.Id_measured_) * 1.5f * axis_->motor_.config_.phase_resistance;
    mechanical_power_ += config_.mechanical_power_bandwidth * update_period_ * (torque * *vel_estimate * M_PI * 2.0f - mechanical_power_);
    electrical_power_ += config_.electrical_power_bandwidth * update_period_ * (ideal_electrical_power - electrical_power_);

    // Spinout check
    // If mechanical power is negative (braking) and measured power is positive, something is wrong
    // This indicates that the controller is trying to stop, but torque is being produced.
    // Usually caused by an incorrect encoder offset
    const bool spinout_check_valid =
        axis_->encoder_.mode_ != Encoder::MODE_SPI_ABS_MT6826S_VERNIER;
    if (spinout_check_valid &&
        mechanical_power_ < config_.spinout_mechanical_power_threshold &&
        electrical_power_ > config_.spinout_electrical_power_threshold) {
        set_error(ERROR_SPINOUT_DETECTED);
        return false;
    }

    held_motor_torque_ = torque * controller_to_motor_torque;
    torque_output_ = held_motor_torque_;

    // TODO: this is inconsistent with the other errors which are sticky.
    // However if we make ERROR_INVALID_ESTIMATE sticky then it will be
    // confusing that a normal sequence of motor calibration + encoder
    // calibration would leave the controller in an error state.
    error_ &= ~ERROR_INVALID_ESTIMATE;
    return true;
}

void Controller::publish_held_torque() {
    torque_output_ = held_motor_torque_;
}
