
#include "odrive_main.h"
#include "control_timeout.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

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

void Controller::start_anticogging_calibration() {
    // Ensure the cogging map was correctly allocated earlier and that the motor is capable of calibrating
    if (axis_->error_ == Axis::ERROR_NONE) {
        config_.anticogging.index = 0;
        anticogging_calibration_initialized_ = false;
        anticogging_valid_ = false;
        config_.anticogging.calib_anticogging = true;
    }
}

float Controller::remove_anticogging_bias()
{
    auto& cogmap = config_.anticogging.cogging_map;
    
    auto sum = std::accumulate(std::begin(cogmap), std::end(cogmap), 0.0f);
    auto average = sum / std::size(cogmap);

    for(auto& val : cogmap) {
        val -= average;
    }

    return average;
}


/*
 * This anti-cogging implementation iterates through each encoder position,
 * waits for zero velocity & position error,
 * then samples the current required to maintain that position.
 * 
 * This holding current is added as a feedforward term in the control loop.
 */
bool Controller::anticogging_calibration(float pos_estimate, float vel_estimate) {
    const float cogging_ratio = axis_->encoder_.getCoggingRatio();
    const float calibration_cpr = axis_->encoder_.getCoggingCalibrationCpr();

    if (!anticogging_calibration_initialized_) {
        const float grid_position = floorf(pos_estimate / cogging_ratio);
        const int32_t grid_index = (int32_t)grid_position;
        anticogging_start_index_ = (uint32_t)mod(grid_index, 3600);
        anticogging_start_pos_ = grid_position * cogging_ratio;
        anticogging_calibration_initialized_ = true;
        input_pos_ = anticogging_start_pos_;
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        return false;
    }

    float pos_err = input_pos_ - pos_estimate;
    if (std::abs(pos_err) <= config_.anticogging.calib_pos_threshold / calibration_cpr &&
        std::abs(vel_estimate) < config_.anticogging.calib_vel_threshold / calibration_cpr) {
        const uint32_t map_index = (anticogging_start_index_ + config_.anticogging.index) % 3600;
        config_.anticogging.cogging_map[map_index] = vel_integrator_torque_;
        ++config_.anticogging.index;
    }
    if (config_.anticogging.index < 3600) {
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        input_pos_ = anticogging_start_pos_ + config_.anticogging.index * cogging_ratio;
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        return false;
    } else {
        config_.anticogging.index = 0;
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        input_pos_ = anticogging_start_pos_;
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        anticogging_valid_ = true;
        anticogging_calibration_initialized_ = false;
        config_.anticogging.calib_anticogging = false;
        return true;
    }
}

void Controller::set_input_pos_and_steps(float const pos) {
    input_pos_ = pos;
    if (config_.circular_setpoints) {
        float const range = config_.circular_setpoint_range;
        axis_->steps_ = (int64_t)(fmodf_pos(pos, range) / range * config_.steps_per_circular_range);
    } else {
        axis_->steps_ = (int64_t)(pos * config_.steps_per_circular_range);
    }
}

void Controller::set_mit_input(float pos_rad, float vel_rad_per_s, float kp, float kd, float torque_ff) {
    // Reject non-finite frames rather than poisoning the control loop.
    if (!std::isfinite(pos_rad) || !std::isfinite(vel_rad_per_s) ||
        !std::isfinite(kp) || !std::isfinite(kd) || !std::isfinite(torque_ff)) {
        return;
    }
    // kp / kd are gains and must not be negative.
    mit_pos_rad_ = pos_rad;
    mit_vel_rad_per_s_ = vel_rad_per_s;
    mit_kp_ = std::max(kp, 0.0f);
    mit_kd_ = std::max(kd, 0.0f);
    mit_torque_ff_ = torque_ff;
}

bool Controller::control_mode_updated() {
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
}

float Controller::update_adrc_torque(float pos_estimate, float vel_estimate,
                                     float torque_cmd) {
    if (!adrc_initialized_) {
        adrc_z1_ = pos_estimate;
        adrc_z2_ = vel_estimate;
        adrc_z3_ = 0.0f;
        adrc_last_torque_ = 0.0f;
        adrc_initialized_ = true;
    }

    const float wo = std::clamp(adrc_bandwidth_, 1.0f, 0.25f * current_meas_hz);
    const float beta1 = 3.0f * wo;
    const float beta2 = 3.0f * wo * wo;
    const float beta3 = wo * wo * wo;
    const float b0 = std::max(adrc_b0_, 1.0e-6f);

    const float e = adrc_z1_ - pos_estimate;
    adrc_z1_ += current_meas_period * (adrc_z2_ - beta1 * e);
    adrc_z2_ += current_meas_period * (adrc_z3_ - beta2 * e + b0 * adrc_last_torque_);
    adrc_z3_ += current_meas_period * (-beta3 * e);

    if (std::isfinite(adrc_disturbance_limit_)) {
        const float lim = std::abs(adrc_disturbance_limit_);
        adrc_z3_ = std::clamp(adrc_z3_, -lim, lim);
    }

    return torque_cmd - adrc_z3_ / b0;
}

float Controller::update_adrc(float pos_estimate, float vel_estimate,
                              float pos_setpoint, float vel_setpoint) {
    const float pos_err = pos_setpoint - pos_estimate;
    const float vel_err = vel_setpoint - vel_estimate;
    const float desired_accel = adrc_pos_gain_ * pos_err + adrc_vel_gain_ * vel_err;
    const float b0 = std::max(adrc_b0_, 1.0e-6f);
    return update_adrc_torque(pos_estimate, vel_estimate, desired_accel / b0);
}

static float limitVel(const float vel_limit, const float vel_estimate, const float vel_gain, const float torque) {
    float Tmax = (vel_limit - vel_estimate) * vel_gain;
    float Tmin = (-vel_limit - vel_estimate) * vel_gain;
    return std::clamp(torque, Tmin, Tmax);
}

bool Controller::update() {
    ControlTimeout::mark_running(*axis_);

    std::optional<float> pos_estimate_linear = pos_estimate_linear_src_.present();
    std::optional<float> pos_estimate_circular = pos_estimate_circular_src_.present();
    std::optional<float> pos_wrap = pos_wrap_src_.present();
    std::optional<float> vel_estimate = vel_estimate_src_.present();

    std::optional<float> anticogging_pos_estimate = axis_->encoder_.pos_estimate_.present();
    std::optional<float> anticogging_vel_estimate = axis_->encoder_.vel_estimate_.present();

    if (axis_->step_dir_active_) {
        if (config_.circular_setpoints) {
            if (!pos_wrap.has_value()) {
                set_error(ERROR_INVALID_CIRCULAR_RANGE);
                return false;
            }
            input_pos_ = (float)(axis_->steps_ % config_.steps_per_circular_range) * (*pos_wrap / (float)(config_.steps_per_circular_range));
        } else {
            input_pos_ = (float)(axis_->steps_) / (float)(config_.steps_per_circular_range);
        }
    }

    if (config_.anticogging.calib_anticogging) {
        if (!anticogging_pos_estimate.has_value() || !anticogging_vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        // non-blocking
        anticogging_calibration(*anticogging_pos_estimate, *anticogging_vel_estimate);
    }

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
            float max_step_size = std::abs(current_meas_period * rate);
            float full_step = target_vel - vel_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            vel_setpoint_ += step;
            torque_setpoint_ = (step / current_meas_period) * config_.inertia;
            if (ControlTimeout::quick_stop_active(*axis_) && std::abs(vel_setpoint_) < 1e-3f) {
                vel_setpoint_ = 0.0f;
                ControlTimeout::mark_holding(*axis_);
            }
        } break;
        case INPUT_MODE_TORQUE_RAMP: {
            float max_step_size = std::abs(current_meas_period * config_.torque_ramp_rate);
            float full_step = input_torque_ - torque_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            torque_setpoint_ += step;
        } break;
        case INPUT_MODE_POS_FILTER: {
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
            vel_setpoint_ += current_meas_period * accel; // delta vel
            pos_setpoint_ += current_meas_period * vel_setpoint_; // Delta pos
        } break;
        case INPUT_MODE_TRAP_TRAJ: {
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
                axis_->trap_traj_.t_ += current_meas_period;
            }
            anticogging_pos_estimate = pos_setpoint_; // FF the position setpoint instead of the pos_estimate
        } break;
        case INPUT_MODE_TUNING: {
            autotuning_phase_ = wrap_pm_pi(autotuning_phase_ + (2.0f * M_PI * autotuning_.frequency * current_meas_period));
            float c = our_arm_cos_f32(autotuning_phase_);
            float s = our_arm_sin_f32(autotuning_phase_);
            pos_setpoint_ = input_pos_ + autotuning_.pos_amplitude * s; // + pos_amp_c * c
            vel_setpoint_ = input_vel_ + autotuning_.vel_amplitude * c;
            torque_setpoint_ = input_torque_ + autotuning_.torque_amplitude * -s;
        } break;
        case INPUT_MODE_MIT: {
            // MIT-style packed control: compute torque directly from the
            // per-frame kp/kd/t_ff against the current encoder estimates.
            // MIT semantics expect the computed torque to reach the motor
            // unmodified, so require torque control mode. In any other mode the
            // velocity PI / position loop would re-process torque_setpoint_ and
            // the result would no longer match the MIT command.
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

    const bool mit_adrc_active = adrc_enabled_ &&
        config_.control_mode == CONTROL_MODE_TORQUE_CONTROL &&
        config_.input_mode == INPUT_MODE_MIT;

    // Position control
    // TODO Decide if we want to use encoder or pll position here
    float gain_scheduling_multiplier = 1.0f;
    float vel_des = vel_setpoint_;
    bool adrc_active = adrc_enabled_ && config_.control_mode >= CONTROL_MODE_VELOCITY_CONTROL;
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

        if (!adrc_active && config_.pos_integrator_gain > 0.0f) {
            pos_integrator_vel_ += config_.pos_integrator_gain * current_meas_period * pos_err;
            const float pos_integrator_limit = std::abs(config_.vel_limit);
            if (std::isfinite(pos_integrator_limit)) {
                pos_integrator_vel_ = std::clamp(pos_integrator_vel_, -pos_integrator_limit, pos_integrator_limit);
            }
        } else {
            pos_integrator_vel_ = 0.0f;
        }

        if (!adrc_active) {
            vel_des += config_.pos_gain * pos_err + pos_integrator_vel_;
        }
        // V-shaped gain shedule based on position error
        float abs_pos_err = std::abs(pos_err);
        if (config_.enable_gain_scheduling && abs_pos_err <= config_.gain_scheduling_width) {
            gain_scheduling_multiplier = abs_pos_err / config_.gain_scheduling_width;
        }
    } else {
        pos_integrator_vel_ = 0.0f;
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
            overspeed_time_ += current_meas_period;
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

    // TODO: Change to controller working in torque units
    // Torque per amp gain scheduling (ACIM)
    float vel_gain = config_.vel_gain;
    float vel_integrator_gain = config_.vel_integrator_gain;
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_ACIM) {
        float effective_flux = axis_->acim_estimator_.rotor_flux_;
        float minflux = axis_->motor_.config_.acim_gain_min_flux;
        if (std::abs(effective_flux) < minflux)
            effective_flux = std::copysignf(minflux, effective_flux);
        vel_gain /= effective_flux;
        vel_integrator_gain /= effective_flux;
        // TODO: also scale the integral value which is also changing units.
        // (or again just do control in torque units)
    }

    // Velocity control
    float torque = torque_setpoint_;

    // Anti-cogging is enabled after calibration
    // We get the current position and apply a current feed-forward
    // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
    if (anticogging_valid_ && config_.anticogging.anticogging_enabled) {
        if (!anticogging_pos_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        float anticogging_pos = *anticogging_pos_estimate / axis_->encoder_.getCoggingRatio();
        const int32_t anticogging_index = (int32_t)floorf(anticogging_pos);
        torque += config_.anticogging.cogging_map[mod(anticogging_index, 3600)];
    }

    float v_err = 0.0f;
    if (mit_adrc_active) {
        if (!adrc_measurement_valid) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        torque = update_adrc_torque(adrc_pos_estimate, adrc_vel_estimate, torque);
        vel_integrator_torque_ = 0.0f;
    } else if (adrc_active) {
        if (!adrc_measurement_valid) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        const float adrc_pos_setpoint = config_.control_mode >= CONTROL_MODE_POSITION_CONTROL
            ? pos_setpoint_
            : adrc_pos_estimate;
        torque += update_adrc(adrc_pos_estimate, adrc_vel_estimate,
                              adrc_pos_setpoint, vel_setpoint_);
        vel_integrator_torque_ = 0.0f;
    } else if (config_.control_mode >= CONTROL_MODE_VELOCITY_CONTROL) {
        if (!vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }

        v_err = vel_des - *vel_estimate;
        torque += (vel_gain * gain_scheduling_multiplier) * v_err;

        // Velocity integral action before limiting
        torque += vel_integrator_torque_;
    }

    // Velocity limiting in current mode
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL && config_.enable_torque_mode_vel_limit) {
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
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL || adrc_active || mit_adrc_active) {
        // reset integral if not in use
        vel_integrator_torque_ = 0.0f;
    } else {
        if (limited) {
            // TODO make decayfactor configurable
            vel_integrator_torque_ *= 0.99f;
        } else {
            vel_integrator_torque_ += ((vel_integrator_gain * gain_scheduling_multiplier) * current_meas_period) * v_err;
        }
        // integrator limiting to prevent windup 
        vel_integrator_torque_ = std::clamp(vel_integrator_torque_, -config_.vel_integrator_limit, config_.vel_integrator_limit);
    }

    float ideal_electrical_power = 0.0f;
    if (axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL) {
        ideal_electrical_power = axis_->motor_.current_control_.power_ - \
            SQ(axis_->motor_.current_control_.Iq_measured_) * 1.5f * axis_->motor_.config_.phase_resistance - \
            SQ(axis_->motor_.current_control_.Id_measured_) * 1.5f * axis_->motor_.config_.phase_resistance;
    }
    else {
        ideal_electrical_power = axis_->motor_.current_control_.power_;
    }
    mechanical_power_ += config_.mechanical_power_bandwidth * current_meas_period * (torque * *vel_estimate * M_PI * 2.0f - mechanical_power_);
    electrical_power_ += config_.electrical_power_bandwidth * current_meas_period * (ideal_electrical_power - electrical_power_);

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

    torque_output_ = torque * controller_to_motor_torque;

    // TODO: this is inconsistent with the other errors which are sticky.
    // However if we make ERROR_INVALID_ESTIMATE sticky then it will be
    // confusing that a normal sequence of motor calibration + encoder
    // calibration would leave the controller in an error state.
    error_ &= ~ERROR_INVALID_ESTIMATE;
    return true;
}
