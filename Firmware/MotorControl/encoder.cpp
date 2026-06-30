
#include "odrive_main.h"
#include "debug_counters.hpp"
#include <Drivers/STM32/stm32_system.h>
#include <algorithm>

Encoder::Encoder(Stm32SpiArbiter* spi_arbiter) :
        spi_arbiter_(spi_arbiter)
{
}

static uint32_t spi_prescaler_from_divisor(uint16_t divisor) {
    switch (divisor) {
        case 2: return SPI_BAUDRATEPRESCALER_2;
        case 4: return SPI_BAUDRATEPRESCALER_4;
        case 8: return SPI_BAUDRATEPRESCALER_8;
        case 16: return SPI_BAUDRATEPRESCALER_16;
        case 32: return SPI_BAUDRATEPRESCALER_32;
        case 64: return SPI_BAUDRATEPRESCALER_64;
        case 128: return SPI_BAUDRATEPRESCALER_128;
        case 256: return SPI_BAUDRATEPRESCALER_256;
        default: return SPI_BAUDRATEPRESCALER_16;
    }
}

bool Encoder::apply_config(ODriveIntf::MotorIntf::MotorType motor_type) {
    config_.parent = this;

    update_pll_gains();

    if (config_.pre_calibrated) {
        if (motor_type == Motor::MOTOR_TYPE_ACIM)
            is_ready_ = true;
    }

    return true;
}

void Encoder::setup() {
    mode_ = config_.mode;

    if(mode_ & MODE_FLAG_ABS){
        abs_spi_cs_pin_init();

        if (mode_ == MODE_SPI_ABS_MT6826S || mode_ == MODE_SPI_ABS_MT6826S_VERNIER) {
            Mt6826sSpi::Config mt6826s_config = make_mt6826s_spi_config();
            mt6826s_spi_.init(spi_arbiter_, abs_spi_cs_gpio_, mt6826s_config);

            if (mode_ == MODE_SPI_ABS_MT6826S_VERNIER) {
                abs_spi_aux_cs_pin_init();
                mt6826s_aux_spi_.init(spi_arbiter_, abs_spi_aux_cs_gpio_, mt6826s_config);
                mt6826s_spi_pair_.init(&mt6826s_spi_, &mt6826s_aux_spi_);
                vernier_resolver_.init(make_vernier_resolver_config());
            }
        }

        if (axis_->controller_.config_.anticogging.pre_calibrated) {
            axis_->controller_.anticogging_valid_ = true;
        }
    }
}

void Encoder::set_error(Error error) {
    vel_estimate_valid_ = false;
    pos_estimate_valid_ = false;
    error_ |= error;
    axis_->error_ |= Axis::ERROR_ENCODER_FAILED;
}

bool Encoder::do_checks(){
    return error_ == ERROR_NONE;
}

void Encoder::update_pll_gains() {
    pll_kp_ = 2.0f * config_.bandwidth;  // basic conversion to discrete time
    pll_ki_ = 0.25f * (pll_kp_ * pll_kp_); // Critically damped

    // Check that we don't get problems with discrete time approximation
    if (!(current_meas_period * pll_kp_ < 1.0f)) {
        set_error(ERROR_UNSTABLE_GAIN);
    }
}

void Encoder::check_pre_calibrated() {
    // TODO: restoring config from python backup is fragile here (ACIM motor type must be set first)
    if (axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_ACIM) {
        if (!is_ready_)
            config_.pre_calibrated = false;
    }
}

// Function that sets the current encoder count to a desired 32-bit value.
void Encoder::set_linear_count(int32_t count) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();

    // Update states
    shadow_count_ = count;
    pos_estimate_counts_ = (float)count;

    cpu_exit_critical(prim);
}

// Function that sets the CPR circular tracking encoder count to a desired 32-bit value.
// Note that this will get mod'ed down to [0, cpr)
void Encoder::set_circular_count(int32_t count, bool update_offset) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();

    if (update_offset) {
        config_.phase_offset += count - count_in_cpr_;
        config_.phase_offset = mod(config_.phase_offset, config_.cpr);
    }

    // Update states
    count_in_cpr_ = mod(count, config_.cpr);
    pos_cpr_counts_ = (float)count_in_cpr_;

    cpu_exit_critical(prim);
}

// @brief Turns the motor in one direction for a bit and then in the other
// direction in order to find the offset between the electrical phase 0
// and the encoder state 0.
bool Encoder::run_offset_calibration() {
    const float start_lock_duration = 1.0f;

    // We use shadow_count_ to do the calibration, but the offset is used by count_in_cpr_
    // Therefore we have to sync them for calibration
    shadow_count_ = count_in_cpr_;

    CRITICAL_SECTION() {
        // Reset state variables
        axis_->open_loop_controller_.Idq_setpoint_ = {0.0f, 0.0f};
        axis_->open_loop_controller_.Vdq_setpoint_ = {0.0f, 0.0f};
        axis_->open_loop_controller_.phase_ = 0.0f;
        axis_->open_loop_controller_.phase_vel_ = 0.0f;

        float max_current_ramp = axis_->motor_.config_.calibration_current / start_lock_duration * 2.0f;
        axis_->open_loop_controller_.max_current_ramp_ = max_current_ramp;
        axis_->open_loop_controller_.max_voltage_ramp_ = max_current_ramp;
        axis_->open_loop_controller_.max_phase_vel_ramp_ = INFINITY;
        axis_->open_loop_controller_.target_current_ = axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL ? axis_->motor_.config_.calibration_current : 0.0f;
        axis_->open_loop_controller_.target_voltage_ = axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL ? 0.0f : axis_->motor_.config_.calibration_current;
        axis_->open_loop_controller_.target_vel_ = 0.0f;
        axis_->open_loop_controller_.total_distance_ = 0.0f;
        axis_->open_loop_controller_.phase_ = axis_->open_loop_controller_.initial_phase_ = wrap_pm_pi(0 - config_.calib_scan_distance / 2.0f);

        axis_->motor_.current_control_.enable_current_control_src_ = (axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL);
        axis_->motor_.current_control_.Idq_setpoint_src_.connect_to(&axis_->open_loop_controller_.Idq_setpoint_);
        axis_->motor_.current_control_.Vdq_setpoint_src_.connect_to(&axis_->open_loop_controller_.Vdq_setpoint_);
        
        axis_->motor_.current_control_.phase_src_.connect_to(&axis_->open_loop_controller_.phase_);
        axis_->acim_estimator_.rotor_phase_src_.connect_to(&axis_->open_loop_controller_.phase_);

        axis_->motor_.phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
        axis_->motor_.current_control_.phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
        axis_->acim_estimator_.rotor_phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
    }
    axis_->wait_for_control_iteration();

    axis_->motor_.arm(&axis_->motor_.current_control_);

    // go to start position of forward scan for start_lock_duration to get ready to scan
    for (size_t i = 0; i < (size_t)(start_lock_duration * 1000.0f); ++i) {
        if (!axis_->motor_.is_armed_) {
            return false; // TODO: return "disarmed" error code
        }
        if (axis_->requested_state_ != Axis::AXIS_STATE_UNDEFINED) {
            axis_->motor_.disarm();
            return false; // TODO: return "aborted" error code
        }
        osDelay(1);
    }


    int32_t init_enc_val = shadow_count_;
    uint32_t num_steps = 0;
    int64_t encvaluesum = 0;

    CRITICAL_SECTION() {
        axis_->open_loop_controller_.target_vel_ = config_.calib_scan_omega;
        axis_->open_loop_controller_.total_distance_ = 0.0f;
    }

    // scan forward
    while ((axis_->requested_state_ == Axis::AXIS_STATE_UNDEFINED) && axis_->motor_.is_armed_) {
        bool reached_target_dist = axis_->open_loop_controller_.total_distance_.any().value_or(-INFINITY) >= config_.calib_scan_distance;
        if (reached_target_dist) {
            break;
        }
        encvaluesum += shadow_count_;
        num_steps++;
        osDelay(1);
    }

    // Check response and direction
    if (shadow_count_ > init_enc_val + 8) {
        // motor same dir as encoder
        config_.direction = 1;
    } else if (shadow_count_ < init_enc_val - 8) {
        // motor opposite dir as encoder
        config_.direction = -1;
    } else {
        // Encoder response error
        set_error(ERROR_NO_RESPONSE);
        axis_->motor_.disarm();
        return false;
    }

    // Check CPR
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float expected_encoder_delta = config_.calib_scan_distance / elec_rad_per_enc;
    float calib_scan_response = std::abs(shadow_count_ - init_enc_val);
    if (std::abs(calib_scan_response - expected_encoder_delta) / expected_encoder_delta > config_.calib_range) {
        set_error(ERROR_CPR_POLEPAIRS_MISMATCH);
        axis_->motor_.disarm();
        return false;
    }

    CRITICAL_SECTION() {
        axis_->open_loop_controller_.target_vel_ = -config_.calib_scan_omega;
    }

    // scan backwards
    while ((axis_->requested_state_ == Axis::AXIS_STATE_UNDEFINED) && axis_->motor_.is_armed_) {
        bool reached_target_dist = axis_->open_loop_controller_.total_distance_.any().value_or(INFINITY) <= 0.0f;
        if (reached_target_dist) {
            break;
        }
        encvaluesum += shadow_count_;
        num_steps++;
        osDelay(1);
    }

    // Motor disarmed because of an error
    if (!axis_->motor_.is_armed_) {
        return false;
    }

    axis_->motor_.disarm();

    config_.phase_offset = encvaluesum / num_steps;
    int32_t residual = encvaluesum - ((int64_t)config_.phase_offset * (int64_t)num_steps);
    config_.phase_offset_float = (float)residual / (float)num_steps + 0.5f;  // add 0.5 to center-align state to phase

    is_ready_ = true;
    return true;
}

void Encoder::sample_now() {
    switch (mode_) {
        case MODE_SPI_ABS_MT6826S: {
            start_mt6826s_main_sample();
            // Do nothing
        } break;

        case MODE_SPI_ABS_MT6826S_VERNIER: {
            start_mt6826s_pair_sample();
        } break;

        default: {
           set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
        } break;
    }
}

void Encoder::mt6826s_spi_cb(void* ctx, const Mt6826sSpi::Sample& sample, bool success) {
    reinterpret_cast<Encoder*>(ctx)->handle_mt6826s_spi_cb(sample, success);
}

void Encoder::handle_mt6826s_spi_cb(const Mt6826sSpi::Sample& sample, bool success) {
    uint32_t prim = cpu_enter_critical();
    mt6826s_main_sample_ = sample;
    cpu_exit_critical(prim);

    if (!success || !sample.valid) {
        return;
    }

    pos_abs_ = sample.angle;
    abs_spi_pos_updated_ = true;

    if (config_.pre_calibrated) {
        is_ready_ = true;
    }
}

bool Encoder::start_mt6826s_main_sample() {
    if (mt6826s_spi_.start_sample_async(&Encoder::mt6826s_spi_cb, this)) {
        return true;
    }

    // MT6826S reads are asynchronous, so a single busy cycle is expected
    // if the previous DMA transaction has not completed yet. Let update()
    // decide if the missing sample rate is persistent enough to be a fault.
    return false;
}

bool Encoder::start_mt6826s_pair_sample() {
    if (mt6826s_spi_pair_.start_sample_async(&Encoder::mt6826s_spi_pair_cb, this)) {
        ++mt6826s_vernier_sample_counter_;
        return true;
    }

    // Pair sampling can miss a control cycle while the main/aux transfers
    // are still in flight. This is not by itself an encoder fault.
    extern DebugCounters g_debug;
    ++g_debug.enc_pair_busy_cnt;
    return false;
}

void Encoder::mt6826s_spi_pair_cb(void* ctx, const Mt6826sSpiPair::PairSample& sample, bool success) {
    reinterpret_cast<Encoder*>(ctx)->handle_mt6826s_spi_pair_cb(sample, success);
}

void Encoder::handle_mt6826s_spi_pair_cb(const Mt6826sSpiPair::PairSample& sample, bool success) {
    extern DebugCounters g_debug;
    ++g_debug.enc_pair_ok_cnt;

    if (sample.main.valid) {
        handle_mt6826s_spi_cb(sample.main, true);
    }

    VernierResolver::Result resolver_result = vernier_resolver_.update(
        sample.main.angle,
        sample.main.valid,
        sample.aux.angle,
        sample.aux.valid
    );

    uint32_t prim = cpu_enter_critical();
    mt6826s_pair_valid_ = success && sample.valid;
    vernier_result_ = resolver_result;
    if (sample.main.sequence != 0) {
        mt6826s_main_sample_ = sample.main;
    }
    if (sample.aux.sequence != 0) {
        mt6826s_aux_sample_ = sample.aux;
    }
    if (sample.sequence != 0) {
        mt6826s_pair_sequence_ = sample.sequence;
    }
    cpu_exit_critical(prim);
}

void Encoder::abs_spi_cs_pin_init(){
    // Decode and init cs pin
    abs_spi_cs_gpio_ = get_gpio(config_.abs_spi_cs_gpio_pin);
    abs_spi_cs_gpio_.config(GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);

    // Write pin high
    abs_spi_cs_gpio_.write(true);
}

void Encoder::abs_spi_aux_cs_pin_init(){
    abs_spi_aux_cs_gpio_ = get_gpio(config_.abs_spi_aux_cs_gpio_pin);
    abs_spi_aux_cs_gpio_.config(GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);
    abs_spi_aux_cs_gpio_.write(true);
}

Mt6826sSpi::Config Encoder::make_mt6826s_spi_config() const {
    Mt6826sSpi::Config mt6826s_config = {};
    mt6826s_config.check_crc = true;
    mt6826s_config.check_fixed_bits = true;
    mt6826s_config.fail_on_status_warning = false;
    mt6826s_config.baudrate_prescaler = spi_prescaler_from_divisor(config_.mt6826s_spi_prescaler);

    switch (config_.mt6826s_spi_mode) {
        case 0:
            mt6826s_config.clk_polarity = SPI_POLARITY_LOW;
            mt6826s_config.clk_phase = SPI_PHASE_1EDGE;
            break;
        case 1:
            mt6826s_config.clk_polarity = SPI_POLARITY_LOW;
            mt6826s_config.clk_phase = SPI_PHASE_2EDGE;
            break;
        case 2:
            mt6826s_config.clk_polarity = SPI_POLARITY_HIGH;
            mt6826s_config.clk_phase = SPI_PHASE_1EDGE;
            break;
        case 3:
        default:
            mt6826s_config.clk_polarity = SPI_POLARITY_HIGH;
            mt6826s_config.clk_phase = SPI_PHASE_2EDGE;
            break;
    }

    return mt6826s_config;
}

void Encoder::apply_mt6826s_spi_config() {
    Mt6826sSpi::Config mt6826s_config = make_mt6826s_spi_config();
    mt6826s_spi_.set_config(mt6826s_config);
    mt6826s_aux_spi_.set_config(mt6826s_config);
}

VernierResolver::Config Encoder::make_vernier_resolver_config() const {
    VernierResolver::Config vernier_config = {};
    vernier_config.angle_counts_per_rev = Mt6826sSpi::kCountsPerRev;
    vernier_config.main_ratio = config_.vernier_main_ratio;
    vernier_config.aux_ratio = config_.vernier_aux_ratio;
    vernier_config.main_offset = config_.vernier_main_offset;
    vernier_config.aux_offset = config_.vernier_aux_offset;
    vernier_config.main_reversed = config_.vernier_main_reversed;
    vernier_config.aux_reversed = config_.vernier_aux_reversed;
    vernier_config.output_reversed = config_.vernier_output_reversed;
    vernier_config.err_accept = config_.vernier_err_accept;
    vernier_config.err_reject = config_.vernier_err_reject;
    vernier_config.max_main_cycle_index = 64;
    vernier_config.use_phase_difference = true;
    return vernier_config;
}

void Encoder::apply_vernier_resolver_config() {
    vernier_resolver_.init(make_vernier_resolver_config());
    vernier_output_estimate_valid_ = false;
    vernier_output_sample_dt_ = 0.0f;
    vernier_output_pair_sequence_ = 0;
}

void Encoder::publish_vernier_output_estimate(float dt) {
    VernierResolver::Result result = {};
    uint32_t pair_sequence = 0;
    uint32_t prim = cpu_enter_critical();
    result = vernier_result_;
    pair_sequence = mt6826s_pair_sequence_;
    cpu_exit_critical(prim);

    if (!result.valid || dt <= 0.0f) {
        return;
    }

    vernier_output_sample_dt_ += dt;

    if (!vernier_output_estimate_valid_) {
        vernier_output_pos_estimate_ = result.position_turns;
        vernier_output_vel_estimate_ = 0.0f;
        vernier_output_estimate_valid_ = true;
        vernier_output_pair_sequence_ = pair_sequence;
        vernier_output_sample_dt_ = 0.0f;
    } else if (pair_sequence != vernier_output_pair_sequence_ && vernier_output_sample_dt_ > 0.0f) {
        const float measured_vel = (result.position_turns - vernier_output_pos_estimate_) / vernier_output_sample_dt_;
        const float alpha = std::min(1.0f, vernier_output_sample_dt_ * config_.bandwidth);
        vernier_output_vel_estimate_ += alpha * (measured_vel - vernier_output_vel_estimate_);
        vernier_output_pos_estimate_ = result.position_turns;
        vernier_output_pair_sequence_ = pair_sequence;
        vernier_output_sample_dt_ = 0.0f;
    }

    pos_estimate_ = vernier_output_pos_estimate_;
    // vel_estimate_ is now always sourced from the motor-side PLL (see update()).
    pos_circular_ = fmodf_pos(vernier_output_pos_estimate_,
                              axis_->controller_.config_.circular_setpoint_range);
}

void Encoder::get_vernier_diagnostics_snapshot(VernierDiagnosticsSnapshot* out) {
    if (!out) {
        return;
    }

    uint32_t prim = cpu_enter_critical();
    out->main_sample = mt6826s_main_sample_;
    out->aux_sample = mt6826s_aux_sample_;
    VernierResolver::Result resolver_result = vernier_result_;
    out->pair_count = mt6826s_pair_sequence_;
    out->pair_valid = mt6826s_pair_valid_;
    cpu_exit_critical(prim);

    Mt6826sSpi::Sample latest_main = {};
    if (mt6826s_spi_.read_last_sample_for_diagnostics(&latest_main)) {
        out->main_sample = latest_main;
    }

    Mt6826sSpi::Sample latest_aux = {};
    if (mt6826s_aux_spi_.read_last_sample_for_diagnostics(&latest_aux)) {
        out->aux_sample = latest_aux;
    }

    out->virtual_count = 0;
    vernier_resolver_.get_virtual_count(config_.vernier_virtual_cpr, &out->virtual_count);
    out->position_turns = resolver_result.position_turns;
    out->residual = resolver_result.residual_turns;
    out->encoder_pos_estimate = pos_estimate_.any().value_or(0.0f);
    out->encoder_vel_estimate = vel_estimate_.any().value_or(0.0f);
    out->encoder_pos_circular = pos_circular_.any().value_or(0.0f);
    out->state = static_cast<uint32_t>(resolver_result.state);
    out->main_spi_dma_error_count = mt6826s_spi_.spi_dma_error_count();
    out->main_crc_error_count = mt6826s_spi_.crc_error_count();
    out->main_fixed_bit_error_count = mt6826s_spi_.fixed_bit_error_count();
    out->main_status_warning_count = mt6826s_spi_.status_warning_count();
    out->main_sample_count = mt6826s_spi_.sample_count();
    out->main_error_count = out->main_crc_error_count
        + out->main_fixed_bit_error_count
        + out->main_spi_dma_error_count;
    out->aux_spi_dma_error_count = mt6826s_aux_spi_.spi_dma_error_count();
    out->aux_crc_error_count = mt6826s_aux_spi_.crc_error_count();
    out->aux_fixed_bit_error_count = mt6826s_aux_spi_.fixed_bit_error_count();
    out->aux_status_warning_count = mt6826s_aux_spi_.status_warning_count();
    out->aux_sample_count = mt6826s_aux_spi_.sample_count();
    out->aux_error_count = out->aux_crc_error_count
        + out->aux_fixed_bit_error_count
        + out->aux_spi_dma_error_count;
    out->pair_error_count = mt6826s_spi_pair_.pair_error_count();
}

bool Encoder::update() {
    // update internal encoder state.
    int32_t delta_enc = 0;
    int32_t pos_abs_latched = pos_abs_; //LATCH

    switch (mode_) {
        case MODE_SPI_ABS_MT6826S:
        case MODE_SPI_ABS_MT6826S_VERNIER: {
            if (abs_spi_pos_updated_ == false) {
                // Low pass filter the error
                spi_error_rate_ += current_meas_period * (1.0f - spi_error_rate_);
                if (spi_error_rate_ > 0.05f) {
                    set_error(ERROR_ABS_SPI_COM_FAIL);
                    return false;
                }
            } else {
                // Low pass filter the error
                spi_error_rate_ += current_meas_period * (0.0f - spi_error_rate_);
            }

            abs_spi_pos_updated_ = false;
            delta_enc = pos_abs_latched - count_in_cpr_; //LATCH
            delta_enc = mod(delta_enc, config_.cpr);
            if (delta_enc > config_.cpr/2) {
                delta_enc -= config_.cpr;
            }

        }break;
        default: {
            set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
            return false;
        } break;
    }

    shadow_count_ += delta_enc;
    count_in_cpr_ += delta_enc;
    count_in_cpr_ = mod(count_in_cpr_, config_.cpr);

    if(mode_ & MODE_FLAG_ABS)
        count_in_cpr_ = pos_abs_latched;

    // Memory for pos_circular
    float pos_cpr_counts_last = pos_cpr_counts_;

    //// run pll (for now pll is in units of encoder counts)
    // Predict current pos
    pos_estimate_counts_ += current_meas_period * vel_estimate_counts_;
    pos_cpr_counts_      += current_meas_period * vel_estimate_counts_;
    // Encoder model
    auto encoder_model = [](float internal_pos)->int32_t {
        return (int32_t)std::floor(internal_pos);
    };
    // discrete phase detector
    float delta_pos_counts = (float)(shadow_count_ - encoder_model(pos_estimate_counts_));
    float delta_pos_cpr_counts = (float)(count_in_cpr_ - encoder_model(pos_cpr_counts_));
    delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float)(config_.cpr));
    delta_pos_cpr_counts_ += 0.1f * (delta_pos_cpr_counts - delta_pos_cpr_counts_); // for debug
    // pll feedback
    pos_estimate_counts_ += current_meas_period * pll_kp_ * delta_pos_counts;
    pos_cpr_counts_ += current_meas_period * pll_kp_ * delta_pos_cpr_counts;
    pos_cpr_counts_ = fmodf_pos(pos_cpr_counts_, (float)(config_.cpr));
    vel_estimate_counts_ += current_meas_period * pll_ki_ * delta_pos_cpr_counts;
    bool snap_to_zero_vel = false;
    if (std::abs(vel_estimate_counts_) < 0.5f * current_meas_period * pll_ki_) {
        vel_estimate_counts_ = 0.0f;  //align delta-sigma on zero to prevent jitter
        snap_to_zero_vel = true;
    }

    const float motor_pos_estimate_turns = pos_estimate_counts_ / (float)config_.cpr;
    const float motor_vel_estimate_turns = vel_estimate_counts_ / (float)config_.cpr;

    // Motor-side velocity from PLL is always used for controller feedback.
    // It is fast, low-noise, and correct for commutation scale.
    vel_estimate_ = motor_vel_estimate_turns;

    // In vernier mode the pos_estimate_ comes from the output-shaft resolver
    // (absolute multi-turn).  For all other modes the motor-side PLL position
    // is used.
    if (mode_ != MODE_SPI_ABS_MT6826S_VERNIER) {
        pos_estimate_ = motor_pos_estimate_turns;
    }
    
    // TODO: we should strictly require that this value is from the previous iteration
    // to avoid spinout scenarios. However that requires a proper way to reset
    // the encoder from error states.
    float pos_circular = pos_circular_.any().value_or(0.0f);
    pos_circular +=  wrap_pm((pos_cpr_counts_ - pos_cpr_counts_last) / (float)config_.cpr, 1.0f);
    pos_circular = fmodf_pos(pos_circular, axis_->controller_.config_.circular_setpoint_range);
    if (mode_ != MODE_SPI_ABS_MT6826S_VERNIER) {
        pos_circular_ = pos_circular;
    }

    //// run encoder count interpolation
    int32_t corrected_enc = count_in_cpr_ - config_.phase_offset;
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel || !config_.enable_phase_interpolation) {
        interpolation_ = 0.5f;
    // reset interpolation if encoder edge comes
    // TODO: This isn't correct. At high velocities the first phase in this count may very well not be at the edge.
    } else if (delta_enc > 0) {
        interpolation_ = 0.0f;
    } else if (delta_enc < 0) {
        interpolation_ = 1.0f;
    } else {
        // Interpolate (predict) between encoder counts using vel_estimate,
        interpolation_ += current_meas_period * vel_estimate_counts_;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (interpolation_ > 1.0f) interpolation_ = 1.0f;
        if (interpolation_ < 0.0f) interpolation_ = 0.0f;
    }
    float interpolated_enc = corrected_enc + interpolation_;

    //// compute electrical phase
    //TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float ph = elec_rad_per_enc * (interpolated_enc - config_.phase_offset_float);
    
    if (is_ready_) {
        phase_ = wrap_pm_pi(ph) * config_.direction;
        phase_vel_ = (2*M_PI) * motor_vel_estimate_turns * axis_->motor_.config_.pole_pairs * config_.direction;
    }

    if (mode_ == MODE_SPI_ABS_MT6826S_VERNIER) {
        publish_vernier_output_estimate(current_meas_period);
    }

    return true;
}
