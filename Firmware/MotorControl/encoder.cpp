
#include "odrive_main.h"
#include "debug_counters.hpp"
#include <Drivers/STM32/stm32_system.h>
#include <algorithm>
#include <cmath>

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

bool Encoder::apply_config() {
    config_.parent = this;

    // Encoder/vernier model constants are baked per motor model
    // (production_config.h). Force-overwrite on every boot so NVM never
    // holds a stale/wrong value. Per-unit calibration (phase_offset,
    // vernier_main/aux_offset, pre_calibrated) is NOT overwritten.
    config_.mode = static_cast<Mode>(ODRIVE_PRODUCTION_ENCODER_MODE);
    config_.cpr = ODRIVE_PRODUCTION_ENCODER_CPR;
    config_.abs_spi_cs_gpio_pin = ODRIVE_PRODUCTION_ABS_SPI_CS_GPIO_PIN;
    config_.abs_spi_aux_cs_gpio_pin = ODRIVE_PRODUCTION_ABS_SPI_AUX_CS_GPIO_PIN;
    config_.vernier_virtual_cpr = ODRIVE_PRODUCTION_VERNIER_VIRTUAL_CPR;
    config_.vernier_main_ratio = ODRIVE_PRODUCTION_VERNIER_MAIN_RATIO;
    config_.vernier_aux_ratio = ODRIVE_PRODUCTION_VERNIER_AUX_RATIO;
    config_.vernier_main_reversed = ODRIVE_PRODUCTION_VERNIER_MAIN_REVERSED;
    config_.vernier_aux_reversed = ODRIVE_PRODUCTION_VERNIER_AUX_REVERSED;
    config_.vernier_output_reversed = ODRIVE_PRODUCTION_VERNIER_OUTPUT_REVERSED;
    config_.vernier_use_phase_difference = ODRIVE_PRODUCTION_VERNIER_USE_PHASE_DIFFERENCE;
    config_.mt6826s_spi_mode = ODRIVE_PRODUCTION_MT6826S_SPI_MODE;
    config_.mt6826s_spi_prescaler = ODRIVE_PRODUCTION_MT6826S_SPI_PRESCALER;
    config_.vernier_err_accept = ODRIVE_PRODUCTION_VERNIER_ERR_ACCEPT;
    config_.vernier_err_reject = ODRIVE_PRODUCTION_VERNIER_ERR_REJECT;

    // Migrate the legacy default stored by earlier firmware. Preserve any
    // genuinely tuned non-default value.
    if (std::abs(config_.bandwidth - 1000.0f) < 1.0e-3f) {
        config_.bandwidth = 458.25757f;
    }

    update_pll_gains();

    return true;
}

void Encoder::setup() {
    mode_ = config_.mode;

    abs_spi_cs_pin_init();
    Mt6826sSpi::Config mt6826s_config = make_mt6826s_spi_config();
    mt6826s_spi_.init(spi_arbiter_, abs_spi_cs_gpio_, mt6826s_config);

    if (mode_ == MODE_SPI_ABS_MT6826S_VERNIER) {
        abs_spi_aux_cs_pin_init();
        mt6826s_aux_spi_.init(spi_arbiter_, abs_spi_aux_cs_gpio_, mt6826s_config);
        mt6826s_spi_pair_.init(&mt6826s_spi_, &mt6826s_aux_spi_);
        vernier_resolver_.init(make_vernier_resolver_config());
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
    constexpr float kPllDamping = 0.7092503f;
    const float wn = std::max(config_.bandwidth, 0.0f);
    pll_kp_ = 2.0f * kPllDamping * wn;
    pll_ki_ = wn * wn;

    // Check that we don't get problems with discrete time approximation
    if (!(current_meas_period * pll_kp_ < 1.0f)) {
        set_error(ERROR_UNSTABLE_GAIN);
    }
}

void Encoder::check_pre_calibrated() {
    if (!is_ready_)
        config_.pre_calibrated = false;
}

// Function that sets the current encoder count to a desired 32-bit value.
void Encoder::set_linear_count(int32_t count) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = cpu_enter_critical();

    // Update states
    shadow_count_ = count;
    shadow_count64_ = count;
    pos_estimate_counts_ = (float)mod(count, config_.cpr);
    pos_cpr_counts_ = pos_estimate_counts_;
    if (mode_ == MODE_SPI_ABS_MT6826S_VERNIER &&
        vernier_main_continuous_valid_) {
        // Re-zeroing the diagnostic count must not look like physical motion
        // to the bounded joint-position tracker on the next control update.
        vernier_main_anchor_pos_ = vernier_main_continuous_pos_;
        vernier_anchor_shadow_count_ = shadow_count64_;
        vernier_anchor_valid_ = true;
        vernier_last_shadow_count_ = shadow_count64_;
        vernier_last_shadow_count_valid_ = true;
    }
    pll_previous_error_counts_ = 0.0f;
    pll_previous_error_valid_ = false;
    reset_controller_velocity_filter();

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
    pos_estimate_counts_ = (float)count_in_cpr_;
    pos_cpr_counts_ = pos_estimate_counts_;
    pll_previous_error_counts_ = 0.0f;
    pll_previous_error_valid_ = false;

    cpu_exit_critical(prim);
}

// @brief Turns the motor in one direction for a bit and then in the other
// direction in order to find the offset between the electrical phase 0
// and the encoder state 0.
bool Encoder::run_offset_calibration() {
    const float start_lock_duration = 1.0f;
    calibration_estimated_pole_pairs_ = 0;

    // We use shadow_count_ to do the calibration, but the offset is used by count_in_cpr_
    // Therefore we have to sync them for calibration
    shadow_count_ = count_in_cpr_;
    shadow_count64_ = count_in_cpr_;

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
        axis_->open_loop_controller_.target_current_ = axis_->motor_.config_.calibration_current;
        axis_->open_loop_controller_.target_voltage_ = 0.0f;
        axis_->open_loop_controller_.target_vel_ = 0.0f;
        axis_->open_loop_controller_.total_distance_ = 0.0f;
        axis_->open_loop_controller_.phase_ = axis_->open_loop_controller_.initial_phase_ = wrap_pm_pi(0 - config_.calib_scan_distance / 2.0f);

        axis_->motor_.current_control_.enable_current_control_src_ = true;
        axis_->motor_.current_control_.Idq_setpoint_src_.connect_to(&axis_->open_loop_controller_.Idq_setpoint_);
        axis_->motor_.current_control_.Vdq_setpoint_src_.connect_to(&axis_->open_loop_controller_.Vdq_setpoint_);
        
        axis_->motor_.current_control_.phase_src_.connect_to(&axis_->open_loop_controller_.phase_);

        axis_->motor_.phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
        axis_->motor_.current_control_.phase_vel_src_.connect_to(&axis_->open_loop_controller_.phase_vel_);
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

    // Identify pole pairs from commanded electrical travel and measured motor
    // encoder travel. Keep the integer only when the scan is close enough to
    // an integer model; this replaces a check against a preconfigured value.
    float calib_scan_response = std::abs(shadow_count_ - init_enc_val);
    const float estimated_pole_pairs = config_.calib_scan_distance * config_.cpr /
        (2.0f * M_PI * calib_scan_response);
    const int32_t integer_pole_pairs = static_cast<int32_t>(
        std::lround(estimated_pole_pairs));
    if (integer_pole_pairs < 1 || integer_pole_pairs > 128 ||
        std::abs(estimated_pole_pairs - integer_pole_pairs) /
            integer_pole_pairs > config_.calib_range) {
        set_error(ERROR_CPR_POLEPAIRS_MISMATCH);
        axis_->motor_.disarm();
        return false;
    }
    calibration_estimated_pole_pairs_ = integer_pole_pairs;

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
            // Lock with coherent pairs, then sample the main encoder at 10 kHz
            // and refresh/check the auxiliary absolute branch at 400 Hz.
            constexpr uint32_t kAuxSampleDivider = 25u;
            VernierResolver::Result result = {};
            uint32_t prim = cpu_enter_critical();
            result = vernier_result_;
            cpu_exit_critical(prim);
            const bool need_pair = !result.valid || !result.locked ||
                ((mt6826s_vernier_sample_counter_ % kAuxSampleDivider) == 0u);
            if (need_pair) {
                start_mt6826s_pair_sample();
            } else {
                ++mt6826s_vernier_sample_counter_;
                start_mt6826s_main_sample();
            }
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
    vernier_config.use_phase_difference = config_.vernier_use_phase_difference;
    return vernier_config;
}

void Encoder::apply_vernier_resolver_config() {
    vernier_resolver_.init(make_vernier_resolver_config());
    vernier_output_estimate_valid_ = false;
    vernier_main_continuous_valid_ = false;
    vernier_output_sample_dt_ = 0.0f;
    vernier_output_pair_sequence_ = 0;
}

float Encoder::vernier_motor_turns_per_output_turn() const {
    if (mode_ != MODE_SPI_ABS_MT6826S_VERNIER) {
        return 1.0f;
    }

    if (std::abs(config_.vernier_main_ratio) < 1.0e-6f) {
        return 1.0f;
    }

    return config_.vernier_main_ratio;
}

float Encoder::vernier_output_direction_sign() const {
    return config_.vernier_output_reversed ? -1.0f : 1.0f;
}

float Encoder::vernier_output_position_from_main(float main_position_turns) const {
    return vernier_output_direction_sign()
           * main_position_turns
           / vernier_motor_turns_per_output_turn();
}

float Encoder::vernier_output_velocity_from_main(float main_velocity_turns) const {
    return vernier_output_direction_sign()
           * main_velocity_turns
           / vernier_motor_turns_per_output_turn();
}

float Encoder::vernier_geometry_velocity_scale(float raw_position_turns) const {
    if (!config_.vernier_geometry_correction_enabled ||
        !std::isfinite(config_.vernier_effective_ratio_scale) ||
        std::abs(config_.vernier_effective_ratio_scale) < 1.0e-6f) {
        return 1.0f;
    }

    // Position is corrected as (raw + C(raw, direction)) / scale. Keep the
    // published velocity in the same coordinate by applying its derivative:
    // d(position)/dt = raw_velocity * (1 + dC/draw) / scale.
    const float phase = fmodf_pos(raw_position_turns, 1.0f);
    const float bin_position = phase * kVernierGeometryCorrectionBins;
    const size_t bin0 = static_cast<size_t>(bin_position) %
                        kVernierGeometryCorrectionBins;
    const size_t bin1 = (bin0 + 1) % kVernierGeometryCorrectionBins;
    const float common_slope =
        (config_.vernier_common_correction[bin1] -
         config_.vernier_common_correction[bin0]) *
        kVernierGeometryCorrectionBins;
    const float directional_slope =
        (config_.vernier_direction_correction[bin1] -
         config_.vernier_direction_correction[bin0]) *
        kVernierGeometryCorrectionBins;
    const float correction_derivative = common_slope +
        static_cast<float>(vernier_geometry_direction_) * directional_slope;
    if (!std::isfinite(correction_derivative)) {
        return 1.0f / config_.vernier_effective_ratio_scale;
    }

    // Old configurations may not have passed the derivative validation used
    // by the complete calibration flow. Never allow a malformed LUT to reverse
    // or explosively amplify the velocity estimate.
    const float local_position_slope = std::clamp(
        1.0f + correction_derivative, 0.25f, 4.0f);
    return local_position_slope / config_.vernier_effective_ratio_scale;
}

float Encoder::apply_vernier_geometry_compensation(float raw_position_turns,
                                                    float raw_velocity_turns) {
    (void)raw_velocity_turns;
    if (!config_.vernier_geometry_correction_enabled) {
        return raw_position_turns;
    }

    const float scale = config_.vernier_effective_ratio_scale;
    if (!std::isfinite(scale) || std::abs(scale) < 1.0e-6f) {
        return raw_position_turns;
    }

    const float correction =
        vernier_geometry_position_correction(raw_position_turns);
    const float corrected_observed = raw_position_turns + correction;
    return corrected_observed / scale;
}

float Encoder::vernier_geometry_position_correction(
        float raw_position_turns) const {
    if (!config_.vernier_geometry_correction_enabled) {
        return 0.0f;
    }

    const float phase = fmodf_pos(raw_position_turns, 1.0f);
    const float bin_position = phase * kVernierGeometryCorrectionBins;
    const size_t bin0 = static_cast<size_t>(bin_position) %
                        kVernierGeometryCorrectionBins;
    const size_t bin1 = (bin0 + 1) % kVernierGeometryCorrectionBins;
    const float fraction = bin_position - floorf(bin_position);
    const float common = config_.vernier_common_correction[bin0] +
        fraction * (config_.vernier_common_correction[bin1] -
                    config_.vernier_common_correction[bin0]);
    const float directional = config_.vernier_direction_correction[bin0] +
        fraction * (config_.vernier_direction_correction[bin1] -
                    config_.vernier_direction_correction[bin0]);
    return common +
        static_cast<float>(vernier_geometry_direction_) * directional;
}

void Encoder::initialize_vernier_circular_position(
        float raw_output_position) {
    const float scale =
        config_.vernier_geometry_correction_enabled &&
        std::isfinite(config_.vernier_effective_ratio_scale) &&
        std::abs(config_.vernier_effective_ratio_scale) >= 1.0e-6f
            ? config_.vernier_effective_ratio_scale : 1.0f;
    vernier_raw_output_phase_ =
        VernierResolver::wrap01(raw_output_position);
    vernier_last_geometry_correction_ =
        vernier_geometry_position_correction(vernier_raw_output_phase_);
    vernier_output_circular_pos_ = VernierResolver::wrap01(
        (raw_output_position + vernier_last_geometry_correction_) / scale);
    vernier_output_circular_valid_ = true;
}

void Encoder::update_vernier_circular_position(float raw_output_delta) {
    if (!vernier_output_circular_valid_) {
        return;
    }

    const float scale =
        config_.vernier_geometry_correction_enabled &&
        std::isfinite(config_.vernier_effective_ratio_scale) &&
        std::abs(config_.vernier_effective_ratio_scale) >= 1.0e-6f
            ? config_.vernier_effective_ratio_scale : 1.0f;
    vernier_raw_output_phase_ = VernierResolver::wrap01(
        vernier_raw_output_phase_ + raw_output_delta);
    const float correction =
        vernier_geometry_position_correction(vernier_raw_output_phase_);
    const float corrected_delta =
        (raw_output_delta + correction -
         vernier_last_geometry_correction_) / scale;
    vernier_output_circular_pos_ = VernierResolver::wrap01(
        vernier_output_circular_pos_ + corrected_delta);
    vernier_last_geometry_correction_ = correction;
}

float Encoder::vernier_main_position_from_output(float output_position_turns) const {
    return vernier_output_direction_sign()
           * output_position_turns
           * vernier_motor_turns_per_output_turn();
}

float Encoder::align_vernier_output_position(float position_turns, float reference_turns) const {
    return position_turns + roundf(reference_turns - position_turns);
}

float Encoder::normalized_main_phase_from_raw_phase(float raw_phase) const {
    float phase = VernierResolver::wrap01(raw_phase);
    if (config_.vernier_main_reversed) {
        phase = VernierResolver::wrap01(-phase);
    }
    return VernierResolver::wrap01(phase - config_.vernier_main_offset);
}

float Encoder::normalized_main_velocity_from_raw_velocity(float raw_velocity) const {
    return config_.vernier_main_reversed ? -raw_velocity : raw_velocity;
}

float Encoder::controller_to_motor_direction() const {
    float output_direction = 1.0f;
    if (mode_ == MODE_SPI_ABS_MT6826S_VERNIER) {
        if (config_.vernier_main_reversed) {
            output_direction = -output_direction;
        }
        if (config_.vernier_output_reversed) {
            output_direction = -output_direction;
        }
    }

    return (float)config_.direction * output_direction;
}

float Encoder::controller_torque_to_motor_torque_scale() const {
    if (mode_ != MODE_SPI_ABS_MT6826S_VERNIER) {
        return 1.0f;
    }

    const float ratio = std::abs(vernier_motor_turns_per_output_turn());
    return ratio > 1.0e-6f ? 1.0f / ratio : 1.0f;
}

bool Encoder::controller_feedback_ready() const {
    if (mode_ != MODE_SPI_ABS_MT6826S_VERNIER) {
        return is_ready_;
    }

    return is_ready_ &&
           vernier_output_estimate_valid_ &&
           vernier_main_continuous_valid_ &&
           vernier_result_.valid &&
           vernier_result_.locked;
}

void Encoder::reset_controller_velocity_filter() {
    controller_velocity_filter_x1_ = 0.0f;
    controller_velocity_filter_x2_ = 0.0f;
    controller_velocity_filter_y1_ = 0.0f;
    controller_velocity_filter_y2_ = 0.0f;
    controller_velocity_filter_initialized_ = false;
}

float Encoder::filter_controller_velocity(float raw_velocity) {
    // SguanFOC v3.0.1 encoder-speed filter: second-order Butterworth LPF,
    // wc=300 rad/s, bilinear transform, evaluated at the 10 kHz base rate.
    constexpr float wc = 300.0f;
    const float tw = current_meas_period * wc;
    const float tw2 = tw * tw;
    const float den0 = tw2 + 2.828427124746f * tw + 4.0f;
    const float b0 = tw2 / den0;
    const float b1 = 2.0f * b0;
    const float b2 = b0;
    const float a1 = (-8.0f + 2.0f * tw2) / den0;
    const float a2 = (tw2 - 2.828427124746f * tw + 4.0f) / den0;

    if (!controller_velocity_filter_initialized_) {
        controller_velocity_filter_x1_ = raw_velocity;
        controller_velocity_filter_x2_ = raw_velocity;
        controller_velocity_filter_y1_ = raw_velocity;
        controller_velocity_filter_y2_ = raw_velocity;
        controller_velocity_filter_initialized_ = true;
        return raw_velocity;
    }

    const float output = b0 * raw_velocity +
                         b1 * controller_velocity_filter_x1_ +
                         b2 * controller_velocity_filter_x2_ -
                         a1 * controller_velocity_filter_y1_ -
                         a2 * controller_velocity_filter_y2_;
    controller_velocity_filter_x2_ = controller_velocity_filter_x1_;
    controller_velocity_filter_x1_ = raw_velocity;
    controller_velocity_filter_y2_ = controller_velocity_filter_y1_;
    controller_velocity_filter_y1_ = std::isfinite(output) ? output : 0.0f;
    return controller_velocity_filter_y1_;
}

void Encoder::reset_vernier_output_velocity_estimate() {
    if (mode_ != MODE_SPI_ABS_MT6826S_VERNIER) {
        return;
    }

    reset_controller_velocity_filter();

    VernierResolver::Result result = {};
    uint32_t pair_sequence = 0;
    int64_t shadow_count = 0;
    uint32_t prim = cpu_enter_critical();
    result = vernier_result_;
    pair_sequence = mt6826s_pair_sequence_;
    shadow_count = shadow_count64_;
    cpu_exit_critical(prim);

    // Use the auxiliary encoder only to determine the initial main-encoder
    // unwrap branch. Runtime position follows exact main-encoder increments.
    if (result.valid) {
        vernier_main_continuous_pos_ = result.main_unwrapped;
        vernier_main_anchor_pos_ = result.main_unwrapped;
        vernier_anchor_shadow_count_ = shadow_count;
        vernier_anchor_valid_ = true;
        vernier_last_main_phase_corr_ = result.main_phase_corr;
        vernier_main_continuous_valid_ = true;
        vernier_last_shadow_count_ = shadow_count;
        vernier_last_shadow_count_valid_ = true;
        const float raw_output_position =
            vernier_output_position_from_main(vernier_main_continuous_pos_);
        initialize_vernier_circular_position(raw_output_position);
        vernier_output_pos_estimate_ =
            apply_vernier_geometry_compensation(raw_output_position, 0.0f);
        pos_estimate_ = vernier_output_pos_estimate_;
        pos_circular_ = vernier_output_circular_pos_;
        joint_pos_rad_ = vernier_output_pos_estimate_ * 2.0f * M_PI;
        vernier_output_estimate_valid_ = true;
    } else {
        vernier_output_estimate_valid_ = false;
        vernier_main_continuous_valid_ = false;
        vernier_anchor_valid_ = false;
        vernier_last_shadow_count_valid_ = false;
        vernier_output_circular_valid_ = false;
    }

    vernier_output_vel_estimate_ = 0.0f;
    vernier_pair_vel_estimate_ = 0.0f;
    vernier_last_aux_correction_ = 0.0f;
    vernier_pair_vel_estimate_valid_ = result.valid;
    vernier_last_pair_position_valid_ = result.valid;
    vernier_last_pair_position_ = vernier_output_pos_estimate_;
    vernier_output_sample_dt_ = 0.0f;
    vernier_output_pair_sequence_ = pair_sequence;
    vel_estimate_ = 0.0f;
}

void Encoder::publish_vernier_output_estimate(float dt, float motor_vel_estimate_turns) {
    VernierResolver::Result result = {};
    uint32_t pair_sequence = 0;
    int64_t shadow_count = 0;
    uint32_t prim = cpu_enter_critical();
    result = vernier_result_;
    pair_sequence = mt6826s_pair_sequence_;
    shadow_count = shadow_count64_;
    cpu_exit_critical(prim);

    if (dt <= 0.0f) {
        return;
    }

    vernier_output_sample_dt_ += dt;
    const float raw_main_pll_phase =
        VernierResolver::wrap01(pos_cpr_counts_ / (float)config_.cpr);
    const float main_phase_corr =
        normalized_main_phase_from_raw_phase(raw_main_pll_phase);

    if (!vernier_main_continuous_valid_) {
        if (!result.valid) {
            return;
        }
        // Initial absolute branch q is from the Vernier/Nonius pair. After this
        // point, the continuous controller coordinate follows main raw counts.
        vernier_main_continuous_pos_ = result.main_unwrapped;
        vernier_main_anchor_pos_ = result.main_unwrapped;
        vernier_anchor_shadow_count_ = shadow_count;
        vernier_anchor_valid_ = true;
        vernier_last_main_phase_corr_ = result.main_phase_corr;
        vernier_main_continuous_valid_ = true;
        vernier_last_shadow_count_ = shadow_count;
        vernier_last_shadow_count_valid_ = true;
        const float raw_output_position =
            vernier_output_position_from_main(vernier_main_continuous_pos_);
        initialize_vernier_circular_position(raw_output_position);
        vernier_output_pos_estimate_ =
            apply_vernier_geometry_compensation(raw_output_position, 0.0f);
        vernier_output_vel_estimate_ = 0.0f;
        vernier_output_estimate_valid_ = true;
        vernier_output_pair_sequence_ = pair_sequence;
        vernier_output_sample_dt_ = 0.0f;
    } else {
        // Position is advanced only by exact integer encoder motion. The PLL
        // phase is intentionally not integrated into the controller position:
        // its sub-count settling can move while the raw encoder is stationary,
        // which a high-gain position loop would turn into a real torque ripple.
        int64_t delta_shadow = 0;
        if (vernier_last_shadow_count_valid_) {
            delta_shadow = shadow_count - vernier_last_shadow_count_;
        }
        vernier_last_shadow_count_ = shadow_count;
        vernier_last_shadow_count_valid_ = true;

        const float main_direction =
            config_.vernier_main_reversed ? -1.0f : 1.0f;
        const float delta_main =
            main_direction * (float)delta_shadow / (float)config_.cpr;

        // Reconstruct from an integer displacement relative to a fixed branch
        // anchor. Do not repeatedly add one encoder count to a large float:
        // around 2^9 motor turns a float32 ULP is already two 32768-CPR counts,
        // so repeated one-count additions can round away forever.
        if (!vernier_anchor_valid_) {
            vernier_main_anchor_pos_ = vernier_main_continuous_pos_;
            vernier_anchor_shadow_count_ = shadow_count;
            vernier_anchor_valid_ = true;
        }
        const int64_t anchor_delta_counts =
            shadow_count - vernier_anchor_shadow_count_;
        vernier_main_continuous_pos_ =
            vernier_main_anchor_pos_ +
            main_direction * (float)anchor_delta_counts / (float)config_.cpr;
        vernier_last_main_phase_corr_ = main_phase_corr;

        // Direction-dependent calibration is a hysteresis branch. Change it
        // only after a real encoder count, never from a PLL velocity transient.
        // delta_main is a motor-shaft displacement. Convert it to the output-
        // shaft coordinate before advancing the circular joint position.
        // Omitting the gear-ratio division makes pos_circular_ move roughly
        // vernier_main_ratio times faster than the matching velocity estimate,
        // which drives the position cascade into a violent reversal.
        const float raw_output_delta =
            vernier_output_velocity_from_main(delta_main);
        if (raw_output_delta > 0.0f) {
            vernier_geometry_direction_ = 1;
        } else if (raw_output_delta < 0.0f) {
            vernier_geometry_direction_ = -1;
        }
        const float raw_output_position =
            vernier_output_position_from_main(vernier_main_continuous_pos_);
        if (vernier_output_circular_valid_) {
            update_vernier_circular_position(raw_output_delta);
        } else {
            // Recovery uses the resolved absolute branch. A delta alone
            // cannot establish the correct [0, 1) joint phase.
            initialize_vernier_circular_position(raw_output_position);
        }
        const float raw_output_velocity =
            vernier_output_velocity_from_main(
                normalized_main_velocity_from_raw_velocity(
                    motor_vel_estimate_turns));
        vernier_output_pos_estimate_ = apply_vernier_geometry_compensation(
            raw_output_position, raw_output_velocity);
        vernier_output_estimate_valid_ = true;
    }

    vernier_output_vel_estimate_ =
        vernier_output_velocity_from_main(
            normalized_main_velocity_from_raw_velocity(motor_vel_estimate_turns))
        * vernier_geometry_velocity_scale(vernier_raw_output_phase_);
    vernier_last_aux_correction_ = 0.0f;

    if (pair_sequence != vernier_output_pair_sequence_) {
        vernier_output_pair_sequence_ = pair_sequence;
        vernier_output_sample_dt_ = 0.0f;
    }

    pos_estimate_ = vernier_output_pos_estimate_;
    // In vernier mode the controller coordinate is the output shaft, so publish
    // the output-shaft velocity together with the output-shaft position.  The
    // motor-side PLL velocity remains used below for FOC phase velocity.
    vel_estimate_ = vernier_output_vel_estimate_;
    pos_circular_ = vernier_output_circular_pos_;
    joint_pos_rad_ = vernier_output_pos_estimate_ * 2.0f * M_PI;
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
    out->main_sample_age_cycles = 0;
    out->max_main_sample_age_cycles = 0;
    out->pll_phase_error_counts = delta_pos_cpr_counts_;
    out->pll_velocity_counts_per_s = vel_estimate_counts_;
    out->pll_position_counts = pos_estimate_counts_;
    out->controller_motor_velocity_turns_per_s =
        controller_velocity_filter_initialized_
            ? controller_velocity_filter_y1_ : 0.0f;
    out->shadow_count = shadow_count_;
    out->count_in_cpr = count_in_cpr_;
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
    out->resolver_valid = resolver_result.valid;
    out->resolver_locked = resolver_result.locked;
    out->resolver_accepted_aux = resolver_result.accepted_aux;
    out->resolver_degraded = resolver_result.degraded;
    out->output_estimate_valid = vernier_output_estimate_valid_;
    out->output_pos_estimate = vernier_output_pos_estimate_;
    out->output_vel_estimate = vernier_output_vel_estimate_;
    out->output_sample_dt = vernier_output_sample_dt_;
    out->output_pair_sequence = vernier_output_pair_sequence_;
    out->output_pair_vel_estimate = vernier_pair_vel_estimate_;
    out->output_last_aux_correction = vernier_last_aux_correction_;
    out->pair_transaction_cycles = 0;
    out->max_pair_transaction_cycles = 0;
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

static float centered_vernier_offset(float value) {
    value = VernierResolver::wrap01(value);
    return value >= 0.5f ? value - 1.0f : value;
}

static float circular_vernier_distance(float a, float b) {
    return std::abs(VernierResolver::wrap_pm_half(a - b));
}

void Encoder::reset_vernier_calibration() {
    vernier_calibration_point_count_ = 0;
    vernier_calibration_fit_valid_ = false;
    vernier_calibration_fitted_main_offset_ = config_.vernier_main_offset;
    vernier_calibration_fitted_aux_offset_ = config_.vernier_aux_offset;
    vernier_calibration_fit_score_ = 0.0f;
    vernier_calibration_worst_residual_ = 0.0f;
}

bool Encoder::capture_vernier_calibration_point() {
    if (mode_ != MODE_SPI_ABS_MT6826S_VERNIER ||
        vernier_calibration_point_count_ >= kVernierCalibrationMaxPoints) {
        return false;
    }

    Mt6826sSpi::Sample main_sample = {};
    Mt6826sSpi::Sample aux_sample = {};
    bool pair_valid = false;
    uint32_t prim = cpu_enter_critical();
    main_sample = mt6826s_main_sample_;
    aux_sample = mt6826s_aux_sample_;
    pair_valid = mt6826s_pair_valid_;
    cpu_exit_critical(prim);

    if (!pair_valid || !main_sample.valid || !aux_sample.valid) {
        return false;
    }

    VernierCalibrationPoint& point =
        vernier_calibration_points_[vernier_calibration_point_count_++];
    point.main_angle = main_sample.angle;
    point.aux_angle = aux_sample.angle;
    vernier_calibration_fit_valid_ = false;
    return true;
}

static float normalize_vernier_calib_angle(uint16_t angle, bool reversed, uint16_t cpr) {
    float phase = static_cast<float>(angle % cpr) / static_cast<float>(cpr);
    if (reversed) {
        phase = VernierResolver::wrap01(-phase);
    }
    return phase;
}

static float vernier_calibration_residual(const Encoder::VernierCalibrationPoint& point,
                                          const Encoder::Config_t& config,
                                          float main_offset,
                                          float aux_offset) {
    const float main_phase =
        normalize_vernier_calib_angle(point.main_angle,
                                      config.vernier_main_reversed,
                                      static_cast<uint16_t>(config.cpr));
    const float aux_phase =
        normalize_vernier_calib_angle(point.aux_angle,
                                      config.vernier_aux_reversed,
                                      static_cast<uint16_t>(config.cpr));
    const float main_phase_corr = VernierResolver::wrap01(main_phase - main_offset);
    const float aux_phase_corr = VernierResolver::wrap01(aux_phase - aux_offset);
    const float ratio_delta = config.vernier_aux_ratio - config.vernier_main_ratio;
    const float coarse_output_phase =
        VernierResolver::wrap01(
            VernierResolver::wrap_pm_half(aux_phase_corr - main_phase_corr)
            / ratio_delta);
    const float predicted_main_phase =
        VernierResolver::wrap01(config.vernier_main_ratio * coarse_output_phase);
    return VernierResolver::wrap_pm_half(main_phase_corr - predicted_main_phase);
}

static float vernier_calibration_objective(const Encoder::VernierCalibrationPoint* points,
                                           uint32_t point_count,
                                           const Encoder::Config_t& config,
                                           float main_offset,
                                           float aux_offset,
                                           float* worst_residual) {
    float sum_sq = 0.0f;
    float worst = 0.0f;
    for (uint32_t i = 0; i < point_count; ++i) {
        const float residual =
            vernier_calibration_residual(points[i], config, main_offset, aux_offset);
        sum_sq += residual * residual;
        worst = std::max(worst, std::abs(residual));
    }
    if (worst_residual) {
        *worst_residual = worst;
    }
    return sqrtf(sum_sq / static_cast<float>(point_count));
}

static float vernier_calibration_phase_span(const Encoder::VernierCalibrationPoint* points,
                                            uint32_t point_count,
                                            const Encoder::Config_t& config) {
    float span = 0.0f;
    for (uint32_t i = 0; i < point_count; ++i) {
        const float main_i =
            normalize_vernier_calib_angle(points[i].main_angle,
                                          config.vernier_main_reversed,
                                          static_cast<uint16_t>(config.cpr));
        const float aux_i =
            normalize_vernier_calib_angle(points[i].aux_angle,
                                          config.vernier_aux_reversed,
                                          static_cast<uint16_t>(config.cpr));
        for (uint32_t j = i + 1; j < point_count; ++j) {
            const float main_j =
                normalize_vernier_calib_angle(points[j].main_angle,
                                              config.vernier_main_reversed,
                                              static_cast<uint16_t>(config.cpr));
            const float aux_j =
                normalize_vernier_calib_angle(points[j].aux_angle,
                                              config.vernier_aux_reversed,
                                              static_cast<uint16_t>(config.cpr));
            span = std::max(span, std::abs(VernierResolver::wrap_pm_half(main_j - main_i)));
            span = std::max(span, std::abs(VernierResolver::wrap_pm_half(aux_j - aux_i)));
        }
    }
    return span;
}

static bool vernier_calibration_score_is_better(float score,
                                                float distance,
                                                float best_score,
                                                float best_distance,
                                                bool has_best) {
    if (!has_best) {
        return true;
    }
    if (score < best_score - 1.0e-9f) {
        return true;
    }
    return std::abs(score - best_score) <= 1.0e-9f && distance < best_distance;
}

bool Encoder::fit_vernier_aux_offset(float search_radius) {
    if (mode_ != MODE_SPI_ABS_MT6826S_VERNIER ||
        vernier_calibration_point_count_ < 2 ||
        std::abs(config_.vernier_aux_ratio - config_.vernier_main_ratio) < 1.0e-6f) {
        return false;
    }

    if (!std::isfinite(search_radius) || search_radius <= 0.0f) {
        search_radius = 0.05f;
    }
    search_radius = std::min(std::max(search_radius, 0.001f), 0.5f);

    const float min_phase_span =
        std::max(8.0f / static_cast<float>(config_.cpr), 1.0e-5f);
    if (vernier_calibration_phase_span(vernier_calibration_points_,
                                       vernier_calibration_point_count_,
                                       config_) < min_phase_span) {
        return false;
    }

    const float main_offset = config_.vernier_main_offset;
    const float aux_center = config_.vernier_aux_offset;
    constexpr uint32_t kGridSteps = 1024;

    bool has_best = false;
    float best_aux = aux_center;
    float best_score = 0.0f;
    float best_distance = 0.0f;
    float best_worst = 0.0f;

    for (uint32_t i = 0; i < kGridSteps; ++i) {
        const float t = (kGridSteps <= 1) ? 0.0f
            : static_cast<float>(i) / static_cast<float>(kGridSteps - 1);
        const float aux_offset =
            VernierResolver::wrap01(aux_center - search_radius + 2.0f * search_radius * t);
        float worst = 0.0f;
        const float score =
            vernier_calibration_objective(vernier_calibration_points_,
                                          vernier_calibration_point_count_,
                                          config_,
                                          main_offset,
                                          aux_offset,
                                          &worst);
        const float distance = circular_vernier_distance(aux_offset, aux_center);
        if (vernier_calibration_score_is_better(score, distance,
                                                best_score, best_distance,
                                                has_best)) {
            best_aux = aux_offset;
            best_score = score;
            best_distance = distance;
            best_worst = worst;
            has_best = true;
        }
    }

    float step = 2.0f * search_radius / static_cast<float>(kGridSteps - 1);
    for (uint32_t iter = 0; iter < 10; ++iter) {
        bool improved = false;
        for (int dir_i = 0; dir_i < 2; ++dir_i) {
            const float dir = dir_i == 0 ? -1.0f : 1.0f;
            const float candidate = VernierResolver::wrap01(best_aux + dir * step);
            const float distance = circular_vernier_distance(candidate, aux_center);
            if (distance > search_radius + 1.0e-6f) {
                continue;
            }
            float worst = 0.0f;
            const float score =
                vernier_calibration_objective(vernier_calibration_points_,
                                              vernier_calibration_point_count_,
                                              config_,
                                              main_offset,
                                              candidate,
                                              &worst);
            if (vernier_calibration_score_is_better(score, distance,
                                                    best_score, best_distance,
                                                    true)) {
                best_aux = candidate;
                best_score = score;
                best_distance = distance;
                best_worst = worst;
                improved = true;
            }
        }
        if (!improved) {
            step *= 0.25f;
        }
    }

    vernier_calibration_fitted_main_offset_ = centered_vernier_offset(main_offset);
    vernier_calibration_fitted_aux_offset_ = centered_vernier_offset(best_aux);
    vernier_calibration_fit_score_ = best_score;
    vernier_calibration_worst_residual_ = best_worst;
    vernier_calibration_fit_valid_ = true;
    return true;
}

bool Encoder::apply_vernier_calibration_fit() {
    if (!vernier_calibration_fit_valid_) {
        return false;
    }

    config_.set_vernier_aux_offset(vernier_calibration_fitted_aux_offset_);
    vernier_calibration_fit_valid_ = false;
    return true;
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

    shadow_count64_ += (int64_t)delta_enc;
    // Keep the legacy int32 telemetry count with explicitly defined wrapping.
    // Avoid signed-overflow UB; realtime control uses shadow_count64_.
    int64_t wrapped_shadow = (int64_t)shadow_count_ + (int64_t)delta_enc;
    if (wrapped_shadow > INT32_MAX) {
        wrapped_shadow -= (INT64_C(1) << 32);
    } else if (wrapped_shadow < INT32_MIN) {
        wrapped_shadow += (INT64_C(1) << 32);
    }
    shadow_count_ = (int32_t)wrapped_shadow;
    count_in_cpr_ += delta_enc;
    count_in_cpr_ = mod(count_in_cpr_, config_.cpr);

    count_in_cpr_ = pos_abs_latched;

    // Memory for pos_circular
    float pos_cpr_counts_last = pos_cpr_counts_;

    //// Tustin PLL in encoder counts, matching SguanFOC's discretization.
    float delta_pos_cpr_counts = (float)count_in_cpr_ - pos_cpr_counts_;
    delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float)(config_.cpr));
    delta_pos_cpr_counts_ += 0.1f * (delta_pos_cpr_counts - delta_pos_cpr_counts_); // for debug
    const float previous_error = pll_previous_error_valid_
        ? pll_previous_error_counts_ : delta_pos_cpr_counts;
    const float old_velocity = vel_estimate_counts_;
    vel_estimate_counts_ +=
        (pll_kp_ + 0.5f * pll_ki_ * current_meas_period) * delta_pos_cpr_counts +
        (-pll_kp_ + 0.5f * pll_ki_ * current_meas_period) * previous_error;
    pll_previous_error_counts_ = delta_pos_cpr_counts;
    pll_previous_error_valid_ = true;
    pos_estimate_counts_ += 0.5f * current_meas_period *
                            (old_velocity + vel_estimate_counts_);
    // The PLL estimates phase, not the multi-turn branch. Bounding the state
    // preserves sub-count float32 resolution regardless of how far the shaft
    // has travelled. shadow_count_ remains the authoritative branch counter.
    pos_estimate_counts_ =
        fmodf_pos(pos_estimate_counts_, (float)config_.cpr);
    pos_cpr_counts_ = pos_estimate_counts_;
    bool snap_to_zero_vel = false;
    if (std::abs(vel_estimate_counts_) < 0.5f * current_meas_period * pll_ki_) {
        vel_estimate_counts_ = 0.0f;  //align delta-sigma on zero to prevent jitter
        snap_to_zero_vel = true;
    }

    // Reconstruct the continuous estimate from an integer branch and the
    // bounded PLL phase correction. Split quotient/remainder before converting
    // to float so a large shadow count cannot quantize the circular phase.
    const int64_t whole_motor_turns = shadow_count64_ / config_.cpr;
    const int32_t shadow_remainder = (int32_t)(
        shadow_count64_ - whole_motor_turns * config_.cpr);
    const float pll_phase_correction = wrap_pm(
        pos_cpr_counts_ - (float)count_in_cpr_, (float)config_.cpr);
    const float motor_pos_estimate_turns =
        (float)whole_motor_turns +
        ((float)shadow_remainder + pll_phase_correction) /
            (float)config_.cpr;
    const float motor_vel_estimate_turns = vel_estimate_counts_ / (float)config_.cpr;
    const float controller_motor_velocity_turns =
        filter_controller_velocity(motor_vel_estimate_turns);

    // For non-vernier modes, controller position and velocity are sourced from
    // the motor-side PLL. In vernier mode, publish_vernier_output_estimate()
    // overrides both pos_estimate_ and vel_estimate_ with output-shaft values.
    if (mode_ != MODE_SPI_ABS_MT6826S_VERNIER) {
        pos_estimate_ = motor_pos_estimate_turns;
        vel_estimate_ = controller_motor_velocity_turns;
    }
    
    // TODO: we should strictly require that this value is from the previous iteration
    // to avoid spinout scenarios. However that requires a proper way to reset
    // the encoder from error states.
    float pos_circular = pos_circular_.any().value_or(0.0f);
    pos_circular +=  wrap_pm((pos_cpr_counts_ - pos_cpr_counts_last) / (float)config_.cpr, 1.0f);
    pos_circular = fmodf_pos(pos_circular, axis_->controller_.config_.circular_setpoint_range);
    if (mode_ != MODE_SPI_ABS_MT6826S_VERNIER) {
        pos_circular_ = pos_circular;
        joint_pos_rad_ = motor_pos_estimate_turns * 2.0f * M_PI;
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
        const float electrical_velocity = (2*M_PI) * motor_vel_estimate_turns *
            axis_->motor_.config_.pole_pairs * config_.direction;
        phase_ = wrap_pm_pi(wrap_pm_pi(ph) * config_.direction +
                            electrical_velocity * config_.electrical_phase_delay);
        phase_vel_ = electrical_velocity;
    }

    if (mode_ == MODE_SPI_ABS_MT6826S_VERNIER) {
        publish_vernier_output_estimate(current_meas_period,
                                        controller_motor_velocity_turns);
    }

    return true;
}
