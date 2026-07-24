#ifndef __ENCODER_HPP
#define __ENCODER_HPP

class Encoder;

#include <board.h> // needed for arm_math.h
#include <Drivers/STM32/stm32_spi_arbiter.hpp>
#include <Drivers/MT6826S/mt6826s_spi.hpp>
#include <Drivers/MT6826S/vernier_resolver.hpp>
#include "utils.hpp"
#include <autogen/interfaces.hpp>
#include "component.hpp"


class Encoder : public ODriveIntf::EncoderIntf {
public:
    static constexpr size_t kVernierGeometryCorrectionBins = 64;

    struct Config_t {
        Mode mode = MODE_SPI_ABS_MT6826S_VERNIER;
        float calib_range = 0.02f; // Accuracy required to pass encoder cpr check
        float calib_scan_distance = 16.0f * M_PI; // rad electrical
        float calib_scan_omega = 4.0f * M_PI; // rad/s electrical
        // PLL natural frequency [rad/s]. The default reproduces SguanFOC's
        // Kp=650, Ki=210000 design with the damping used in update_pll_gains().
        float bandwidth = 458.25757f;
        int32_t phase_offset = 0;        // Offset between encoder count and rotor electrical phase
        float phase_offset_float = 0.0f; // Sub-count phase alignment offset
        int32_t cpr = ODRIVE_PRODUCTION_ENCODER_CPR;
        bool pre_calibrated = false; // If true, this means the offset stored in
                                    // configuration is valid and does not need
                                    // be determined by run_offset_calibration.
        int32_t direction = 0; // direction with respect to motor
        bool enable_phase_interpolation = true; // Use velocity to interpolate inside the count state
        uint16_t abs_spi_cs_gpio_pin = ODRIVE_PRODUCTION_ABS_SPI_CS_GPIO_PIN;
        uint16_t abs_spi_aux_cs_gpio_pin = ODRIVE_PRODUCTION_ABS_SPI_AUX_CS_GPIO_PIN;
        float vernier_main_ratio = ODRIVE_PRODUCTION_VERNIER_MAIN_RATIO;
        float vernier_aux_ratio = ODRIVE_PRODUCTION_VERNIER_AUX_RATIO;
        float vernier_main_offset = 0.0f;
        float vernier_aux_offset = 0.0f;
        bool vernier_main_reversed = ODRIVE_PRODUCTION_VERNIER_MAIN_REVERSED;
        bool vernier_aux_reversed = ODRIVE_PRODUCTION_VERNIER_AUX_REVERSED;
        bool vernier_use_phase_difference = ODRIVE_PRODUCTION_VERNIER_USE_PHASE_DIFFERENCE;
        bool vernier_output_reversed = ODRIVE_PRODUCTION_VERNIER_OUTPUT_REVERSED;
        int32_t vernier_virtual_cpr = ODRIVE_PRODUCTION_VERNIER_VIRTUAL_CPR;
        float vernier_err_accept = ODRIVE_PRODUCTION_VERNIER_ERR_ACCEPT;
        float vernier_err_reject = ODRIVE_PRODUCTION_VERNIER_ERR_REJECT;
        uint16_t mt6826s_spi_mode = ODRIVE_PRODUCTION_MT6826S_SPI_MODE;
        uint16_t mt6826s_spi_prescaler = ODRIVE_PRODUCTION_MT6826S_SPI_PRESCALER;
        float vernier_aux_correction_bandwidth = 0.0f;
        float vernier_aux_velocity_bandwidth = 0.0f;
        float vernier_aux_max_correction = 0.002f;
        bool vernier_geometry_correction_enabled = false;
        float vernier_effective_ratio_scale = 1.0f;
        float vernier_common_correction[kVernierGeometryCorrectionBins] = {};
        float vernier_direction_correction[kVernierGeometryCorrectionBins] = {};
        float electrical_phase_delay = 0.0f; // [s], phase advance


        // custom setters
        Encoder* parent = nullptr;
        void set_abs_spi_cs_gpio_pin(uint16_t value) { abs_spi_cs_gpio_pin = value; parent->abs_spi_cs_pin_init(); }
        void set_abs_spi_aux_cs_gpio_pin(uint16_t value) { abs_spi_aux_cs_gpio_pin = value; parent->abs_spi_aux_cs_pin_init(); }
        void set_vernier_main_ratio(float value) { vernier_main_ratio = value; parent->apply_vernier_resolver_config(); }
        void set_vernier_aux_ratio(float value) { vernier_aux_ratio = value; parent->apply_vernier_resolver_config(); }
        void set_vernier_main_offset(float value) { vernier_main_offset = value; parent->apply_vernier_resolver_config(); }
        void set_vernier_aux_offset(float value) { vernier_aux_offset = value; parent->apply_vernier_resolver_config(); }
        void set_vernier_main_reversed(bool value) { vernier_main_reversed = value; parent->apply_vernier_resolver_config(); }
        void set_vernier_aux_reversed(bool value) { vernier_aux_reversed = value; parent->apply_vernier_resolver_config(); }
        void set_vernier_use_phase_difference(bool value) { vernier_use_phase_difference = value; parent->apply_vernier_resolver_config(); }
        void set_vernier_output_reversed(bool value) { vernier_output_reversed = value; parent->apply_vernier_resolver_config(); }
        void set_vernier_err_accept(float value) { vernier_err_accept = value; parent->apply_vernier_resolver_config(); }
        void set_vernier_err_reject(float value) { vernier_err_reject = value; parent->apply_vernier_resolver_config(); }
        void set_vernier_aux_correction_bandwidth(float value) { vernier_aux_correction_bandwidth = value; parent->reset_vernier_output_velocity_estimate(); }
        void set_vernier_aux_velocity_bandwidth(float value) { vernier_aux_velocity_bandwidth = value; parent->reset_vernier_output_velocity_estimate(); }
        void set_vernier_aux_max_correction(float value) { vernier_aux_max_correction = value; parent->reset_vernier_output_velocity_estimate(); }
        void set_vernier_geometry_correction_enabled(bool value) { vernier_geometry_correction_enabled = value; parent->reset_vernier_output_velocity_estimate(); }
        void set_vernier_effective_ratio_scale(float value) { vernier_effective_ratio_scale = value; parent->reset_vernier_output_velocity_estimate(); }
        void set_electrical_phase_delay(float value) { electrical_phase_delay = value; }
        void set_mt6826s_spi_mode(uint16_t value) { mt6826s_spi_mode = value; parent->apply_mt6826s_spi_config(); }
        void set_mt6826s_spi_prescaler(uint16_t value) { mt6826s_spi_prescaler = value; parent->apply_mt6826s_spi_config(); }
        void set_pre_calibrated(bool value) { pre_calibrated = value; parent->check_pre_calibrated(); }
        void set_bandwidth(float value) { bandwidth = value; parent->update_pll_gains(); }
    };

    Encoder(Stm32SpiArbiter* spi_arbiter);
    
    bool apply_config();
    void setup();
    void set_error(Error error);
    bool do_checks();

    void update_pll_gains();
    void check_pre_calibrated();

    void set_linear_count(int32_t count);
    void set_circular_count(int32_t count, bool update_offset);
    bool calib_enc_offset(float voltage_magnitude);

    bool run_offset_calibration();
    void sample_now();
    bool update();

    Stm32SpiArbiter* spi_arbiter_;
    Axis* axis_ = nullptr; // set by Axis constructor

    Config_t config_;

    Error error_ = ERROR_NONE;
    bool is_ready_ = false;
    int32_t shadow_count_ = 0;
    int64_t shadow_count64_ = 0;
    int32_t count_in_cpr_ = 0;
    float interpolation_ = 0.0f;
    OutputPort<float> phase_ = 0.0f;     // [rad]
    OutputPort<float> phase_vel_ = 0.0f; // [rad/s]
    // PLL phase state in [0, cpr). Keeping this bounded is essential: a
    // continuous float32 count loses single-count resolution above 2^24 and
    // creates a false stationary velocity limit cycle. shadow_count_ carries
    // the multi-turn branch.
    float pos_estimate_counts_ = 0.0f;  // [count], modulo CPR
    float pos_cpr_counts_ = 0.0f;  // [count], published PLL phase
    float delta_pos_cpr_counts_ = 0.0f;  // [count] phase detector result for debug
    float vel_estimate_counts_ = 0.0f;  // [count/s]
    float pll_kp_ = 0.0f;   // [count/s / count]
    float pll_ki_ = 0.0f;   // [(count/s^2) / count]
    float pll_previous_error_counts_ = 0.0f;
    bool pll_previous_error_valid_ = false;
    float controller_velocity_filter_x1_ = 0.0f;
    float controller_velocity_filter_x2_ = 0.0f;
    float controller_velocity_filter_y1_ = 0.0f;
    float controller_velocity_filter_y2_ = 0.0f;
    bool controller_velocity_filter_initialized_ = false;
    int32_t pos_abs_ = 0;
    float spi_error_rate_ = 0.0f;

    OutputPort<float> pos_estimate_ = 0.0f; // [turn]
    OutputPort<float> vel_estimate_ = 0.0f; // [turn/s]
    OutputPort<float> pos_circular_ = 0.0f; // [turn]
    OutputPort<float> joint_pos_rad_ = 0.0f; // [rad], linear mechanical coordinate

    bool pos_estimate_valid_ = false;
    bool vel_estimate_valid_ = false;

    static void mt6826s_spi_cb(void* ctx, const Mt6826sSpi::Sample& sample, bool success);
    void handle_mt6826s_spi_cb(const Mt6826sSpi::Sample& sample, bool success);
    static void mt6826s_spi_pair_cb(void* ctx, const Mt6826sSpiPair::PairSample& sample, bool success);
    void handle_mt6826s_spi_pair_cb(const Mt6826sSpiPair::PairSample& sample, bool success);
    void abs_spi_cs_pin_init();
    void abs_spi_aux_cs_pin_init();
    Mt6826sSpi::Config make_mt6826s_spi_config() const;
    void apply_mt6826s_spi_config();
    VernierResolver::Config make_vernier_resolver_config() const;
    void apply_vernier_resolver_config();
    float controller_to_motor_direction() const;
    float controller_torque_to_motor_torque_scale() const;
    bool controller_feedback_ready() const;
    float vernier_motor_turns_per_output_turn() const;
    float vernier_output_direction_sign() const;
    float vernier_output_position_from_main(float main_position_turns) const;
    float vernier_output_velocity_from_main(float main_velocity_turns) const;
    float apply_vernier_geometry_compensation(float raw_position_turns,
                                              float raw_velocity_turns);
    float vernier_geometry_position_correction(float raw_position_turns) const;
    float vernier_geometry_velocity_scale(float raw_position_turns) const;
    void initialize_vernier_circular_position(float raw_output_position);
    void update_vernier_circular_position(float raw_output_delta);
    float vernier_main_position_from_output(float output_position_turns) const;
    float align_vernier_output_position(float position_turns, float reference_turns) const;
    float normalized_main_phase_from_raw_phase(float raw_phase) const;
    float normalized_main_velocity_from_raw_velocity(float raw_velocity) const;
    void publish_vernier_output_estimate(float dt, float motor_vel_estimate_turns);
    void reset_vernier_output_velocity_estimate();
    float filter_controller_velocity(float raw_velocity);
    void reset_controller_velocity_filter();
    bool start_mt6826s_main_sample();
    bool start_mt6826s_pair_sample();
    bool abs_spi_pos_updated_ = false;
    Mode mode_ = MODE_SPI_ABS_MT6826S_VERNIER;
    Stm32Gpio abs_spi_cs_gpio_;
    Stm32Gpio abs_spi_aux_cs_gpio_;
    Mt6826sSpi mt6826s_spi_;
    Mt6826sSpi mt6826s_aux_spi_;
    Mt6826sSpiPair mt6826s_spi_pair_;
    VernierResolver vernier_resolver_;

    struct VernierDiagnosticsSnapshot {
        Mt6826sSpi::Sample main_sample = {};
        Mt6826sSpi::Sample aux_sample = {};
        uint32_t pair_count = 0;
        bool pair_valid = false;
        int32_t virtual_count = 0;
        float position_turns = 0.0f;
        float residual = 0.0f;
        float encoder_pos_estimate = 0.0f;
        float encoder_vel_estimate = 0.0f;
        float encoder_pos_circular = 0.0f;
        uint32_t state = 0;
        bool resolver_valid = false;
        bool resolver_locked = false;
        bool resolver_accepted_aux = false;
        bool resolver_degraded = false;
        bool output_estimate_valid = false;
        float output_pos_estimate = 0.0f;
        float output_vel_estimate = 0.0f;
        float output_sample_dt = 0.0f;
        uint32_t output_pair_sequence = 0;
        float output_pair_vel_estimate = 0.0f;
        float output_last_aux_correction = 0.0f;
        uint32_t main_error_count = 0;
        uint32_t aux_error_count = 0;
        uint32_t pair_error_count = 0;
        uint32_t main_spi_dma_error_count = 0;
        uint32_t main_crc_error_count = 0;
        uint32_t main_fixed_bit_error_count = 0;
        uint32_t main_status_warning_count = 0;
        uint32_t main_sample_count = 0;
        uint32_t aux_spi_dma_error_count = 0;
        uint32_t aux_crc_error_count = 0;
        uint32_t aux_fixed_bit_error_count = 0;
        uint32_t aux_status_warning_count = 0;
        uint32_t aux_sample_count = 0;
        uint32_t pair_transaction_cycles = 0;
        uint32_t max_pair_transaction_cycles = 0;
        uint32_t main_sample_age_cycles = 0;
        uint32_t max_main_sample_age_cycles = 0;
        float pll_phase_error_counts = 0.0f;
        float pll_velocity_counts_per_s = 0.0f;
        float pll_position_counts = 0.0f;
        float controller_motor_velocity_turns_per_s = 0.0f;
        int32_t shadow_count = 0;
        int32_t count_in_cpr = 0;
    };
    void get_vernier_diagnostics_snapshot(VernierDiagnosticsSnapshot* out);
    void reset_vernier_calibration();
    bool capture_vernier_calibration_point();
    bool fit_vernier_aux_offset(float search_radius);
    bool apply_vernier_calibration_fit();
    uint32_t get_vernier_calibration_point_count() const { return vernier_calibration_point_count_; }
    bool get_vernier_calibration_fit_valid() const { return vernier_calibration_fit_valid_; }
    float get_vernier_calibration_fitted_main_offset() const { return vernier_calibration_fitted_main_offset_; }
    float get_vernier_calibration_fitted_aux_offset() const { return vernier_calibration_fitted_aux_offset_; }
    float get_vernier_calibration_fit_score() const { return vernier_calibration_fit_score_; }
    float get_vernier_calibration_worst_residual() const { return vernier_calibration_worst_residual_; }
    int32_t get_calibration_estimated_pole_pairs() const {
        return calibration_estimated_pole_pairs_;
    }

    Mt6826sSpi::Sample mt6826s_main_sample_;
    Mt6826sSpi::Sample mt6826s_aux_sample_;
    VernierResolver::Result vernier_result_;
    float vernier_output_pos_estimate_ = 0.0f;
    float vernier_output_vel_estimate_ = 0.0f;
    float vernier_raw_output_phase_ = 0.0f;
    float vernier_output_circular_pos_ = 0.0f;
    float vernier_last_geometry_correction_ = 0.0f;
    bool vernier_output_circular_valid_ = false;
    float vernier_output_sample_dt_ = 0.0f;
    uint32_t vernier_output_pair_sequence_ = 0;
    bool vernier_output_estimate_valid_ = false;
    float vernier_pair_vel_estimate_ = 0.0f;
    float vernier_last_pair_position_ = 0.0f;
    float vernier_last_aux_correction_ = 0.0f;
    bool vernier_pair_vel_estimate_valid_ = false;
    bool vernier_last_pair_position_valid_ = false;
    float vernier_main_continuous_pos_ = 0.0f;
    float vernier_main_anchor_pos_ = 0.0f;
    int64_t vernier_anchor_shadow_count_ = 0;
    bool vernier_anchor_valid_ = false;
    float vernier_last_main_phase_corr_ = 0.0f;
    bool vernier_main_continuous_valid_ = false;
    int64_t vernier_last_shadow_count_ = 0;
    bool vernier_last_shadow_count_valid_ = false;
    int8_t vernier_geometry_direction_ = 0;
    uint32_t mt6826s_pair_sequence_ = 0;
    uint32_t mt6826s_vernier_sample_counter_ = 0;
    bool mt6826s_pair_valid_ = false;
    int32_t calibration_estimated_pole_pairs_ = 0;

    struct VernierCalibrationPoint {
        uint16_t main_angle = 0;
        uint16_t aux_angle = 0;
    };
    static constexpr uint32_t kVernierCalibrationMaxPoints = 16;
    VernierCalibrationPoint vernier_calibration_points_[kVernierCalibrationMaxPoints] = {};
    uint32_t vernier_calibration_point_count_ = 0;
    bool vernier_calibration_fit_valid_ = false;
    float vernier_calibration_fitted_main_offset_ = 0.0f;
    float vernier_calibration_fitted_aux_offset_ = 0.0f;
    float vernier_calibration_fit_score_ = 0.0f;
    float vernier_calibration_worst_residual_ = 0.0f;

};

#endif // __ENCODER_HPP
