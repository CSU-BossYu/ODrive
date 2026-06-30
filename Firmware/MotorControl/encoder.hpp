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
    static constexpr uint32_t MODE_FLAG_ABS = 0x100;
    struct Config_t {
        Mode mode = MODE_SPI_ABS_MT6826S_VERNIER;
        float calib_range = 0.02f; // Accuracy required to pass encoder cpr check
        float calib_scan_distance = 16.0f * M_PI; // rad electrical
        float calib_scan_omega = 4.0f * M_PI; // rad/s electrical
        float bandwidth = 1000.0f;
        int32_t phase_offset = 0;        // Offset between encoder count and rotor electrical phase
        float phase_offset_float = 0.0f; // Sub-count phase alignment offset
        int32_t cpr = 32768;
        bool pre_calibrated = false; // If true, this means the offset stored in
                                    // configuration is valid and does not need
                                    // be determined by run_offset_calibration.
        int32_t direction = 0; // direction with respect to motor
        bool enable_phase_interpolation = true; // Use velocity to interpolate inside the count state
        uint16_t abs_spi_cs_gpio_pin = 1;
        uint16_t abs_spi_aux_cs_gpio_pin = 4;
        float vernier_main_ratio = 1.0f;
        float vernier_aux_ratio = 1.0f;
        float vernier_main_offset = 0.0f;
        float vernier_aux_offset = 0.0f;
        bool vernier_main_reversed = false;
        bool vernier_aux_reversed = false;
        int32_t vernier_virtual_cpr = 32768;
        float vernier_err_accept = 0.02f;
        float vernier_err_reject = 0.08f;
        uint16_t mt6826s_spi_mode = 3;
        uint16_t mt6826s_spi_prescaler = 8;


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
        void set_vernier_err_accept(float value) { vernier_err_accept = value; parent->apply_vernier_resolver_config(); }
        void set_vernier_err_reject(float value) { vernier_err_reject = value; parent->apply_vernier_resolver_config(); }
        void set_mt6826s_spi_mode(uint16_t value) { mt6826s_spi_mode = value; parent->apply_mt6826s_spi_config(); }
        void set_mt6826s_spi_prescaler(uint16_t value) { mt6826s_spi_prescaler = value; parent->apply_mt6826s_spi_config(); }
        void set_pre_calibrated(bool value) { pre_calibrated = value; parent->check_pre_calibrated(); }
        void set_bandwidth(float value) { bandwidth = value; parent->update_pll_gains(); }
    };

    Encoder(Stm32SpiArbiter* spi_arbiter);
    
    bool apply_config(ODriveIntf::MotorIntf::MotorType motor_type);
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
    int32_t count_in_cpr_ = 0;
    float interpolation_ = 0.0f;
    OutputPort<float> phase_ = 0.0f;     // [rad]
    OutputPort<float> phase_vel_ = 0.0f; // [rad/s]
    float pos_estimate_counts_ = 0.0f;  // [count]
    float pos_cpr_counts_ = 0.0f;  // [count]
    float delta_pos_cpr_counts_ = 0.0f;  // [count] phase detector result for debug
    float vel_estimate_counts_ = 0.0f;  // [count/s]
    float pll_kp_ = 0.0f;   // [count/s / count]
    float pll_ki_ = 0.0f;   // [(count/s^2) / count]
    int32_t pos_abs_ = 0;
    float spi_error_rate_ = 0.0f;

    OutputPort<float> pos_estimate_ = 0.0f; // [turn]
    OutputPort<float> vel_estimate_ = 0.0f; // [turn/s]
    OutputPort<float> pos_circular_ = 0.0f; // [turn]

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
    void publish_vernier_output_estimate(float dt);
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
    };
    void get_vernier_diagnostics_snapshot(VernierDiagnosticsSnapshot* out);

    Mt6826sSpi::Sample mt6826s_main_sample_;
    Mt6826sSpi::Sample mt6826s_aux_sample_;
    VernierResolver::Result vernier_result_;
    float vernier_output_pos_estimate_ = 0.0f;
    float vernier_output_vel_estimate_ = 0.0f;
    float vernier_output_sample_dt_ = 0.0f;
    uint32_t vernier_output_pair_sequence_ = 0;
    bool vernier_output_estimate_valid_ = false;
    uint32_t mt6826s_pair_sequence_ = 0;
    uint32_t mt6826s_vernier_sample_counter_ = 0;
    bool mt6826s_pair_valid_ = false;

    constexpr float getCoggingRatio(){
        return 1.0f / 3600.0f;
    }

};

#endif // __ENCODER_HPP
