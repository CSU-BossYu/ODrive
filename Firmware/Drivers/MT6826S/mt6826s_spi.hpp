#pragma once

#include <stdint.h>
#include <stddef.h>

#include <Drivers/STM32/stm32_gpio.hpp>
#include <Drivers/STM32/stm32_spi_arbiter.hpp>

// Small MT6826S SPI driver for ODrive v3.x style firmware.
//
// Scope:
//   - asynchronous DMA SPI read through Stm32SpiArbiter
//   - MT6826S burst angle read command: 0xA003, then 0x003..0x006 data bytes
//   - CRC/status/fixed-bit validation
//   - optional back-to-back two-sensor pair read helper for Vernier resolver
//
// Non-goals:
//   - EEPROM programming
//   - blocking SPI reads in the control loop
//   - encoder PLL / ODrive Encoder integration; this file only returns raw samples

class Mt6826sSpi {
public:
    static constexpr uint16_t kCountsPerRev = 32768;  // MT6826S ANGLE[14:0]
    static constexpr uint8_t kStatusOverspeed = 1u << 0;
    static constexpr uint8_t kStatusWeakField = 1u << 1;
    static constexpr uint8_t kStatusUndervoltage = 1u << 2;

    enum Error : uint32_t {
        ERROR_NONE              = 0,
        ERROR_NOT_INITIALIZED   = 1u << 0,
        ERROR_SPI_BUSY          = 1u << 1,
        ERROR_DMA_FAILED        = 1u << 2,
        ERROR_CRC_MISMATCH      = 1u << 3,
        ERROR_FIXED_BITS        = 1u << 4,
        ERROR_STATUS_WARNING    = 1u << 5,
        ERROR_NULL_ARGUMENT     = 1u << 6,
    };

    struct Config {
        // ODrive v3 absolute SPI encoders commonly use PRESCALER_16.
        // Keep this conservative first; raise only after checking DMA timing and signal integrity.
        uint32_t baudrate_prescaler = SPI_BAUDRATEPRESCALER_4;
        uint32_t clk_polarity = SPI_POLARITY_HIGH;
        uint32_t clk_phase = SPI_PHASE_2EDGE;

        // Datasheet CRC is CRC-8 poly x^8 + x^2 + x + 1, applied to registers 0x003..0x005.
        bool check_crc = true;
        uint8_t crc_init = 0x00;

        // In normal angle burst response, reg 0x004 bit0 is fixed 0,
        // and reg 0x005 bits[7:3] are fixed 00000.
        bool check_fixed_bits = true;

        // STATUS[2:0] reports warnings. They are always counted; this flag decides
        // whether a status warning also invalidates the sample.
        bool fail_on_status_warning = false;
    };

    struct Sample {
        uint16_t angle = 0;        // 0..32767, ANGLE[14:0]
        uint8_t status = 0;        // STATUS[2:0]
        uint8_t crc = 0;           // received CRC byte, register 0x006
        uint8_t crc_calc = 0;      // calculated CRC for diagnostics
        uint8_t raw[4] = {0, 0, 0, 0}; // raw registers: 0x003, 0x004, 0x005, 0x006
        uint32_t sequence = 0;
        bool valid = false;
    };

    typedef void (*CompletionCallback)(void* ctx, const Sample& sample, bool success);

    Mt6826sSpi() = default;

    void init(Stm32SpiArbiter* spi_arbiter, Stm32Gpio ncs_gpio, const Config& config);
    void set_config(const Config& config);

    bool is_initialized() const { return spi_arbiter_ != nullptr; }
    bool is_busy() const { return transaction_in_flight_ || task_.is_in_use; }

    // Starts one non-blocking burst angle read. Returns false if the task cannot be queued.
    // Completion callback runs from the SPI/DMA completion context, so keep it short.
    bool start_sample_async(CompletionCallback cb = nullptr, void* ctx = nullptr);

    // Copies latest sample. consume_sample() clears the "new sample" flag; read_latest_sample() does not.
    bool consume_sample(Sample* out);
    bool read_latest_sample(Sample* out) const;
    bool read_last_sample_for_diagnostics(Sample* out) const;
    bool has_new_sample() const { return new_sample_ready_; }

    uint32_t error() const { return error_; }
    void clear_error() { error_ = ERROR_NONE; }

    uint32_t spi_busy_count() const { return spi_busy_count_; }
    uint32_t spi_dma_error_count() const { return spi_dma_error_count_; }
    uint32_t crc_error_count() const { return crc_error_count_; }
    uint32_t fixed_bit_error_count() const { return fixed_bit_error_count_; }
    uint32_t status_warning_count() const { return status_warning_count_; }
    uint32_t sample_count() const { return sample_count_; }

    static uint8_t crc8_mt6826s(const uint8_t* data, size_t length, uint8_t init = 0x00);
    static bool parse_burst_response(const uint8_t rx[6], const Config& config, Sample* out, uint32_t sequence);

private:
    static constexpr size_t kBurstFrameLen = 6;
    static constexpr uint16_t kBurstReadAngleCommand = 0xA003; // C3..C0=1010, address=0x003

    static void spi_done_cb(void* ctx, bool success);
    void handle_spi_done(bool success);

    SPI_InitTypeDef make_spi_config() const;
    void prepare_tx_buffer();
    void set_error(Error error) { error_ |= error; }

    Stm32SpiArbiter* spi_arbiter_ = nullptr;
    Stm32Gpio ncs_gpio_;
    Config config_;

    Stm32SpiArbiter::SpiTask task_ = {};
    uint8_t tx_[kBurstFrameLen] = {};
    uint8_t rx_[kBurstFrameLen] = {};

    CompletionCallback completion_cb_ = nullptr;
    void* completion_ctx_ = nullptr;

    Sample latest_sample_ = {};
    uint32_t next_sequence_ = 0;

    volatile bool transaction_in_flight_ = false;
    volatile bool new_sample_ready_ = false;

    volatile uint32_t error_ = ERROR_NONE;
    volatile uint32_t spi_busy_count_ = 0;
    volatile uint32_t spi_dma_error_count_ = 0;
    volatile uint32_t crc_error_count_ = 0;
    volatile uint32_t fixed_bit_error_count_ = 0;
    volatile uint32_t status_warning_count_ = 0;
    volatile uint32_t sample_count_ = 0;
};


class Mt6826sSpiPair {
public:
    enum Error : uint32_t {
        ERROR_NONE             = 0,
        ERROR_NOT_INITIALIZED  = 1u << 0,
        ERROR_PAIR_BUSY        = 1u << 1,
        ERROR_MAIN_START_FAIL  = 1u << 2,
        ERROR_AUX_START_FAIL   = 1u << 3,
        ERROR_MAIN_READ_FAIL   = 1u << 4,
        ERROR_AUX_READ_FAIL    = 1u << 5,
    };

    struct PairSample {
        Mt6826sSpi::Sample main = {};
        Mt6826sSpi::Sample aux = {};
        uint32_t sequence = 0;
        uint32_t request_cycles = 0;
        uint32_t main_complete_cycles = 0;
        uint32_t aux_complete_cycles = 0;
        bool valid = false;
    };

    typedef void (*CompletionCallback)(void* ctx, const PairSample& sample, bool success);

    Mt6826sSpiPair() = default;

    void init(Mt6826sSpi* main_sensor, Mt6826sSpi* aux_sensor);

    bool is_initialized() const { return main_ != nullptr && aux_ != nullptr; }
    bool is_busy() const { return state_ != STATE_IDLE; }

    // Starts back-to-back main then aux reads on the same SPI arbiter.
    // This does not make the two samples simultaneous; it gives them one pair sequence number.
    bool start_sample_async(CompletionCallback cb = nullptr, void* ctx = nullptr);

    bool consume_pair(PairSample* out);
    bool read_latest_pair(PairSample* out) const;
    bool has_new_pair() const { return new_pair_ready_; }

    uint32_t error() const { return error_; }
    void clear_error() { error_ = ERROR_NONE; }

    uint32_t pair_count() const { return pair_count_; }
    uint32_t pair_error_count() const { return pair_error_count_; }

private:
    enum State : uint8_t {
        STATE_IDLE = 0,
        STATE_READING_MAIN,
        STATE_READING_AUX,
    };

    static void main_done_cb(void* ctx, const Mt6826sSpi::Sample& sample, bool success);
    static void aux_done_cb(void* ctx, const Mt6826sSpi::Sample& sample, bool success);

    void handle_main_done(const Mt6826sSpi::Sample& sample, bool success);
    void handle_aux_done(const Mt6826sSpi::Sample& sample, bool success);
    void finish(bool success, Error error_if_failed = ERROR_NONE);
    void set_error(Error error) { error_ |= error; }

    Mt6826sSpi* main_ = nullptr;
    Mt6826sSpi* aux_ = nullptr;

    volatile State state_ = STATE_IDLE;

    CompletionCallback completion_cb_ = nullptr;
    void* completion_ctx_ = nullptr;

    PairSample pending_pair_ = {};
    PairSample latest_pair_ = {};
    Error pending_error_ = ERROR_NONE;
    uint32_t next_sequence_ = 0;

    volatile bool new_pair_ready_ = false;
    volatile uint32_t error_ = ERROR_NONE;
    volatile uint32_t pair_count_ = 0;
    volatile uint32_t pair_error_count_ = 0;
};
