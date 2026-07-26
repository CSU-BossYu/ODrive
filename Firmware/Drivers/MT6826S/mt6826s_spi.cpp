#include "mt6826s_spi.hpp"

#include <Drivers/STM32/stm32_system.h>

// ------------------------
// Mt6826sSpi
// ------------------------

void Mt6826sSpi::init(Stm32SpiArbiter* spi_arbiter, Stm32Gpio ncs_gpio, const Config& config) {
    spi_arbiter_ = spi_arbiter;
    ncs_gpio_ = ncs_gpio;
    config_ = config;

    completion_cb_ = nullptr;
    completion_ctx_ = nullptr;
    transaction_in_flight_ = false;
    new_sample_ready_ = false;
    error_ = ERROR_NONE;
    next_sequence_ = 0;
    latest_sample_ = {};

    spi_busy_count_ = 0;
    spi_dma_error_count_ = 0;
    crc_error_count_ = 0;
    fixed_bit_error_count_ = 0;
    status_warning_count_ = 0;
    sample_count_ = 0;

    prepare_tx_buffer();

    if (spi_arbiter_) {
        ncs_gpio_.config(GPIO_MODE_OUTPUT_PP, GPIO_PULLUP);
        ncs_gpio_.write(true);
    } else {
        set_error(ERROR_NOT_INITIALIZED);
    }
}

void Mt6826sSpi::set_config(const Config& config) {
    uint32_t prim = cpu_enter_critical();
    config_ = config;
    cpu_exit_critical(prim);
}

SPI_InitTypeDef Mt6826sSpi::make_spi_config() const {
    SPI_InitTypeDef cfg = {};
    cfg.Mode = SPI_MODE_MASTER;
    cfg.Direction = SPI_DIRECTION_2LINES;
    cfg.DataSize = SPI_DATASIZE_8BIT;
    cfg.CLKPolarity = config_.clk_polarity;
    cfg.CLKPhase = config_.clk_phase;
    cfg.NSS = SPI_NSS_SOFT;
    cfg.BaudRatePrescaler = config_.baudrate_prescaler;
    cfg.FirstBit = SPI_FIRSTBIT_MSB;
    cfg.TIMode = SPI_TIMODE_DISABLE;
    cfg.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    cfg.CRCPolynomial = 10;
    return cfg;
}

void Mt6826sSpi::prepare_tx_buffer() {
    // Full duplex burst frame:
    //   MOSI[0..1] = 0xA003 command
    //   MOSI[2..5] = don't-care clocks to receive registers 0x003..0x006
    tx_[0] = static_cast<uint8_t>((kBurstReadAngleCommand >> 8) & 0xffu);
    tx_[1] = static_cast<uint8_t>(kBurstReadAngleCommand & 0xffu);
    tx_[2] = 0x00;
    tx_[3] = 0x00;
    tx_[4] = 0x00;
    tx_[5] = 0x00;
}

bool Mt6826sSpi::start_sample_async(CompletionCallback cb, void* ctx) {
    if (!spi_arbiter_) {
        set_error(ERROR_NOT_INITIALIZED);
        return false;
    }

    if (transaction_in_flight_) {
        ++spi_busy_count_;
        set_error(ERROR_SPI_BUSY);
        return false;
    }

    if (!Stm32SpiArbiter::acquire_task(&task_)) {
        ++spi_busy_count_;
        set_error(ERROR_SPI_BUSY);
        return false;
    }

    completion_cb_ = cb;
    completion_ctx_ = ctx;

    prepare_tx_buffer();

    task_.config = make_spi_config();
    task_.ncs_gpio = ncs_gpio_;
    task_.tx_buf = tx_;
    task_.rx_buf = rx_;
    task_.length = kBurstFrameLen;
    task_.on_complete = &Mt6826sSpi::spi_done_cb;
    task_.on_complete_ctx = this;
    task_.next = nullptr;

    transaction_in_flight_ = true;
    spi_arbiter_->transfer_async(&task_);
    return true;
}

void Mt6826sSpi::spi_done_cb(void* ctx, bool success) {
    reinterpret_cast<Mt6826sSpi*>(ctx)->handle_spi_done(success);
}

void Mt6826sSpi::handle_spi_done(bool success) {
    Sample sample = {};
    bool ok = success;

    if (!success) {
        ++spi_dma_error_count_;
        set_error(ERROR_DMA_FAILED);
    } else {
        const uint32_t seq = next_sequence_ + 1;
        ok = parse_burst_response(rx_, config_, &sample, seq);
        const uint8_t status = rx_[4] & 0x07u;

        if (status) {
            ++status_warning_count_;
            if (config_.fail_on_status_warning) {
                set_error(ERROR_STATUS_WARNING);
            }
        }

        if (!ok) {
            if (sample.crc_calc != sample.crc) {
                ++crc_error_count_;
                set_error(ERROR_CRC_MISMATCH);
            }

            // Fixed-bit and status-warning errors are not uniquely encoded by parse result,
            // so detect them here for counters/diagnostics.
            const bool fixed_bits_bad = ((rx_[3] & 0x01u) != 0u) || ((rx_[4] & 0xf8u) != 0u);
            if (config_.check_fixed_bits && fixed_bits_bad) {
                ++fixed_bit_error_count_;
                set_error(ERROR_FIXED_BITS);
            }
        }
    }

    if (success) {
        uint32_t prim = cpu_enter_critical();
        next_sequence_ = sample.sequence;
        latest_sample_ = sample;
        new_sample_ready_ = true;
        if (ok) {
            ++sample_count_;
        }
        cpu_exit_critical(prim);
    }

    transaction_in_flight_ = false;

    CompletionCallback cb = completion_cb_;
    void* cb_ctx = completion_ctx_;
    completion_cb_ = nullptr;
    completion_ctx_ = nullptr;

    Stm32SpiArbiter::release_task(&task_);

    if (cb) {
        cb(cb_ctx, sample, ok);
    }
}

bool Mt6826sSpi::consume_sample(Sample* out) {
    if (!out) {
        set_error(ERROR_NULL_ARGUMENT);
        return false;
    }

    uint32_t prim = cpu_enter_critical();
    const bool ready = new_sample_ready_;
    if (ready) {
        *out = latest_sample_;
        new_sample_ready_ = false;
    }
    cpu_exit_critical(prim);
    return ready;
}

bool Mt6826sSpi::read_latest_sample(Sample* out) const {
    if (!out) {
        return false;
    }

    uint32_t prim = cpu_enter_critical();
    const bool valid = latest_sample_.valid;
    if (valid) {
        *out = latest_sample_;
    }
    cpu_exit_critical(prim);
    return valid;
}

bool Mt6826sSpi::read_last_sample_for_diagnostics(Sample* out) const {
    if (!out) {
        return false;
    }

    uint32_t prim = cpu_enter_critical();
    *out = latest_sample_;
    const bool has_sample = latest_sample_.sequence != 0u;
    cpu_exit_critical(prim);
    return has_sample;
}

uint8_t Mt6826sSpi::crc8_mt6826s(const uint8_t* data, size_t length, uint8_t init) {
    uint8_t crc = init;

    for (size_t i = 0; i < length; ++i) {
        uint8_t byte = data[i];
        for (int bit = 0; bit < 8; ++bit) {
            const bool feedback = ((crc ^ byte) & 0x80u) != 0u;
            crc <<= 1;
            if (feedback) {
                crc ^= 0x07u; // x^8 + x^2 + x + 1
            }
            byte <<= 1;
        }
    }

    return crc;
}

bool Mt6826sSpi::parse_burst_response(const uint8_t rx[6], const Config& config, Sample* out, uint32_t sequence) {
    if (!rx || !out) {
        return false;
    }

    Sample sample = {};

    // rx[0..1] are don't-care/Hi-Z during the 16-bit command phase.
    const uint8_t reg003 = rx[2];
    const uint8_t reg004 = rx[3];
    const uint8_t reg005 = rx[4];
    const uint8_t reg006 = rx[5];

    sample.raw[0] = reg003;
    sample.raw[1] = reg004;
    sample.raw[2] = reg005;
    sample.raw[3] = reg006;

    sample.angle = (static_cast<uint16_t>(reg003) << 7)
                 | (static_cast<uint16_t>(reg004) >> 1);
    sample.status = reg005 & 0x07u;
    sample.crc = reg006;
    sample.crc_calc = crc8_mt6826s(&rx[2], 3, config.crc_init);
    sample.sequence = sequence;

    bool ok = true;

    if (config.check_fixed_bits) {
        // reg004 bit0 fixed 0; reg005 bits[7:3] fixed 00000.
        ok = ok && ((reg004 & 0x01u) == 0u);
        ok = ok && ((reg005 & 0xf8u) == 0u);
    }

    if (config.check_crc) {
        ok = ok && (sample.crc_calc == sample.crc);
    }

    if (config.fail_on_status_warning) {
        ok = ok && (sample.status == 0u);
    }

    sample.valid = ok;
    *out = sample;
    return ok;
}


// ------------------------
// Mt6826sSpiPair
// ------------------------

void Mt6826sSpiPair::init(Mt6826sSpi* main_sensor, Mt6826sSpi* aux_sensor) {
    main_ = main_sensor;
    aux_ = aux_sensor;
    state_ = STATE_IDLE;

    completion_cb_ = nullptr;
    completion_ctx_ = nullptr;

    pending_pair_ = {};
    pending_pair_.request_cycles = DWT->CYCCNT;
    latest_pair_ = {};
    pending_error_ = ERROR_NONE;
    next_sequence_ = 0;

    new_pair_ready_ = false;
    error_ = ERROR_NONE;
    pair_count_ = 0;
    pair_error_count_ = 0;

    if (!main_ || !aux_) {
        set_error(ERROR_NOT_INITIALIZED);
    }
}

bool Mt6826sSpiPair::start_sample_async(CompletionCallback cb, void* ctx) {
    if (!main_ || !aux_) {
        set_error(ERROR_NOT_INITIALIZED);
        return false;
    }

    if (state_ != STATE_IDLE) {
        set_error(ERROR_PAIR_BUSY);
        return false;
    }

    completion_cb_ = cb;
    completion_ctx_ = ctx;
    pending_pair_ = {};
    pending_error_ = ERROR_NONE;
    state_ = STATE_READING_MAIN;

    if (!main_->start_sample_async(&Mt6826sSpiPair::main_done_cb, this)) {
        state_ = STATE_IDLE;
        ++pair_error_count_;
        set_error(ERROR_MAIN_START_FAIL);
        return false;
    }

    return true;
}

void Mt6826sSpiPair::main_done_cb(void* ctx, const Mt6826sSpi::Sample& sample, bool success) {
    reinterpret_cast<Mt6826sSpiPair*>(ctx)->handle_main_done(sample, success);
}

void Mt6826sSpiPair::aux_done_cb(void* ctx, const Mt6826sSpi::Sample& sample, bool success) {
    reinterpret_cast<Mt6826sSpiPair*>(ctx)->handle_aux_done(sample, success);
}

void Mt6826sSpiPair::handle_main_done(const Mt6826sSpi::Sample& sample, bool success) {
    if (state_ != STATE_READING_MAIN) {
        finish(false, ERROR_MAIN_READ_FAIL);
        return;
    }

    pending_pair_.main = sample;
    pending_pair_.main_complete_cycles = DWT->CYCCNT;
    if (!success || !sample.valid) {
        pending_error_ = ERROR_MAIN_READ_FAIL;
    }

    state_ = STATE_READING_AUX;

    if (!aux_->start_sample_async(&Mt6826sSpiPair::aux_done_cb, this)) {
        finish(false, ERROR_AUX_START_FAIL);
    }
}

void Mt6826sSpiPair::handle_aux_done(const Mt6826sSpi::Sample& sample, bool success) {
    if (state_ != STATE_READING_AUX) {
        finish(false, ERROR_AUX_READ_FAIL);
        return;
    }

    pending_pair_.aux = sample;
    pending_pair_.aux_complete_cycles = DWT->CYCCNT;
    if (!success || !sample.valid) {
        if (pending_error_ == ERROR_NONE) {
            pending_error_ = ERROR_AUX_READ_FAIL;
        } else {
            set_error(ERROR_AUX_READ_FAIL);
        }
    }

    finish(pending_error_ == ERROR_NONE, pending_error_);
}

void Mt6826sSpiPair::finish(bool success, Error error_if_failed) {
    PairSample cb_sample = pending_pair_;

    if (success) {
        cb_sample.sequence = next_sequence_ + 1;
        cb_sample.valid = true;

        uint32_t prim = cpu_enter_critical();
        next_sequence_ = cb_sample.sequence;
        latest_pair_ = cb_sample;
        new_pair_ready_ = true;
        ++pair_count_;
        cpu_exit_critical(prim);
    } else {
        cb_sample.sequence = next_sequence_ + 1;
        cb_sample.valid = false;
        ++pair_error_count_;
        if (error_if_failed != ERROR_NONE) {
            set_error(error_if_failed);
        }
        uint32_t prim = cpu_enter_critical();
        next_sequence_ = cb_sample.sequence;
        latest_pair_ = cb_sample;
        new_pair_ready_ = true;
        cpu_exit_critical(prim);
    }

    state_ = STATE_IDLE;

    CompletionCallback cb = completion_cb_;
    void* cb_ctx = completion_ctx_;
    completion_cb_ = nullptr;
    completion_ctx_ = nullptr;

    if (cb) {
        cb(cb_ctx, cb_sample, success);
    }
}

bool Mt6826sSpiPair::consume_pair(PairSample* out) {
    if (!out) {
        return false;
    }

    uint32_t prim = cpu_enter_critical();
    const bool ready = new_pair_ready_;
    if (ready) {
        *out = latest_pair_;
        new_pair_ready_ = false;
    }
    cpu_exit_critical(prim);
    return ready;
}

bool Mt6826sSpiPair::read_latest_pair(PairSample* out) const {
    if (!out) {
        return false;
    }

    uint32_t prim = cpu_enter_critical();
    const bool valid = latest_pair_.valid;
    if (valid) {
        *out = latest_pair_;
    }
    cpu_exit_critical(prim);
    return valid;
}
