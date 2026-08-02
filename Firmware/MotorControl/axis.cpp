
#include <stdlib.h>
#include <functional>
#include "gpio.h"

#include "odrive_main.h"
#include "control_timeout.hpp"
#include "crash_recorder.hpp"
#include "utils.hpp"
#include "communication/interface_can.hpp"

namespace {

// TEMPORARY: exclude reducer geometry, mechanical and delay identification.
// Motor flux linkage/Kt is still identified by an independent electrical scan.
constexpr bool kEnableReducerIdentification = false;

void apply_supervisor_fault(
        void* context,
        odrive::fault::FaultRecord& record,
        uint32_t projection,
        bool record_trace) {
    if (context != nullptr) {
        static_cast<Axis*>(context)->raise_fault(
            record, static_cast<Axis::Error>(projection), record_trace);
    }
}

bool clear_supervisor_faults(
        void* context,
        const odrive::fault::ClearRequest& request,
        bool* empty_after_clear) {
    if (context == nullptr) {
        return false;
    }
    return static_cast<Axis*>(context)->clear_faults_for_supervisor(
        request, empty_after_clear);
}

bool first_supervisor_fault(
        void* context,
        odrive::fault::FaultRecord* record) {
    return context != nullptr &&
        static_cast<Axis*>(context)->first_fault(record);
}

}  // namespace

Axis::Axis(int axis_num,
           osPriority thread_priority,
           Encoder& encoder,
           Controller& controller,
           Motor& motor,
           TrapezoidalTrajectory& trap)
    : axis_num_(axis_num),
      thread_priority_(thread_priority),
      encoder_(encoder),
      controller_(controller),
      motor_(motor),
      trap_traj_(trap),
      safety_supervisor_(fault_manager_, realtime_event_ring_, this,
                         apply_supervisor_fault, clear_supervisor_faults,
                         first_supervisor_fault),
      trace_consumer_(critical_event_ring_, state_event_ring_, log_ring_,
                      scope_ring_)
{
    encoder_.axis_ = this;
    controller_.axis_ = this;
    motor_.axis_ = this;
    trap_traj_.axis_ = this;
    odrive::safety::Command boot;
    boot.request_id = allocate_command_request_id();
    boot.source = odrive::safety::CommandSource::INTERNAL;
    boot.type = odrive::safety::CommandType::BOOT_COMPLETE;
    safety_supervisor_.submit_command(boot);
    supervisor_trace_mailbox_.publish_from_task({
        safety_supervisor_.state_epoch(),
        static_cast<uint8_t>(safety_supervisor_.state()),
        static_cast<uint8_t>(safety_supervisor_.operation()), 0u, 0u});
}

uint32_t Axis::raise_fault(odrive::fault::FaultRecord record,
                           Error legacy_projection,
                           bool record_trace) {
    uint32_t sequence = 0;
    bool first_fault = false;
    CRITICAL_SECTION() {
        if (fault_manager_.raise(record)) {
            sequence = record.fault_sequence;
        }
        odrive::fault::FaultRecord first;
        first_fault = fault_manager_.first_fault(&first) &&
            first.fault_sequence == record.fault_sequence;
        if (legacy_projection != ERROR_NONE) {
            error_ |= legacy_projection;  // LEGACY_ERROR_PROJECTION
        }
    }

    if (!record_trace) {
        return sequence;
    }

    odrive::trace::CriticalEvent event;
    event.sequence = record.control_sequence;
    event.control_sequence = record.control_sequence;
    event.state_epoch = record.state_epoch;
    event.timestamp_cycles = record.timestamp_cycles;
    event.fault_sequence = record.fault_sequence;
    event.type = odrive::trace::CriticalEventType::FAULT;
    event.source = record.source;
    event.code = record.code;
    event.site = record.site;
    event.severity = record.severity;
    event.parent_fault_sequence = record.parent_fault_sequence;
    event.arg0 = record.arg0;
    event.arg1 = record.arg1;
    event.arg2 = record.arg2;
    critical_event_ring_.push_from_task(event);
    CRITICAL_SECTION() {
        critical_black_box_.record_from_context(event);
        if (first_fault) {
            critical_black_box_.freeze_on_first_fault();
        }
    }
    return sequence;
}

bool Axis::clear_faults(const odrive::fault::ClearRequest& request) {
    bool changed = false;
    CRITICAL_SECTION() {
        changed = fault_manager_.clear(request);
    }
    return changed;
}

bool Axis::clear_faults_for_supervisor(
        const odrive::fault::ClearRequest& request,
        bool* empty_after_clear) {
    bool changed = false;
    bool empty = false;
    CRITICAL_SECTION() {
        changed = fault_manager_.clear(request);
        empty = fault_manager_.record_count() == 0u;
        if (empty) {
            motor_.error_ = Motor::ERROR_NONE;
            controller_.error_ = Controller::ERROR_NONE;
            controller_.clear_overspeed_snapshot();
            encoder_.error_ = Encoder::ERROR_NONE;
            encoder_.spi_error_rate_ = 0.0f;
            encoder_.clear_lz5710_faults();
            error_ = ERROR_NONE;
        }
        if (empty_after_clear != nullptr) {
            *empty_after_clear = empty;
        }
    }
    if (empty) {
        ControlTimeout::clear(*this);
        ControlTimeout::feed_command(*this);
        CRITICAL_SECTION() {
            critical_black_box_.reset();
        }
    }
    return changed;
}

void Axis::clear_errors() {
    bool empty_after_clear = false;
    clear_faults_for_supervisor({0u, 0u}, &empty_after_clear);
    if (empty_after_clear) {
        safety_supervisor_.handle_external_clear();
    }
}

bool Axis::first_fault(odrive::fault::FaultRecord* out) const {
    bool found = false;
    CRITICAL_SECTION() {
        found = fault_manager_.first_fault(out);
    }
    return found;
}

bool Axis::submit_command(const odrive::safety::Command& command) {
    return safety_supervisor_.submit_command(command);
}

bool Axis::submit_realtime_event(const odrive::safety::RealtimeEvent& event) {
    return realtime_event_ring_.push(event);
}

bool Axis::record_critical_event_from_isr(
        const odrive::trace::CriticalEvent& event) {
    crash_recorder_note_critical(
        event.sequence, event.control_sequence, event.timestamp_cycles,
        static_cast<uint16_t>(event.source),
        static_cast<uint16_t>(event.code),
        static_cast<uint16_t>(event.site),
        static_cast<uint8_t>(event.severity),
        event.arg0, event.arg1, event.arg2);
    const bool queued = critical_event_ring_.push_from_isr(event);
    critical_black_box_.record_from_isr(event);
    if (event.type == odrive::trace::CriticalEventType::FAULT) {
        critical_black_box_.freeze_on_first_fault();
        scope_capture_.trigger_fault_from_isr();
    }
    return queued;
}

bool Axis::snapshot_critical_black_box(
        odrive::trace::CriticalEvent* output, size_t capacity,
        size_t* written) const {
    bool success = false;
    CRITICAL_SECTION() {
        success = critical_black_box_.snapshot(output, capacity, written);
    }
    return success;
}

void Axis::capture_realtime_snapshot_from_isr(uint32_t isr_start_cycles) {
    (void)isr_start_cycles;
    odrive::trace::RealtimeSnapshot snapshot;
    snapshot.sequence = odrv.n_evt_control_loop_;
    snapshot.control_sequence = odrv.n_evt_control_loop_;
    snapshot.encoder_sample_sequence = encoder_.consumed_main_sample_sequence_;
    snapshot.feedback_sequence = encoder_.consumed_vernier_pair_sequence_;
    snapshot.timestamp_cycles = DWT->CYCCNT;
    snapshot.isr_cycles = previous_complete_isr_cycles_;
    snapshot.encoder_cycles = task_times_.encoder_update.max_length_;
    snapshot.controller_cycles = task_times_.controller_update.max_length_;
    snapshot.motor_cycles = task_times_.motor_update.max_length_;

    snapshot.phase = encoder_.phase_.present().value_or(0.0f);
    snapshot.phase_velocity = encoder_.phase_vel_.present().value_or(0.0f);
    snapshot.position = encoder_.pos_estimate_.present().value_or(0.0f);
    snapshot.velocity = encoder_.vel_estimate_.present().value_or(0.0f);
    snapshot.id_measured = motor_.current_control_.Id_measured_;
    snapshot.iq_measured = motor_.current_control_.Iq_measured_;
    const auto idq_setpoint = motor_.Idq_setpoint_.present().value_or(
        float2D{0.0f, 0.0f});
    snapshot.id_setpoint = idq_setpoint.first;
    snapshot.iq_setpoint = idq_setpoint.second;
    snapshot.torque_setpoint =
        controller_.torque_output_.present().value_or(0.0f);
    snapshot.controller_output = snapshot.torque_setpoint;
    snapshot.flags = (motor_.is_armed_ ? 1u : 0u) |
        (controller_feedback_active_ ? 2u : 0u) |
        (closed_loop_prepared_ ? 4u : 0u);
    odrive::trace::SupervisorTraceState supervisor_state;
    if (supervisor_trace_mailbox_.read_from_isr(&supervisor_state)) {
        snapshot.state_epoch = supervisor_state.state_epoch;
        snapshot.safety_state = supervisor_state.safety_state;
        snapshot.operation = supervisor_state.operation;
        snapshot.readiness_flags = supervisor_state.readiness_flags;
    } else {
        snapshot.flags |= 0x80u;
    }

    realtime_snapshot_buffer_.publish_from_isr(snapshot);
    crash_recorder_update_context(
        snapshot.control_sequence, snapshot.state_epoch,
        snapshot.safety_state, snapshot.operation);

    odrive::scope::generated::ScopeSampleContext scope_sample;
    scope_sample.control_sequence = snapshot.control_sequence;
    scope_sample.timestamp_cycles = snapshot.timestamp_cycles;
    scope_sample.state_epoch = snapshot.state_epoch;
    scope_sample.safety_state = snapshot.safety_state;
    scope_sample.operation = snapshot.operation;
    scope_sample.phase = snapshot.phase;
    scope_sample.phase_velocity = snapshot.phase_velocity;
    scope_sample.position = snapshot.position;
    scope_sample.velocity = snapshot.velocity;
    scope_sample.id_measured = snapshot.id_measured;
    scope_sample.iq_measured = snapshot.iq_measured;
    scope_sample.id_setpoint = snapshot.id_setpoint;
    scope_sample.iq_setpoint = snapshot.iq_setpoint;
    scope_sample.torque_setpoint = snapshot.torque_setpoint;
    scope_sample.controller_output = snapshot.controller_output;
    scope_capture_.sample_from_isr(scope_sample);
}

void Axis::finish_realtime_snapshot_from_isr(uint32_t isr_start_cycles) {
    previous_complete_isr_cycles_ = DWT->CYCCNT - isr_start_cycles;
}

bool Axis::read_realtime_snapshot(
        odrive::trace::RealtimeSnapshot* snapshot) const {
    return realtime_snapshot_buffer_.read_consistent(snapshot);
}

void Axis::service_trace_buffers() {
    const uint8_t state = static_cast<uint8_t>(safety_supervisor_.state());
    const uint8_t operation =
        static_cast<uint8_t>(safety_supervisor_.operation());
    const auto readiness = safety_supervisor_.readiness();
    scope_capture_.notify_state_from_task(state);
    if (state != last_trace_state_ || operation != last_trace_operation_ ||
        safety_supervisor_.state_epoch() != last_trace_epoch_ ||
        readiness.sequence != last_trace_readiness_sequence_) {
        odrive::trace::StateEvent event;
        event.sequence = ++trace_sequence_;
        event.state_epoch = safety_supervisor_.state_epoch();
        event.timestamp_cycles = DWT->CYCCNT;
        event.request_id = last_command_result_valid_
            ? last_command_result_.request_id : 0u;
        event.state = state;
        event.operation = operation;
        event.axis_state = static_cast<uint8_t>(current_state_);
        event.type = readiness.sequence != last_trace_readiness_sequence_
            ? odrive::trace::StateEventType::READINESS
            : odrive::trace::StateEventType::TRANSITION;
        event.reason = last_command_result_valid_
            ? last_command_result_.reason
            : odrive::fault::FaultCode::NONE;
        state_event_ring_.push(event);
        last_trace_state_ = state;
        last_trace_operation_ = operation;
        last_trace_epoch_ = safety_supervisor_.state_epoch();
        last_trace_readiness_sequence_ = readiness.sequence;
    }
    trace_consumer_.consume();
}

void Axis::service_safety_supervisor() {
    constexpr size_t kMaximumServicePasses = 4;
    for (size_t pass = 0; pass < kMaximumServicePasses; ++pass) {
        safety_supervisor_.tick(odrv.n_evt_control_loop_);

        odrive::safety::CommandResult result;
        while (safety_supervisor_.pop_result(&result)) {
            last_command_result_ = result;
            last_command_result_valid_ = true;
            odrive::trace::LogRecord log;
            log.sequence = ++trace_sequence_;
            log.timestamp_cycles = DWT->CYCCNT;
            log.arg0 = result.request_id;
            log.arg1 = result.state_epoch;
            log.code = static_cast<uint16_t>(result.status);
            log.level = result.status == odrive::safety::CommandStatus::FAILED
                ? odrive::trace::LogLevel::ERROR
                : odrive::trace::LogLevel::INFO;
            log.category = 1u;
            log_ring_.push(log);
            for (const auto& slot : command_result_sinks_) {
                if (slot.sink != nullptr) {
                    slot.sink(slot.context, result);
                }
            }
        }

        uint32_t legacy_axis_state = 0;
        while (safety_supervisor_.pop_legacy_axis_state(&legacy_axis_state)) {
            requested_state_ = static_cast<AxisState>(legacy_axis_state);
        }

        odrive::safety::RealtimeRequest request;
        if (!safety_supervisor_.pop_realtime_request(&request)) {
            break;
        }

        odrive::safety::RealtimeEvent event;
        event.sequence = odrv.n_evt_control_loop_ == 0u
            ? 1u : odrv.n_evt_control_loop_;
        event.epoch = request.epoch;
        event.feedback_sequence = event.sequence;
        event.control_sequence = odrv.n_evt_control_loop_;
        event.timestamp_cycles = DWT->CYCCNT;

        switch (request.type) {
            case odrive::safety::RealtimeRequestType::PREPARE_OPERATION:
                if (request.operation == odrive::safety::Operation::CLOSED_LOOP &&
                    safety_supervisor_.state() ==
                        odrive::safety::SafetyState::PREPARING &&
                    request.epoch == safety_supervisor_.state_epoch() &&
                    prepare_closed_loop_control()) {
                    event.type = odrive::safety::RealtimeEventType::READY_SNAPSHOT;
                    event.readiness_flags = odrive::safety::kRequiredReadinessFlags;
                } else if (error_ == ERROR_NONE &&
                           motor_.error_ == Motor::ERROR_NONE &&
                           encoder_.error_ == Encoder::ERROR_NONE &&
                           controller_.error_ == Controller::ERROR_NONE) {
                    // Configuration/readiness rejection is a command outcome,
                    // not a persistent device fault. Undo partial preparation
                    // and let the Supervisor return to SAFE_OFF.
                    stop_closed_loop_control();
                    event.type =
                        odrive::safety::RealtimeEventType::PREPARE_REJECTED;
                    event.source = odrive::fault::FaultSource::CONTROLLER;
                    event.code = odrive::fault::FaultCode::CONTROLLER_REJECTED;
                    event.site = odrive::fault::FaultSite::CLOSED_LOOP_PREPARE;
                    event.severity = odrive::fault::FaultSeverity::INFO;
                } else {
                    event.type = odrive::safety::RealtimeEventType::FAULT;
                    event.source = odrive::fault::FaultSource::SAFETY;
                    event.code = odrive::fault::FaultCode::CONTROLLER_REJECTED;
                    event.site = odrive::fault::FaultSite::CLOSED_LOOP_PREPARE;
                    event.severity = odrive::fault::FaultSeverity::LATCHED;
                }
                safety_supervisor_.handle_realtime_event(event);
                break;

            case odrive::safety::RealtimeRequestType::ARM:
                if (safety_supervisor_.state() ==
                        odrive::safety::SafetyState::READY &&
                    request.epoch == safety_supervisor_.state_epoch() &&
                    arm_closed_loop_control()) {
                    event.type = odrive::safety::RealtimeEventType::ARM_CONFIRMED;
                    requested_state_ = AXIS_STATE_CLOSED_LOOP_CONTROL;
                } else {
                    event.type = odrive::safety::RealtimeEventType::FAULT;
                    event.source = odrive::fault::FaultSource::POWER_STAGE;
                    event.code = odrive::fault::FaultCode::POWER_STAGE_SHUTDOWN;
                    event.site = odrive::fault::FaultSite::POWER_STAGE_ARM;
                    event.severity = odrive::fault::FaultSeverity::LATCHED;
                    event.legacy_projection = ERROR_MOTOR_FAILED;
                }
                safety_supervisor_.handle_realtime_event(event);
                break;

            case odrive::safety::RealtimeRequestType::DISARM:
                stop_closed_loop_control();
                requested_state_ = AXIS_STATE_IDLE;
                event.type = odrive::safety::RealtimeEventType::DISARM_CONFIRMED;
                safety_supervisor_.handle_realtime_event(event);
                break;
        }
    }

    if (safety_supervisor_.state() ==
        odrive::safety::SafetyState::FAULT_LATCHED) {
        motor_.disarm();
    }
    const auto readiness = safety_supervisor_.readiness();
    supervisor_trace_mailbox_.publish_from_task({
        safety_supervisor_.state_epoch(),
        static_cast<uint8_t>(safety_supervisor_.state()),
        static_cast<uint8_t>(safety_supervisor_.operation()),
        readiness.flags, 0u});
    service_trace_buffers();
}

uint32_t Axis::allocate_command_request_id() {
    ++command_request_sequence_;
    if (command_request_sequence_ == 0u) {
        ++command_request_sequence_;
    }
    return command_request_sequence_;
}

bool Axis::add_command_result_sink(void* context, CommandResultSink sink) {
    if (sink == nullptr) return false;
    for (const auto& slot : command_result_sinks_) {
        if (slot.context == context && slot.sink == sink) return true;
    }
    for (auto& slot : command_result_sinks_) {
        if (slot.sink == nullptr) {
            slot = {context, sink};
            return true;
        }
    }
    return false;
}

void Axis::set_trace_dispatch_sink(void* context,
                                   odrive::trace::TraceDispatchFn sink) {
    trace_consumer_.set_callback(context, sink);
}

Axis::LockinConfig_t Axis::default_calibration() {
    Axis::LockinConfig_t config;
    config.current = ODRIVE_PRODUCTION_AXIS_CALIBRATION_CURRENT;           // [A]
    config.ramp_time = 0.4f;          // [s]
    config.ramp_distance = 1 * M_PI;  // [rad]
    config.accel = 20.0f;     // [rad/s^2]
    config.vel = 40.0f; // [rad/s]
    config.finish_distance = 100.0f * 2.0f * M_PI;  // [rad]
    config.finish_on_vel = false;
    config.finish_on_distance = true;
    return config;
}

bool Axis::apply_config() {
    config_.parent = this;
    watchdog_feed();
    return true;
}

void Axis::clear_config() {
    config_ = {};
    config_.can.node_id = ODRIVE_PRODUCTION_CAN_NODE_ID + axis_num_;
}

static void run_state_machine_loop_wrapper(void* ctx) {
    reinterpret_cast<Axis*>(ctx)->run_state_machine_loop();
    reinterpret_cast<Axis*>(ctx)->thread_id_valid_ = false;
}

// @brief Starts run_state_machine_loop in a new thread
void Axis::start_thread() {
    osThreadDef(thread_def, run_state_machine_loop_wrapper, thread_priority_, 0, stack_size_ / sizeof(StackType_t));
    thread_id_ = osThreadCreate(osThread(thread_def), this);
    thread_id_valid_ = true;
}

/**
 * @brief Blocks until at least one complete control loop has been executed.
 */
bool Axis::wait_for_control_iteration() {
    osSignalWait(0x0001, osWaitForever); // this might return instantly
    osSignalWait(0x0001, osWaitForever); // this might be triggered at the
                                         // end of a control loop iteration
                                         // which was started before we entered
                                         // this function
    osSignalWait(0x0001, osWaitForever);
    return true;
}

bool Axis::start_calibration_session(uint32_t request_options) {
    // A calibration owns the axis. Do not splice it into a legacy state-chain
    // or replace live parameters while another operation is in progress.
    if (current_state_ != AXIS_STATE_IDLE ||
        requested_state_ != AXIS_STATE_UNDEFINED || motor_.is_armed_ ||
        calibration_session_.active()) {
        return false;
    }

    // A failed identification stage can leave component and axis errors
    // latched after the power stage has already been disarmed. Starting a new
    // explicit calibration is also an acknowledgement of those stale errors.
    // Persistent hardware faults are detected again by the normal checks or
    // the subsequent arm request.
    odrv.clear_errors();

    bool started = false;
    CRITICAL_SECTION() {
        started = calibration_session_.begin(request_options);
        if (started) {
            calibration_record_buffer_.reset();
            calibration_pending_result_ = {};
            calibration_pending_result_.session_id =
                calibration_session_.session_id();
            calibration_start_pending_ = true;
            // Wake run_idle_loop. The axis thread consumes this request and
            // then owns all subsequent stage changes.
            requested_state_ = AXIS_STATE_IDLE;
        }
    }
    return started;
}

bool Axis::abort_calibration_session() {
    bool aborted = false;
    CRITICAL_SECTION() {
        aborted = calibration_session_.abort();
        if (aborted) {
            calibration_start_pending_ = false;
            calibration_capture_enabled_ = false;
            requested_state_ = AXIS_STATE_IDLE;
        }
    }
    return aborted;
}

void Axis::capture_calibration_electrical_sample(
        uint32_t output_timestamp, const float (&pwm_timings)[3],
        bool pwm_valid) {
    if (!calibration_capture_enabled_) {
        return;
    }

    CalibrationElectricalSampleV1 sample = {};
    sample.sample_ticks = DWT->CYCCNT;
    sample.control_ticks = output_timestamp;
    sample.sequence = calibration_sample_sequence_++;
    sample.session_id = calibration_session_.session_id();
    sample.axis_state = static_cast<uint16_t>(current_state_);
    sample.vbus = vbus_voltage;
    sample.flags |= CAL_SAMPLE_VBUS_VALID;

    const auto& main = encoder_.mt6826s_main_sample_;
    const auto& aux = encoder_.mt6826s_aux_sample_;
    sample.main_raw = main.angle;
    sample.aux_raw = aux.angle;
    if (main.valid) sample.flags |= CAL_SAMPLE_MAIN_VALID;
    if (aux.valid) sample.flags |= CAL_SAMPLE_AUX_VALID;
    if (motor_.is_armed_) sample.flags |= CAL_SAMPLE_MOTOR_ARMED;
    if (encoder_.vernier_result_.valid) sample.flags |= CAL_SAMPLE_RESOLVER_VALID;
    if (encoder_.vernier_result_.degraded) sample.flags |= CAL_SAMPLE_RESOLVER_DEGRADED;

    if (motor_.current_meas_.has_value()) {
        sample.ia = motor_.current_meas_->phA;
        sample.ib = motor_.current_meas_->phB;
        sample.ic = motor_.current_meas_->phC;
        sample.flags |= CAL_SAMPLE_CURRENT_VALID;
    }

    const auto phase = motor_.current_control_.phase_;
    if (phase.has_value()) {
        sample.electrical_phase = *phase;
        sample.vd_applied = motor_.current_control_.final_v_d_;
        sample.vq_applied = motor_.current_control_.final_v_q_;
        sample.flags |= CAL_SAMPLE_APPLIED_VOLTAGE_VALID;
    }

    if (pwm_valid) {
        sample.duty_a = pwm_timings[0];
        sample.duty_b = pwm_timings[1];
        sample.duty_c = pwm_timings[2];
        sample.flags |= CAL_SAMPLE_DUTY_VALID;
        constexpr float kSaturationMargin = 0.001f;
        if (pwm_timings[0] <= kSaturationMargin || pwm_timings[0] >= 1.0f - kSaturationMargin ||
            pwm_timings[1] <= kSaturationMargin || pwm_timings[1] >= 1.0f - kSaturationMargin ||
            pwm_timings[2] <= kSaturationMargin || pwm_timings[2] >= 1.0f - kSaturationMargin) {
            sample.flags |= CAL_SAMPLE_PWM_SATURATED;
        }
    }

    calibration_record_buffer_.push_from_isr(
        CAL_RECORD_ELECTRICAL_FAST_V1, sample);
}

bool Axis::capture_calibration_full_sample(CalibrationSampleV1* captured) {
    CalibrationSampleV1 sample = {};
    sample.sample_ticks = DWT->CYCCNT;
    sample.control_ticks = motor_.current_control_.ctrl_timestamp_;
    sample.sequence = calibration_sample_sequence_++;
    sample.session_id = calibration_session_.session_id();
    sample.axis_state = static_cast<uint16_t>(current_state_);
    sample.vbus = vbus_voltage;
    sample.flags |= CAL_SAMPLE_VBUS_VALID;

    Mt6826sSpiPair::PairSample pair = {};
    const bool pair_available =
        encoder_.mt6826s_spi_pair_.read_latest_pair(&pair);
    const auto& main = pair_available
        ? pair.main : encoder_.mt6826s_main_sample_;
    const auto& aux = pair_available
        ? pair.aux : encoder_.mt6826s_aux_sample_;
    sample.main_sequence = main.sequence;
    sample.aux_sequence = aux.sequence;
    sample.pair_sequence = pair_available
        ? pair.sequence : encoder_.mt6826s_pair_sequence_;
    sample.pair_sample_skew_cycles = pair_available
        ? pair.aux_complete_cycles - pair.main_complete_cycles : 0;
    sample.main_raw = main.angle;
    sample.aux_raw = aux.angle;
    if (main.valid) sample.flags |= CAL_SAMPLE_MAIN_VALID;
    if (aux.valid) sample.flags |= CAL_SAMPLE_AUX_VALID;
    if (motor_.is_armed_) sample.flags |= CAL_SAMPLE_MOTOR_ARMED;
    if (encoder_.vernier_result_.valid) sample.flags |= CAL_SAMPLE_RESOLVER_VALID;
    if (encoder_.vernier_result_.degraded) sample.flags |= CAL_SAMPLE_RESOLVER_DEGRADED;

    if (motor_.current_meas_.has_value()) {
        sample.ia = motor_.current_meas_->phA;
        sample.ib = motor_.current_meas_->phB;
        sample.ic = motor_.current_meas_->phC;
        sample.id = motor_.current_control_.Id_measured_;
        sample.iq = motor_.current_control_.Iq_measured_;
        sample.flags |= CAL_SAMPLE_CURRENT_VALID;
    }
    sample.vd_applied = motor_.current_control_.final_v_d_;
    sample.vq_applied = motor_.current_control_.final_v_q_;
    sample.flags |= CAL_SAMPLE_APPLIED_VOLTAGE_VALID;

    if (motor_.last_pwm_timings_valid_) {
        sample.duty_a = motor_.last_pwm_timings_[0];
        sample.duty_b = motor_.last_pwm_timings_[1];
        sample.duty_c = motor_.last_pwm_timings_[2];
        sample.flags |= CAL_SAMPLE_DUTY_VALID;
        constexpr float kSaturationMargin = 0.001f;
        if (sample.duty_a <= kSaturationMargin ||
            sample.duty_a >= 1.0f - kSaturationMargin ||
            sample.duty_b <= kSaturationMargin ||
            sample.duty_b >= 1.0f - kSaturationMargin ||
            sample.duty_c <= kSaturationMargin ||
            sample.duty_c >= 1.0f - kSaturationMargin) {
            sample.flags |= CAL_SAMPLE_PWM_SATURATED;
        }
    }

    sample.position_turns = encoder_.pos_estimate_.any().value_or(0.0f);
    sample.velocity_turns_per_s =
        encoder_.vel_estimate_.any().value_or(0.0f);
    if (encoder_.mode_ == Encoder::MODE_SPI_ABS_MT6826S_VERNIER) {
        sample.position_turns /= 2.0f * M_PI;
        sample.velocity_turns_per_s /= 60.0f;
    }
    sample.electrical_phase =
        motor_.current_control_.phase_.value_or(0.0f);
    sample.electrical_velocity =
        motor_.current_control_.phase_vel_.value_or(0.0f);
    sample.output_position_turns = encoder_.vernier_result_.position_turns;
    sample.board_temperature = motor_.fet_thermistor_.temperature_;
    sample.motor_temperature = motor_.motor_thermistor_.temperature_;

    calibration_record_buffer_.push_from_isr(CAL_RECORD_FULL_V1, sample);
    if (captured) {
        *captured = sample;
    }
    return (sample.flags & (CAL_SAMPLE_MAIN_VALID | CAL_SAMPLE_AUX_VALID)) ==
           (CAL_SAMPLE_MAIN_VALID | CAL_SAMPLE_AUX_VALID);
}

bool Axis::run_calibration_encoder_alignment() {
    const uint32_t required =
        CalibrationPendingResult::VALID_PHASE_RESISTANCE |
        CalibrationPendingResult::VALID_PHASE_INDUCTANCE;
    if ((calibration_pending_result_.validity & required) != required) {
        return false;
    }

    const float active_resistance = motor_.config_.phase_resistance;
    const float active_inductance = motor_.config_.phase_inductance;
    const bool active_motor_calibrated = motor_.is_calibrated_;
    const int32_t active_direction = encoder_.config_.direction;
    const int32_t active_phase_offset = encoder_.config_.phase_offset;
    const float active_phase_offset_float = encoder_.config_.phase_offset_float;
    const bool active_encoder_ready = encoder_.is_ready_;

    motor_.config_.phase_resistance =
        calibration_pending_result_.phase_resistance;
    motor_.config_.phase_inductance =
        calibration_pending_result_.phase_inductance;
    motor_.is_calibrated_ = true;
    motor_.update_current_controller_gains();

    const bool success = encoder_.run_offset_calibration();
    if (success) {
        calibration_pending_result_.encoder_direction =
            encoder_.config_.direction;
        calibration_pending_result_.phase_offset =
            encoder_.config_.phase_offset;
        calibration_pending_result_.phase_offset_float =
            encoder_.config_.phase_offset_float;
        calibration_pending_result_.pole_pairs =
            encoder_.get_calibration_estimated_pole_pairs();
        calibration_pending_result_.validity |=
            CalibrationPendingResult::VALID_ENCODER_DIRECTION |
            CalibrationPendingResult::VALID_ELECTRICAL_OFFSET |
            CalibrationPendingResult::VALID_POLE_PAIRS;
    }

    // Identification is transactional: restore every active value even when
    // the candidate succeeded.
    encoder_.config_.direction = active_direction;
    encoder_.config_.phase_offset = active_phase_offset;
    encoder_.config_.phase_offset_float = active_phase_offset_float;
    encoder_.is_ready_ = active_encoder_ready;
    motor_.config_.phase_resistance = active_resistance;
    motor_.config_.phase_inductance = active_inductance;
    motor_.is_calibrated_ = active_motor_calibrated;
    motor_.update_current_controller_gains();
    return success;
}

bool Axis::run_calibration_flux_scan() {
    const uint32_t required =
        CalibrationPendingResult::VALID_PHASE_RESISTANCE |
        CalibrationPendingResult::VALID_PHASE_INDUCTANCE |
        CalibrationPendingResult::VALID_POLE_PAIRS;
    if ((calibration_pending_result_.validity & required) != required) {
        return false;
    }

    const float active_resistance = motor_.config_.phase_resistance;
    const float active_inductance = motor_.config_.phase_inductance;
    const bool active_calibrated = motor_.is_calibrated_;
    const int32_t active_pole_pairs = motor_.config_.pole_pairs;
    motor_.config_.phase_resistance =
        calibration_pending_result_.phase_resistance;
    motor_.config_.phase_inductance =
        calibration_pending_result_.phase_inductance;
    motor_.config_.pole_pairs = calibration_pending_result_.pole_pairs;
    motor_.is_calibrated_ = true;
    motor_.update_current_controller_gains();

    bool success = calibration_flux_fitter_.init(
        calibration_pending_result_.phase_resistance,
        calibration_pending_result_.phase_inductance,
        motor_.config_.pole_pairs);
    if (encoder_.mode_ == Encoder::MODE_SPI_ABS_MT6826S_VERNIER) {
        encoder_.reset_vernier_calibration();
    }

    // This excitation is expressed entirely in electrical radians. It does not
    // use the reducer ratio, Vernier branch, output position, or geometry LUT.
    // Ten electrical revolutions per direction provide ample steady samples
    // while moving the rotor by less than one mechanical revolution for LZ5710.
    constexpr float kElectricalVelocityRadPerSecond = 40.0f;
    constexpr float kElectricalDistanceRad = 10.0f * 2.0f * M_PI;
    constexpr uint32_t kSettlingSamples = 100u;

    for (uint32_t segment = 0; success && segment < 2u; ++segment) {
        const int direction = segment == 0u ? 1 : -1;
        LockinConfig_t scan = default_calibration();
        scan.current = motor_.config_.calibration_current;
        scan.ramp_time = 0.5f;
        scan.accel = 2.0f * kElectricalVelocityRadPerSecond;
        scan.vel = direction * kElectricalVelocityRadPerSecond;
        scan.finish_distance = direction * kElectricalDistanceRad;
        scan.finish_on_vel = false;
        scan.finish_on_distance = true;

        uint32_t steady_samples = 0u;
        const uint32_t vernier_target_points = (segment + 1u) * 8u;
        const uint32_t progress_start = 250u + segment * 300u;
        success = run_lockin_spin(scan, false, [&](bool reached_target_vel) {
            if (reached_target_vel) {
                if (++steady_samples > kSettlingSamples) {
                    CalibrationSampleV1 sample = {};
                    // Flux fitting needs dq current, applied voltage and the
                    // commanded electrical velocity only. Encoder validity is
                    // recorded for diagnostics but does not gate this fit.
                    capture_calibration_full_sample(&sample);
                    calibration_flux_fitter_.add_sample(sample);
                    if (encoder_.mode_ ==
                            Encoder::MODE_SPI_ABS_MT6826S_VERNIER &&
                        encoder_.get_vernier_calibration_point_count() <
                            vernier_target_points) {
                        // 1/64 sensor revolution prevents a fast loop from
                        // filling the fit with nearly identical observations.
                        // Each scan direction contributes up to eight points.
                        encoder_.capture_vernier_calibration_point(
                            static_cast<uint16_t>(
                                std::max<int32_t>(
                                    1, encoder_.config_.cpr / 64)));
                    }
                }
            } else {
                steady_samples = 0u;
            }

            const float traveled = std::abs(
                open_loop_controller_.total_distance_.any().value_or(0.0f));
            const float fraction = std::clamp(
                traveled / kElectricalDistanceRad, 0.0f, 1.0f);
            calibration_session_.set_progress(
                progress_start + static_cast<uint32_t>(300.0f * fraction));
            return calibration_session_.active();
        });
        if (success) {
            osDelay(250);
        }
    }

    CalibrationFluxFitter::Result flux = {};
    success = success && calibration_flux_fitter_.finish(&flux);
    calibration_pending_result_.flux_linkage = flux.flux_linkage;
    calibration_pending_result_.torque_constant = flux.torque_constant;
    calibration_pending_result_.flux_sample_stddev = flux.sample_stddev;
    calibration_pending_result_.flux_mean_std_error = flux.mean_std_error;
    calibration_pending_result_.flux_used_samples = flux.used_samples;
    if (success) {
        calibration_pending_result_.validity |=
            CalibrationPendingResult::VALID_FLUX_LINKAGE;
    } else if (calibration_session_.active()) {
        uint32_t failure =
            CalibrationSession::FAILURE_FLUX_NONPHYSICAL_MEAN;
        if (calibration_flux_fitter_.failure_reason() ==
            CalibrationFluxFitter::FAILURE_INSUFFICIENT_BLOCKS) {
            failure =
                CalibrationSession::FAILURE_FLUX_INSUFFICIENT_SAMPLES;
        } else if (calibration_flux_fitter_.failure_reason() ==
                   CalibrationFluxFitter::FAILURE_EXCESSIVE_DISPERSION) {
            failure =
                CalibrationSession::FAILURE_FLUX_EXCESSIVE_DISPERSION;
        }
        calibration_session_.fail(failure);
    }

    if (encoder_.mode_ == Encoder::MODE_SPI_ABS_MT6826S_VERNIER) {
        const bool vernier_success = finalize_vernier_offset_candidate();
        success = success && vernier_success;
    }

    motor_.disarm();
    motor_.config_.phase_resistance = active_resistance;
    motor_.config_.phase_inductance = active_inductance;
    motor_.config_.pole_pairs = active_pole_pairs;
    motor_.is_calibrated_ = active_calibrated;
    motor_.update_current_controller_gains();
    return success;
}

bool Axis::finalize_vernier_offset_candidate() {
    constexpr uint32_t kMinimumPoints = 12u;
    const uint32_t point_count =
        encoder_.get_vernier_calibration_point_count();
    calibration_pending_result_.vernier_used_samples = point_count;
    if (point_count < kMinimumPoints) {
        if (calibration_session_.active()) {
            calibration_session_.fail(
                CalibrationSession::FAILURE_VERNIER_INSUFFICIENT_SAMPLES);
        }
        return false;
    }

    // 0.05 sensor turns exceeds one 2*pi/21 equivalence interval, so a
    // first-time unit with arbitrary assembly phase always has a canonical
    // equivalent solution near the current persisted value.
    if (!encoder_.fit_vernier_aux_offset(0.05f * 2.0f * M_PI)) {
        if (calibration_session_.active()) {
            calibration_session_.fail(
                CalibrationSession::FAILURE_VERNIER_FIT);
        }
        return false;
    }

    auto& result = calibration_pending_result_;
    result.vernier_main_offset_rad =
        encoder_.get_vernier_calibration_fitted_main_offset();
    result.vernier_aux_offset_rad =
        encoder_.get_vernier_calibration_fitted_aux_offset();
    result.vernier_fit_rms_rad =
        encoder_.get_vernier_calibration_fit_score();
    result.vernier_worst_residual_rad =
        encoder_.get_vernier_calibration_worst_residual();
    result.vernier_minimum_margin_rad =
        encoder_.get_vernier_calibration_minimum_margin();

    if (!std::isfinite(result.vernier_fit_rms_rad) ||
        !std::isfinite(result.vernier_worst_residual_rad) ||
        result.vernier_worst_residual_rad >
            encoder_.effective_vernier_residual_accept_rad()) {
        if (calibration_session_.active()) {
            calibration_session_.fail(
                CalibrationSession::FAILURE_VERNIER_RESIDUAL);
        }
        return false;
    }
    if (!std::isfinite(result.vernier_minimum_margin_rad) ||
        result.vernier_minimum_margin_rad < 0.04f) {
        if (calibration_session_.active()) {
            calibration_session_.fail(
                CalibrationSession::FAILURE_VERNIER_AMBIGUOUS);
        }
        return false;
    }

    result.validity |= CalibrationPendingResult::VALID_VERNIER_OFFSETS;
    return true;
}

bool Axis::run_calibration_geometry_scan() {
    const uint32_t required =
        CalibrationPendingResult::VALID_PHASE_RESISTANCE |
        CalibrationPendingResult::VALID_PHASE_INDUCTANCE |
        CalibrationPendingResult::VALID_POLE_PAIRS;
    if ((calibration_pending_result_.validity & required) != required) {
        return false;
    }

    const float active_resistance = motor_.config_.phase_resistance;
    const float active_inductance = motor_.config_.phase_inductance;
    const bool active_calibrated = motor_.is_calibrated_;
    const int32_t active_pole_pairs = motor_.config_.pole_pairs;
    motor_.config_.phase_resistance =
        calibration_pending_result_.phase_resistance;
    motor_.config_.phase_inductance =
        calibration_pending_result_.phase_inductance;
    motor_.config_.pole_pairs = calibration_pending_result_.pole_pairs;
    motor_.is_calibrated_ = true;
    motor_.update_current_controller_gains();

    const uint32_t turns_option =
        (calibration_session_.request_options() >> 8) & 0xFFu;
    const float output_turns = static_cast<float>(
        std::clamp<uint32_t>(turns_option == 0 ? 2u : turns_option, 1u, 8u));
    const float ratio = std::max(
        std::abs(encoder_.config_.vernier_main_ratio), 1.0f);
    const float pole_pairs = static_cast<float>(
        std::max<int32_t>(motor_.config_.pole_pairs, 1));
    const float electrical_distance =
        output_turns * ratio * pole_pairs * 2.0f * M_PI;
    constexpr float kOutputSpeedTurnsPerSecond = 0.05f;
    const float electrical_velocity =
        kOutputSpeedTurnsPerSecond * ratio * pole_pairs * 2.0f * M_PI;

    CalibrationGeometryFitter::Config fitter_config = {};
    fitter_config.pole_pairs = motor_.config_.pole_pairs;
    fitter_config.main_ratio = encoder_.config_.vernier_main_ratio;
    fitter_config.aux_ratio = encoder_.config_.vernier_aux_ratio;
    fitter_config.main_offset = encoder_.config_.vernier_main_offset;
    fitter_config.aux_offset = encoder_.config_.vernier_aux_offset;
    fitter_config.main_reversed = encoder_.config_.vernier_main_reversed;
    fitter_config.aux_reversed = encoder_.config_.vernier_aux_reversed;
    fitter_config.output_reversed = encoder_.config_.vernier_output_reversed;
    fitter_config.cpu_hz = static_cast<float>(SystemCoreClock);

    bool success = calibration_geometry_fitter_.init(fitter_config) &&
        calibration_flux_fitter_.init(
            calibration_pending_result_.phase_resistance,
            calibration_pending_result_.phase_inductance,
            motor_.config_.pole_pairs);
    uint32_t segment_index = 0;
    for (uint32_t pass = 0; success && pass < 2; ++pass) {
        for (int direction : {1, -1}) {
            if (!calibration_session_.active()) {
                success = false;
                break;
            }
            success = pass == 0
                ? calibration_geometry_fitter_.begin_scale_segment(direction)
                : calibration_geometry_fitter_.begin_lut_segment(direction);
            if (!success) break;

            LockinConfig_t scan = default_calibration();
            scan.current = motor_.config_.calibration_current;
            scan.ramp_time = 0.5f;
            scan.accel = electrical_velocity * 2.0f;
            scan.vel = direction * electrical_velocity;
            scan.finish_distance = direction * electrical_distance;
            scan.finish_on_vel = false;
            scan.finish_on_distance = true;

            const uint32_t progress_start = 250u + segment_index * 100u;
            uint32_t steady_samples = 0;
            success = run_lockin_spin(scan, false, [&](bool reached_target_vel) {
                CalibrationSampleV1 sample = {};
                if (capture_calibration_full_sample(&sample)) {
                    calibration_geometry_fitter_.add_sample(sample);
                    // Flux is sensitive to dq transients during acceleration.
                    // Admit only steady-speed samples after an extra 100 ms
                    // current-loop settling window. Geometry can safely use
                    // the complete ramp because its regression is centered.
                    if (reached_target_vel) {
                        if (++steady_samples > 100u) {
                            calibration_flux_fitter_.add_sample(sample);
                        }
                    } else {
                        steady_samples = 0;
                    }
                }
                const float traveled = std::abs(
                    open_loop_controller_.total_distance_.any().value_or(0.0f));
                const float fraction = std::clamp(
                    traveled / electrical_distance, 0.0f, 1.0f);
                calibration_session_.set_progress(
                    progress_start + static_cast<uint32_t>(100.0f * fraction));
                return calibration_session_.active();
            });
            success = success && calibration_geometry_fitter_.finish_segment();
            ++segment_index;
            if (!success) break;
            osDelay(250);
        }
        if (success && pass == 0) {
            success = calibration_geometry_fitter_.finish_scale_pass();
        }
    }

    CalibrationGeometryFitter::Result geometry_result = {};
    CalibrationFluxFitter::Result flux_result = {};
    const bool motion_success = success;
    const bool geometry_success = motion_success &&
        calibration_geometry_fitter_.finish(&geometry_result);
    if (geometry_success) {
        calibration_pending_result_.effective_ratio_scale =
            geometry_result.effective_ratio_scale;
        std::copy(geometry_result.common_correction.begin(),
                  geometry_result.common_correction.end(),
                  calibration_pending_result_.common_geometry_correction.begin());
        std::copy(geometry_result.direction_correction.begin(),
                  geometry_result.direction_correction.end(),
                  calibration_pending_result_.direction_geometry_correction.begin());
        calibration_pending_result_.geometry_raw_rms =
            geometry_result.raw_rms_turns;
        calibration_pending_result_.geometry_corrected_rms =
            geometry_result.corrected_rms_turns;
        calibration_pending_result_.geometry_direction_peak_to_peak =
            geometry_result.direction_peak_to_peak_turns;
        calibration_pending_result_.geometry_used_samples =
            geometry_result.used_samples;
        calibration_pending_result_.validity |=
            CalibrationPendingResult::VALID_GEOMETRY_MODEL;
    }

    const bool flux_success = motion_success &&
        calibration_flux_fitter_.finish(&flux_result);
    if (motion_success) {
        // Publish quality diagnostics even when the validity bit remains clear.
        calibration_pending_result_.flux_linkage = flux_result.flux_linkage;
        calibration_pending_result_.torque_constant = flux_result.torque_constant;
        calibration_pending_result_.flux_sample_stddev = flux_result.sample_stddev;
        calibration_pending_result_.flux_mean_std_error =
            flux_result.mean_std_error;
        calibration_pending_result_.flux_used_samples = flux_result.used_samples;
    }
    if (flux_success) {
        calibration_pending_result_.validity |=
            CalibrationPendingResult::VALID_FLUX_LINKAGE;
    }

    success = geometry_success && flux_success;
    if (geometry_success && !flux_success && calibration_session_.active()) {
        // Do not collapse a flux-quality rejection into the generic geometry
        // failure reported by the caller. The valid geometry candidate remains
        // visible for diagnosis, but nothing is committed.
        uint32_t failure = CalibrationSession::FAILURE_FLUX_NONPHYSICAL_MEAN;
        if (calibration_flux_fitter_.failure_reason() ==
            CalibrationFluxFitter::FAILURE_INSUFFICIENT_BLOCKS) {
            failure = CalibrationSession::FAILURE_FLUX_INSUFFICIENT_SAMPLES;
        } else if (calibration_flux_fitter_.failure_reason() ==
                   CalibrationFluxFitter::FAILURE_EXCESSIVE_DISPERSION) {
            failure = CalibrationSession::FAILURE_FLUX_EXCESSIVE_DISPERSION;
        }
        calibration_session_.fail(failure);
    }

    motor_.disarm();
    motor_.config_.phase_resistance = active_resistance;
    motor_.config_.phase_inductance = active_inductance;
    motor_.config_.pole_pairs = active_pole_pairs;
    motor_.is_calibrated_ = active_calibrated;
    motor_.update_current_controller_gains();
    return success;
}

bool Axis::run_calibration_mechanical_scan() {
    const uint32_t required =
        CalibrationPendingResult::VALID_PHASE_RESISTANCE |
        CalibrationPendingResult::VALID_PHASE_INDUCTANCE |
        CalibrationPendingResult::VALID_POLE_PAIRS |
        CalibrationPendingResult::VALID_GEOMETRY_MODEL |
        CalibrationPendingResult::VALID_FLUX_LINKAGE;
    if ((calibration_pending_result_.validity & required) != required) {
        return false;
    }

    const Motor::Config_t motor_snapshot = motor_.config_;
    const Encoder::Config_t encoder_snapshot = encoder_.config_;
    const Controller::Config_t controller_snapshot = controller_.config_;
    const bool motor_calibrated_snapshot = motor_.is_calibrated_;
    const bool encoder_ready_snapshot = encoder_.is_ready_;
    const float input_vel_snapshot = controller_.input_vel_;
    const float input_torque_snapshot = controller_.input_torque_;
    const auto& candidate = calibration_pending_result_;

    motor_.config_.phase_resistance = candidate.phase_resistance;
    motor_.config_.phase_inductance = candidate.phase_inductance;
    motor_.config_.pole_pairs = candidate.pole_pairs;
    motor_.config_.flux_linkage = candidate.flux_linkage;
    motor_.config_.torque_constant = candidate.torque_constant;
    motor_.is_calibrated_ = true;
    encoder_.config_.direction = candidate.encoder_direction;
    encoder_.config_.phase_offset = candidate.phase_offset;
    encoder_.config_.phase_offset_float = candidate.phase_offset_float;
    encoder_.config_.vernier_effective_ratio_scale =
        candidate.effective_ratio_scale;
    std::copy(candidate.common_geometry_correction.begin(),
              candidate.common_geometry_correction.end(),
              std::begin(encoder_.config_.vernier_common_correction));
    std::copy(candidate.direction_geometry_correction.begin(),
              candidate.direction_geometry_correction.end(),
              std::begin(encoder_.config_.vernier_direction_correction));
    encoder_.config_.vernier_geometry_correction_enabled = true;
    encoder_.is_ready_ = true;

    // Disable existing mechanical feed-forward while identifying it. The
    // velocity feedback loop still supplies the physical torque through Iq.
    controller_.config_.inertia = 0.0f;
    controller_.config_.friction_static_pos = 0.0f;
    controller_.config_.friction_static_neg = 0.0f;
    controller_.config_.friction_coulomb_pos = 0.0f;
    controller_.config_.friction_coulomb_neg = 0.0f;
    controller_.config_.friction_viscous_pos = 0.0f;
    controller_.config_.friction_viscous_neg = 0.0f;
    controller_.config_.enable_sta = false;
    controller_.config_.enable_friction_compensation = false;
    controller_.config_.control_mode = Controller::CONTROL_MODE_VELOCITY_CONTROL;
    controller_.config_.input_mode = Controller::INPUT_MODE_VEL_RAMP;
    // The production low-speed gains may intentionally be soft and unable to
    // break gearbox stiction during identification. These temporary output-
    // shaft gains provide enough excitation without changing current limits.
    controller_.config_.vel_gain = std::max(controller_.config_.vel_gain, 5.0f);
    controller_.config_.vel_integrator_gain =
        std::max(controller_.config_.vel_integrator_gain, 20.0f);
    controller_.config_.velocity_accel_limit = 0.20f;
    controller_.config_.velocity_decel_limit = 0.20f;
    controller_.config_.vel_limit = std::max(controller_.config_.vel_limit, 0.15f);
    controller_.input_vel_ = 0.0f;
    controller_.input_torque_ = 0.0f;
    motor_.update_current_controller_gains();
    encoder_.reset_vernier_output_velocity_estimate();

    const bool fitter_initialized = calibration_mechanical_fitter_.init(
        candidate.torque_constant,
        encoder_.config_.vernier_main_ratio,
        1.0f, static_cast<float>(SystemCoreClock),
        encoder_.controller_to_motor_direction());
    const bool closed_loop_started = fitter_initialized &&
        start_closed_loop_control() && motor_.is_armed_;
    bool success = closed_loop_started;
    bool motion_fault = false;
    if (!closed_loop_started && calibration_session_.active()) {
        calibration_session_.fail(
            CalibrationSession::FAILURE_MECHANICAL_CLOSED_LOOP_START);
    }

    constexpr std::array<float, 3> kOutputSpeeds = {0.02f, 0.05f, 0.10f};
    uint32_t segment_index = 0;

    for (float output_speed : kOutputSpeeds) {
        for (int direction : {1, -1}) {
            if (!success || !calibration_session_.active() || !motor_.is_armed_) {
                if (calibration_session_.active() && !motor_.is_armed_) {
                    motion_fault = true;
                }
                success = false;
                break;
            }
            calibration_mechanical_fitter_.begin_segment();
            const uint32_t progress_start = 650u + segment_index * 33u;
            const float target = direction * output_speed;
            controller_.input_vel_ = target;
            // Identification consumes measured velocity/acceleration/Iq, so
            // exact setpoint tracking is not a prerequisite. A fixed profile
            // also admits stick-slip behavior instead of timing it out.
            constexpr uint32_t kDriveDurationMs = 2500;
            for (uint32_t elapsed_ms = 0; elapsed_ms < kDriveDurationMs; ++elapsed_ms) {
                CalibrationSampleV1 sample = {};
                if (capture_calibration_full_sample(&sample)) {
                    calibration_mechanical_fitter_.add_sample(sample);
                }
                float velocity = encoder_.vel_estimate_.any().value_or(0.0f);
                if (encoder_.mode_ ==
                    Encoder::MODE_SPI_ABS_MT6826S_VERNIER) {
                    velocity /= 60.0f;
                }
                if (!std::isfinite(velocity) || std::abs(velocity) > 0.20f) {
                    success = false;
                    motion_fault = true;
                    break;
                }
                calibration_session_.set_progress(
                    progress_start + elapsed_ms * 24u / kDriveDurationMs);
                if (!calibration_session_.active() || !motor_.is_armed_) {
                    if (calibration_session_.active() && !motor_.is_armed_) {
                        motion_fault = true;
                    }
                    success = false;
                    break;
                }
                osDelay(1);
            }

            // Include the opposite-sign acceleration while ramping back to
            // zero. This separates inertia from same-direction friction.
            controller_.input_vel_ = 0.0f;
            constexpr uint32_t kReturnDurationMs = 1500;
            for (uint32_t elapsed_ms = 0;
                 success && elapsed_ms < kReturnDurationMs; ++elapsed_ms) {
                CalibrationSampleV1 sample = {};
                if (capture_calibration_full_sample(&sample)) {
                    calibration_mechanical_fitter_.add_sample(sample);
                }
                float velocity = encoder_.vel_estimate_.any().value_or(0.0f);
                if (encoder_.mode_ ==
                    Encoder::MODE_SPI_ABS_MT6826S_VERNIER) {
                    velocity /= 60.0f;
                }
                if (!std::isfinite(velocity) || std::abs(velocity) > 0.20f) {
                    success = false;
                    motion_fault = true;
                    break;
                }
                calibration_session_.set_progress(
                    progress_start + 24u + elapsed_ms * 9u / kReturnDurationMs);
                if (!calibration_session_.active() || !motor_.is_armed_) {
                    if (calibration_session_.active() && !motor_.is_armed_) {
                        motion_fault = true;
                    }
                    success = false;
                    break;
                }
                osDelay(1);
            }
            calibration_mechanical_fitter_.finish_segment();
            ++segment_index;
            if (!success) break;
        }
    }

    controller_.input_vel_ = 0.0f;
    stop_closed_loop_control();

    calibration_pending_result_.mechanical_attempted_samples =
        calibration_mechanical_fitter_.attempted_samples();
    calibration_pending_result_.mechanical_rejected_invalid =
        calibration_mechanical_fitter_.rejected_invalid();
    calibration_pending_result_.mechanical_rejected_saturated =
        calibration_mechanical_fitter_.rejected_saturated();
    calibration_pending_result_.mechanical_rejected_low_velocity =
        calibration_mechanical_fitter_.rejected_low_velocity();
    calibration_pending_result_.mechanical_max_abs_velocity =
        calibration_mechanical_fitter_.max_abs_velocity();
    calibration_pending_result_.mechanical_rejected_timing =
        calibration_mechanical_fitter_.rejected_timing();

    CalibrationMechanicalFitter::Result mechanical = {};
    if (success) {
        success = calibration_mechanical_fitter_.finish(&mechanical);
    } else {
        mechanical.used_samples = calibration_mechanical_fitter_.sample_count();
    }
    if (mechanical.used_samples > 0u) {
        // Preserve invalid fit diagnostics for UI display without setting the
        // mechanical validity bit or activating any compensation.
        calibration_pending_result_.output_inertia = mechanical.output_inertia;
        calibration_pending_result_.friction_coulomb_pos = mechanical.coulomb_pos;
        calibration_pending_result_.friction_coulomb_neg = mechanical.coulomb_neg;
        calibration_pending_result_.friction_viscous_pos = mechanical.viscous_pos;
        calibration_pending_result_.friction_viscous_neg = mechanical.viscous_neg;
        calibration_pending_result_.mechanical_residual_rms_torque =
            mechanical.residual_rms_torque;
        calibration_pending_result_.mechanical_used_samples =
            mechanical.used_samples;
    }
    if (success) {
        calibration_pending_result_.validity |=
            CalibrationPendingResult::VALID_MECHANICAL_MODEL;
    } else if (calibration_session_.active()) {
        uint32_t failure =
            motion_fault
                ? CalibrationSession::FAILURE_MECHANICAL_TRACKING_TIMEOUT
                : CalibrationSession::FAILURE_MECHANICAL_INSUFFICIENT_EXCITATION;
        const uint32_t attempted = calibration_mechanical_fitter_.attempted_samples();
        if (!motion_fault && attempted > 0u &&
            calibration_mechanical_fitter_.rejected_invalid() == attempted) {
            failure = CalibrationSession::FAILURE_MECHANICAL_ALL_INVALID;
        } else if (!motion_fault && attempted > 0u &&
                   calibration_mechanical_fitter_.rejected_saturated() == attempted) {
            failure = CalibrationSession::FAILURE_MECHANICAL_ALL_SATURATED;
        } else if (!motion_fault && attempted > 0u &&
                   calibration_mechanical_fitter_.sample_count() == 0u &&
                   calibration_mechanical_fitter_.rejected_timing() >= attempted - 6u) {
            failure = CalibrationSession::FAILURE_MECHANICAL_ALL_TIMING_INVALID;
        } else if (!motion_fault && attempted > 0u &&
                   calibration_mechanical_fitter_.sample_count() == 0u &&
                   calibration_mechanical_fitter_.max_abs_velocity() < 0.005f) {
            failure = CalibrationSession::FAILURE_MECHANICAL_NO_MOTION;
        } else if (!motion_fault && calibration_mechanical_fitter_.failure_reason() ==
            CalibrationMechanicalFitter::FAILURE_SINGULAR_REGRESSION) {
            failure = CalibrationSession::FAILURE_MECHANICAL_SINGULAR_REGRESSION;
        } else if (!motion_fault && calibration_mechanical_fitter_.failure_reason() ==
                   CalibrationMechanicalFitter::FAILURE_NONPHYSICAL_PARAMETERS) {
            failure = CalibrationSession::FAILURE_MECHANICAL_NONPHYSICAL_PARAMETERS;
        }
        calibration_session_.fail(failure);
    }

    motor_.config_ = motor_snapshot;
    encoder_.config_ = encoder_snapshot;
    controller_.config_ = controller_snapshot;
    motor_.is_calibrated_ = motor_calibrated_snapshot;
    encoder_.is_ready_ = encoder_ready_snapshot;
    motor_.config_.parent = &motor_;
    encoder_.config_.parent = &encoder_;
    controller_.config_.parent = &controller_;
    controller_.input_vel_ = input_vel_snapshot;
    controller_.input_torque_ = input_torque_snapshot;
    motor_.update_current_controller_gains();
    encoder_.reset_vernier_output_velocity_estimate();
    return success;
}

bool Axis::run_calibration_delay_scan() {
    const uint32_t required =
        CalibrationPendingResult::VALID_PHASE_RESISTANCE |
        CalibrationPendingResult::VALID_PHASE_INDUCTANCE |
        CalibrationPendingResult::VALID_ENCODER_DIRECTION |
        CalibrationPendingResult::VALID_ELECTRICAL_OFFSET |
        CalibrationPendingResult::VALID_POLE_PAIRS |
        CalibrationPendingResult::VALID_GEOMETRY_MODEL |
        CalibrationPendingResult::VALID_FLUX_LINKAGE |
        CalibrationPendingResult::VALID_MECHANICAL_MODEL;
    if ((calibration_pending_result_.validity & required) != required) {
        return false;
    }

    const Motor::Config_t motor_snapshot = motor_.config_;
    const Encoder::Config_t encoder_snapshot = encoder_.config_;
    const Controller::Config_t controller_snapshot = controller_.config_;
    const bool motor_calibrated_snapshot = motor_.is_calibrated_;
    const bool encoder_ready_snapshot = encoder_.is_ready_;
    const float input_vel_snapshot = controller_.input_vel_;
    const float input_torque_snapshot = controller_.input_torque_;
    const auto& candidate = calibration_pending_result_;

    motor_.config_.phase_resistance = candidate.phase_resistance;
    motor_.config_.phase_inductance = candidate.phase_inductance;
    motor_.config_.pole_pairs = candidate.pole_pairs;
    motor_.config_.flux_linkage = candidate.flux_linkage;
    motor_.config_.torque_constant = candidate.torque_constant;
    motor_.is_calibrated_ = true;
    encoder_.config_.direction = candidate.encoder_direction;
    encoder_.config_.phase_offset = candidate.phase_offset;
    encoder_.config_.phase_offset_float = candidate.phase_offset_float;
    encoder_.config_.electrical_phase_delay = 0.0f;
    encoder_.config_.vernier_effective_ratio_scale =
        candidate.effective_ratio_scale;
    std::copy(candidate.common_geometry_correction.begin(),
              candidate.common_geometry_correction.end(),
              std::begin(encoder_.config_.vernier_common_correction));
    std::copy(candidate.direction_geometry_correction.begin(),
              candidate.direction_geometry_correction.end(),
              std::begin(encoder_.config_.vernier_direction_correction));
    encoder_.config_.vernier_geometry_correction_enabled = true;
    encoder_.is_ready_ = true;
    controller_.config_.inertia = candidate.output_inertia;
    controller_.config_.friction_static_pos = candidate.friction_coulomb_pos;
    controller_.config_.friction_static_neg = candidate.friction_coulomb_neg;
    controller_.config_.friction_coulomb_pos = candidate.friction_coulomb_pos;
    controller_.config_.friction_coulomb_neg = candidate.friction_coulomb_neg;
    controller_.config_.friction_viscous_pos = candidate.friction_viscous_pos;
    controller_.config_.friction_viscous_neg = candidate.friction_viscous_neg;
    controller_.config_.enable_sta = false;
    controller_.config_.control_mode = Controller::CONTROL_MODE_VELOCITY_CONTROL;
    controller_.config_.input_mode = Controller::INPUT_MODE_VEL_RAMP;
    controller_.config_.vel_gain = std::max(controller_.config_.vel_gain, 5.0f);
    controller_.config_.vel_integrator_gain =
        std::max(controller_.config_.vel_integrator_gain, 20.0f);
    controller_.config_.velocity_accel_limit = 0.20f;
    controller_.config_.velocity_decel_limit = 0.20f;
    controller_.config_.vel_limit = std::max(controller_.config_.vel_limit, 0.20f);
    controller_.input_vel_ = 0.0f;
    controller_.input_torque_ = 0.0f;
    motor_.update_current_controller_gains();
    encoder_.reset_vernier_output_velocity_estimate();

    const bool fitter_initialized = calibration_delay_fitter_.init(
        candidate.phase_resistance, candidate.phase_inductance);
    const bool closed_loop_started = fitter_initialized &&
        start_closed_loop_control() && motor_.is_armed_;
    bool success = closed_loop_started;
    if (!closed_loop_started && calibration_session_.active()) {
        calibration_session_.fail(
            CalibrationSession::FAILURE_DELAY_CLOSED_LOOP_START);
    }
    constexpr std::array<float, 3> kOutputSpeeds = {0.04f, 0.08f, 0.12f};
    uint32_t segment_index = 0;
    for (int direction : {1, -1}) {
        for (float speed : kOutputSpeeds) {
            if (!success || !calibration_session_.active() || !motor_.is_armed_) {
                success = false;
                break;
            }
            const float target = direction * speed;
            controller_.input_vel_ = target;
            constexpr uint32_t kSettleMs = 1500;
            for (uint32_t elapsed_ms = 0; elapsed_ms < kSettleMs; ++elapsed_ms) {
                if (!calibration_session_.active() || !motor_.is_armed_) {
                    success = false;
                    break;
                }
                float velocity =
                    encoder_.vel_estimate_.any().value_or(0.0f);
                if (encoder_.mode_ ==
                    Encoder::MODE_SPI_ABS_MT6826S_VERNIER) {
                    velocity /= 60.0f;
                }
                if (!std::isfinite(velocity) || std::abs(velocity) > 0.20f) {
                    success = false;
                    break;
                }
                calibration_session_.set_progress(
                    850u + segment_index * 15u + elapsed_ms * 4u / kSettleMs);
                osDelay(1);
            }
            constexpr uint32_t kSampleMs = 1500;
            calibration_delay_fitter_.begin_segment();
            for (uint32_t sample_ms = 0; success && sample_ms < kSampleMs; ++sample_ms) {
                CalibrationSampleV1 sample = {};
                if (capture_calibration_full_sample(&sample)) {
                    calibration_delay_fitter_.add_sample(sample);
                }
                calibration_session_.set_progress(
                    854u + segment_index * 15u + sample_ms * 11u / kSampleMs);
                if (!calibration_session_.active() || !motor_.is_armed_) {
                    success = false;
                    break;
                }
                osDelay(1);
            }
            calibration_delay_fitter_.finish_segment();
            ++segment_index;
            if (!success) break;
        }
    }

    controller_.input_vel_ = 0.0f;
    for (uint32_t i = 0; i < 1500 && motor_.is_armed_; ++i) osDelay(1);
    stop_closed_loop_control();

    CalibrationDelayFitter::Result delay = {};
    if (success) success = calibration_delay_fitter_.finish(&delay);
    calibration_pending_result_.electrical_delay = delay.electrical_delay;
    calibration_pending_result_.delay_residual_phase_offset =
        delay.residual_phase_offset;
    calibration_pending_result_.delay_residual_rms = delay.residual_rms;
    calibration_pending_result_.delay_used_samples = delay.used_samples;
    calibration_pending_result_.delay_attempted_samples =
        calibration_delay_fitter_.attempted_count();
    calibration_pending_result_.delay_rejected_invalid =
        calibration_delay_fitter_.rejected_invalid();
    calibration_pending_result_.delay_rejected_saturated =
        calibration_delay_fitter_.rejected_saturated();
    calibration_pending_result_.delay_rejected_speed =
        calibration_delay_fitter_.rejected_speed();
    calibration_pending_result_.delay_rejected_emf =
        calibration_delay_fitter_.rejected_emf();
    calibration_pending_result_.delay_rejected_phase =
        calibration_delay_fitter_.rejected_phase();
    calibration_pending_result_.delay_max_abs_electrical_speed =
        calibration_delay_fitter_.max_abs_electrical_speed();
    if (success) {
        calibration_pending_result_.validity |=
            CalibrationPendingResult::VALID_ELECTRICAL_DELAY;
    } else if (calibration_session_.active()) {
        uint32_t failure = CalibrationSession::FAILURE_DELAY_INSUFFICIENT_SAMPLES;
        if (calibration_delay_fitter_.failure_reason() ==
            CalibrationDelayFitter::FAILURE_UNOBSERVABLE_SPEED) {
            failure = CalibrationSession::FAILURE_DELAY_UNOBSERVABLE_SPEED;
        } else if (calibration_delay_fitter_.failure_reason() ==
                   CalibrationDelayFitter::FAILURE_NONPHYSICAL_RESULT) {
            failure = CalibrationSession::FAILURE_DELAY_NONPHYSICAL_RESULT;
        }
        calibration_session_.fail(failure);
    }

    motor_.config_ = motor_snapshot;
    encoder_.config_ = encoder_snapshot;
    controller_.config_ = controller_snapshot;
    motor_.is_calibrated_ = motor_calibrated_snapshot;
    encoder_.is_ready_ = encoder_ready_snapshot;
    motor_.config_.parent = &motor_;
    encoder_.config_.parent = &encoder_;
    controller_.config_.parent = &controller_;
    controller_.input_vel_ = input_vel_snapshot;
    controller_.input_torque_ = input_torque_snapshot;
    motor_.update_current_controller_gains();
    encoder_.reset_vernier_output_velocity_estimate();
    return success;
}

bool Axis::validate_calibration_candidate() const {
    uint32_t required =
        CalibrationPendingResult::VALID_PHASE_RESISTANCE |
        CalibrationPendingResult::VALID_PHASE_INDUCTANCE |
        CalibrationPendingResult::VALID_ENCODER_DIRECTION |
        CalibrationPendingResult::VALID_ELECTRICAL_OFFSET |
        CalibrationPendingResult::VALID_FLUX_LINKAGE |
        CalibrationPendingResult::VALID_POLE_PAIRS;
    if (encoder_.mode_ == Encoder::MODE_SPI_ABS_MT6826S_VERNIER) {
        required |= CalibrationPendingResult::VALID_VERNIER_OFFSETS;
    }
    const auto& result = calibration_pending_result_;
    if ((result.validity & required) != required ||
        result.session_id != calibration_session_.session_id()) {
        return false;
    }
    if (!std::isfinite(result.phase_resistance) ||
        result.phase_resistance < 1.0e-4f || result.phase_resistance > 10.0f ||
        !std::isfinite(result.phase_inductance) ||
        result.phase_inductance < 1.0e-7f || result.phase_inductance > 1.0f ||
        (result.encoder_direction != 1 && result.encoder_direction != -1) ||
        !std::isfinite(result.phase_offset_float) ||
        std::abs(result.phase_offset_float) > 2.0f ||
        result.pole_pairs < 1 || result.pole_pairs > 128) {
        return false;
    }
    if (encoder_.mode_ == Encoder::MODE_SPI_ABS_MT6826S_VERNIER &&
        (!std::isfinite(result.vernier_main_offset_rad) ||
         !std::isfinite(result.vernier_aux_offset_rad) ||
         std::abs(result.vernier_main_offset_rad) > M_PI ||
         std::abs(result.vernier_aux_offset_rad) > M_PI ||
         !std::isfinite(result.vernier_fit_rms_rad) ||
         !std::isfinite(result.vernier_worst_residual_rad) ||
         result.vernier_fit_rms_rad < 0.0f ||
         result.vernier_worst_residual_rad < 0.0f ||
         result.vernier_worst_residual_rad >
             encoder_.effective_vernier_residual_accept_rad() ||
         !std::isfinite(result.vernier_minimum_margin_rad) ||
         result.vernier_minimum_margin_rad < 0.04f ||
         result.vernier_used_samples < 12u)) {
        return false;
    }
    if (result.validity & CalibrationPendingResult::VALID_GEOMETRY_MODEL) {
        if (!std::isfinite(result.effective_ratio_scale) ||
            result.effective_ratio_scale < 0.8f ||
            result.effective_ratio_scale > 1.2f ||
            !std::isfinite(result.geometry_raw_rms) ||
            !std::isfinite(result.geometry_corrected_rms) ||
            result.geometry_corrected_rms > result.geometry_raw_rms * 1.05f ||
            result.geometry_used_samples <
                4u * CalibrationGeometryFitter::kBins) {
            return false;
        }
        for (size_t i = 0; i < CalibrationGeometryFitter::kBins; ++i) {
            const float common = result.common_geometry_correction[i];
            const float directional = result.direction_geometry_correction[i];
            if (!std::isfinite(common) || !std::isfinite(directional) ||
                std::abs(common) > 0.05f || std::abs(directional) > 0.05f) {
                return false;
            }
            const size_t next = (i + 1u) % CalibrationGeometryFitter::kBins;
            const float common_slope =
                (result.common_geometry_correction[next] - common) *
                CalibrationGeometryFitter::kBins;
            const float directional_slope =
                (result.direction_geometry_correction[next] - directional) *
                CalibrationGeometryFitter::kBins;
            // Both directional branches must remain monotonic. This also
            // bounds the LUT-derived velocity correction used at runtime.
            const float forward_slope =
                1.0f + common_slope + directional_slope;
            const float reverse_slope =
                1.0f + common_slope - directional_slope;
            if (!std::isfinite(forward_slope) ||
                !std::isfinite(reverse_slope) ||
                forward_slope < 0.25f || forward_slope > 4.0f ||
                reverse_slope < 0.25f || reverse_slope > 4.0f) {
                return false;
            }
        }
    }
    if (result.validity & CalibrationPendingResult::VALID_FLUX_LINKAGE) {
        const float derived_kt =
            1.5f * result.pole_pairs * result.flux_linkage;
        if (!std::isfinite(result.flux_linkage) ||
            result.flux_linkage <= 1.0e-6f ||
            result.flux_linkage > 1.0f ||
            !std::isfinite(result.torque_constant) ||
            result.torque_constant <= 0.0f ||
            std::abs(result.torque_constant - derived_kt) /
                result.torque_constant > 1.0e-3f ||
            !std::isfinite(result.flux_sample_stddev) ||
            result.flux_sample_stddev > result.flux_linkage ||
            !std::isfinite(result.flux_mean_std_error) ||
            result.flux_mean_std_error > result.flux_linkage * 0.05f ||
            result.flux_used_samples < 256u) {
            return false;
        }
    }
    if (result.validity & CalibrationPendingResult::VALID_MECHANICAL_MODEL) {
        const float mean_coulomb = 0.5f *
            (result.friction_coulomb_pos + result.friction_coulomb_neg);
        const float max_coulomb = std::max(result.friction_coulomb_pos,
                                           result.friction_coulomb_neg);
        const float max_viscous = std::max(result.friction_viscous_pos,
                                          result.friction_viscous_neg);
        constexpr float kCalibrationMaxVelocity = 0.20f;
        constexpr float kCalibrationAcceleration = 0.20f;
        const float predicted_peak_torque =
            result.output_inertia * kCalibrationAcceleration + max_coulomb +
            max_viscous * kCalibrationMaxVelocity;
        const float available_output_torque = result.torque_constant *
            motor_.config_.current_lim *
            std::abs(encoder_.config_.vernier_main_ratio);
        if (!std::isfinite(result.output_inertia) ||
            result.output_inertia < 0.0f ||
            !std::isfinite(result.friction_coulomb_pos) ||
            !std::isfinite(result.friction_coulomb_neg) ||
            !std::isfinite(result.friction_viscous_pos) ||
            !std::isfinite(result.friction_viscous_neg) ||
            result.friction_coulomb_pos < 0.0f ||
            result.friction_coulomb_neg < 0.0f ||
            result.friction_viscous_pos < 0.0f ||
            result.friction_viscous_neg < 0.0f ||
            !std::isfinite(predicted_peak_torque) ||
            !std::isfinite(available_output_torque) ||
            available_output_torque <= 0.0f ||
            predicted_peak_torque > 0.8f * available_output_torque ||
            !std::isfinite(result.mechanical_residual_rms_torque) ||
            result.mechanical_residual_rms_torque >
                std::max(0.05f, mean_coulomb) ||
            result.mechanical_used_samples < 1000u) {
            return false;
        }
    }
    if (result.validity & CalibrationPendingResult::VALID_ELECTRICAL_DELAY) {
        if (!std::isfinite(result.electrical_delay) ||
            result.electrical_delay < 0.0f ||
            result.electrical_delay > 500.0e-6f ||
            !std::isfinite(result.delay_residual_phase_offset) ||
            std::abs(result.delay_residual_phase_offset) > 0.20f ||
            !std::isfinite(result.delay_residual_rms) ||
            result.delay_residual_rms > 0.10f ||
            result.delay_used_samples < 1000u) {
            return false;
        }
    }
    return true;
}

bool Axis::commit_calibration_candidate() {
    if (!validate_calibration_candidate() || motor_.is_armed_) {
        return false;
    }

    const Motor::Config_t motor_snapshot = motor_.config_;
    const Encoder::Config_t encoder_snapshot = encoder_.config_;
    const Controller::Config_t controller_snapshot = controller_.config_;
    const bool motor_calibrated_snapshot = motor_.is_calibrated_;
    const bool encoder_ready_snapshot = encoder_.is_ready_;
    const auto& result = calibration_pending_result_;

    CRITICAL_SECTION() {
        motor_.config_.phase_resistance = result.phase_resistance;
        motor_.config_.phase_inductance = result.phase_inductance;
        motor_.config_.pole_pairs = result.pole_pairs;
        if (result.validity & CalibrationPendingResult::VALID_FLUX_LINKAGE) {
            motor_.config_.flux_linkage = result.flux_linkage;
            motor_.config_.torque_constant = result.torque_constant;
        }
        motor_.config_.pre_calibrated = true;
        motor_.is_calibrated_ = true;

        encoder_.config_.direction = result.encoder_direction;
        encoder_.config_.phase_offset = result.phase_offset;
        encoder_.config_.phase_offset_float = result.phase_offset_float;
        if (result.validity &
            CalibrationPendingResult::VALID_VERNIER_OFFSETS) {
            encoder_.config_.vernier_main_offset =
                result.vernier_main_offset_rad;
            encoder_.config_.vernier_aux_offset =
                result.vernier_aux_offset_rad;
        }
        if (result.validity & CalibrationPendingResult::VALID_ELECTRICAL_DELAY) {
            encoder_.config_.electrical_phase_delay = result.electrical_delay;
        }
        if (result.validity & CalibrationPendingResult::VALID_GEOMETRY_MODEL) {
            encoder_.config_.vernier_effective_ratio_scale =
                result.effective_ratio_scale;
            std::copy(result.common_geometry_correction.begin(),
                      result.common_geometry_correction.end(),
                      std::begin(encoder_.config_.vernier_common_correction));
            std::copy(
                result.direction_geometry_correction.begin(),
                result.direction_geometry_correction.end(),
                std::begin(encoder_.config_.vernier_direction_correction));
            encoder_.config_.vernier_geometry_correction_enabled = true;
        }
        encoder_.config_.pre_calibrated = true;
        encoder_.is_ready_ =
            encoder_.mode_ != Encoder::MODE_SPI_ABS_MT6826S_VERNIER;

        if (result.validity & CalibrationPendingResult::VALID_MECHANICAL_MODEL) {
            controller_.config_.inertia = result.output_inertia;
            controller_.config_.friction_static_pos =
                result.friction_coulomb_pos;
            controller_.config_.friction_static_neg =
                result.friction_coulomb_neg;
            controller_.config_.friction_coulomb_pos =
                result.friction_coulomb_pos;
            controller_.config_.friction_coulomb_neg =
                result.friction_coulomb_neg;
            controller_.config_.friction_viscous_pos =
                result.friction_viscous_pos;
            controller_.config_.friction_viscous_neg =
                result.friction_viscous_neg;
            // Keep the robust PI speed loop after calibration. STA requires an
            // explicit runtime enable after its gains are identified.
            controller_.config_.enable_sta = false;
        }
    }

    motor_.update_current_controller_gains();
    if (encoder_.mode_ == Encoder::MODE_SPI_ABS_MT6826S_VERNIER) {
        encoder_.apply_vernier_resolver_config();
    }
    controller_.reset_sta();

    // Committing a validated calibration and admitting closed-loop control are
    // separate operations. Applying vernier parameters intentionally resets
    // the resolver, tracker and PLL, so requiring the complete feedback chain
    // to relock inside this storage transaction makes a valid calibration
    // depend on an arbitrary timeout. Persist the validated candidate here;
    // the normal closed-loop gate still requires controller_feedback_ready().

    if (odrv.save_configuration()) {
        return true;
    }

    CRITICAL_SECTION() {
        motor_.config_ = motor_snapshot;
        encoder_.config_ = encoder_snapshot;
        controller_.config_ = controller_snapshot;
        motor_.is_calibrated_ = motor_calibrated_snapshot;
        encoder_.is_ready_ = encoder_ready_snapshot;
        motor_.config_.parent = &motor_;
        encoder_.config_.parent = &encoder_;
        controller_.config_.parent = &controller_;
    }
    motor_.update_current_controller_gains();
    if (encoder_.mode_ == Encoder::MODE_SPI_ABS_MT6826S_VERNIER) {
        encoder_.apply_vernier_resolver_config();
    }
    controller_.reset_sta();
    return false;
}

void Axis::service_calibration_session_start() {
    if (!calibration_start_pending_) {
        return;
    }

    CRITICAL_SECTION() {
        calibration_start_pending_ = false;
        calibration_sample_sequence_ = 0;
        calibration_session_.set_stage(CalibrationSession::STAGE_PRECHECK);
        calibration_session_.set_progress(10);

        switch (calibration_session_.profile()) {
            case CalibrationSession::PROFILE_FULL:
            case CalibrationSession::PROFILE_ELECTRICAL:
                calibration_session_.set_stage(
                    CalibrationSession::STAGE_ELECTRICAL_CAPTURE);
                // The new full pipeline advances one owned stage at a time.
                // Do not invoke the legacy full sequence, which mutates active
                // encoder configuration before transactional validation.
                requested_state_ = AXIS_STATE_MOTOR_CALIBRATION;
                break;
            case CalibrationSession::PROFILE_MECHANICAL:
            case CalibrationSession::PROFILE_VALIDATE_ONLY:
                calibration_session_.fail(
                    CalibrationSession::FAILURE_PROFILE_NOT_IMPLEMENTED);
                break;
        }
        calibration_session_.set_progress(20);
    }
}

// @brief Do axis level checks and call subcomponent do_checks
// Returns true if everything is ok.
bool Axis::do_checks(uint32_t timestamp) {
    // Sub-components should use set_error which will propegate to this error_
    motor_.effective_current_lim();
    motor_.do_checks(timestamp);

    return check_for_errors();
}

// @brief Feed the watchdog to prevent watchdog timeouts.
void Axis::watchdog_feed() {
    watchdog_current_value_ = get_watchdog_reset();
}

// @brief Check the watchdog timer for expiration. Also sets the watchdog error bit if expired.
bool Axis::watchdog_check() {
    if (!config_.enable_watchdog) return true;

    // explicit check here to ensure that we don't underflow back to UINT32_MAX
    if (watchdog_current_value_ > 0) {
        watchdog_current_value_--;
        return true;
    } else {
        odrive::fault::FaultRecord record;
        record.control_sequence = odrv.n_evt_control_loop_;
        record.timestamp_cycles = DWT->CYCCNT;
        record.source = odrive::fault::FaultSource::AXIS;
        record.code = odrive::fault::FaultCode::TIMEOUT;
        record.site = odrive::fault::FaultSite::AXIS_WATCHDOG;
        record.severity = odrive::fault::FaultSeverity::LATCHED;
        raise_fault(record, ERROR_WATCHDOG_TIMER_EXPIRED);
        return false;
    }
}

bool Axis::run_lockin_spin(const LockinConfig_t &lockin_config, bool remain_armed,
        std::function<bool(bool)> loop_cb) {
    CRITICAL_SECTION() {
        // Reset state variables
        open_loop_controller_.Idq_setpoint_ = {0.0f, 0.0f};
        open_loop_controller_.Vdq_setpoint_ = {0.0f, 0.0f};
        open_loop_controller_.phase_ = 0.0f;
        open_loop_controller_.phase_vel_ = 0.0f;

        open_loop_controller_.max_current_ramp_ = lockin_config.current / lockin_config.ramp_time;
        open_loop_controller_.max_voltage_ramp_ = lockin_config.current / lockin_config.ramp_time;
        open_loop_controller_.max_phase_vel_ramp_ = lockin_config.accel;
        open_loop_controller_.target_current_ = lockin_config.current;
        open_loop_controller_.target_voltage_ = 0.0f;
        open_loop_controller_.target_vel_ = lockin_config.vel;
        open_loop_controller_.total_distance_ = 0.0f;

        motor_.current_control_.enable_current_control_src_ = true;
        motor_.current_control_.Idq_setpoint_src_.connect_to(&open_loop_controller_.Idq_setpoint_);
        motor_.current_control_.Vdq_setpoint_src_.connect_to(&open_loop_controller_.Vdq_setpoint_);

        motor_.current_control_.phase_src_.connect_to(&open_loop_controller_.phase_);
        
        motor_.phase_vel_src_.connect_to(&open_loop_controller_.phase_vel_);
        motor_.current_control_.phase_vel_src_.connect_to(&open_loop_controller_.phase_vel_);
    }
    wait_for_control_iteration();

    motor_.arm(&motor_.current_control_);

    bool success = false;
    float dir = lockin_config.vel >= 0.0f ? 1.0f : -1.0f;

    while ((requested_state_ == AXIS_STATE_UNDEFINED) && motor_.is_armed_) {
        bool reached_target_vel = std::abs(open_loop_controller_.phase_vel_.any().value_or(0.0f) - lockin_config.vel) <= std::numeric_limits<float>::epsilon();
        bool reached_target_dist = open_loop_controller_.total_distance_.any().value_or(0.0f) * dir >= lockin_config.finish_distance * dir;

        // Check if terminal condition is reached
        bool terminal_condition = (reached_target_vel && lockin_config.finish_on_vel)
                               || (reached_target_dist && lockin_config.finish_on_distance);
        if (terminal_condition) {
            success = true;
            break;
        }

        if (loop_cb)
            if (!loop_cb(reached_target_vel))
                break;

        // TODO: use new sync function instead
        asm volatile ("" ::: "memory");
        osDelay(1);
    }

    if (!success || !remain_armed) {
        motor_.disarm();
    }

    return success;
}


bool Axis::prepare_closed_loop_control() {
    if (closed_loop_prepared_) {
        return true;
    }
    controller_feedback_active_ = false;
    closed_loop_phase_feedback_ready_ = false;
    closed_loop_controller_ready_ = false;
    // Hook up the data paths between the components
    CRITICAL_SECTION() {
        controller_.pos_estimate_circular_src_.connect_to(&encoder_.pos_circular_);
        controller_.pos_wrap_src_.connect_to(&controller_.config_.circular_setpoint_range);
        controller_.pos_estimate_linear_src_.connect_to(&encoder_.pos_estimate_);
        controller_.vel_estimate_src_.connect_to(&encoder_.vel_estimate_);

        encoder_.reset_vernier_output_velocity_estimate();
        if (!encoder_.controller_feedback_ready()) {
            encoder_.set_error(Encoder::ERROR_VERNIER_RESOLVER_FAIL);
            return false;
        }

        // To avoid any transient on startup, we intialize the setpoint to be the current position
        if (!controller_.control_mode_updated()) {
            return false;
        }
        controller_.input_pos_updated();

        // Avoid integrator windup issues
        controller_.vel_integrator_torque_ = 0.0f;
        controller_.mechanical_power_ = 0.0f;
        controller_.electrical_power_ = 0.0f;

        motor_.torque_setpoint_src_.connect_to(&controller_.torque_output_);
        motor_.direction_ = encoder_.controller_to_motor_direction();

        motor_.current_control_.enable_current_control_src_ = true;
        motor_.current_control_.Idq_setpoint_src_.connect_to(&motor_.Idq_setpoint_);
        motor_.current_control_.Vdq_setpoint_src_.connect_to(&motor_.Vdq_setpoint_);

        // phase
        OutputPort<float>* phase_src = &encoder_.phase_;
        motor_.current_control_.phase_src_.connect_to(phase_src);
        // phase vel
        OutputPort<float>* phase_vel_src = &encoder_.phase_vel_;
        motor_.phase_vel_src_.connect_to(phase_vel_src);
        motor_.current_control_.phase_vel_src_.connect_to(phase_vel_src);
    }

    if (!motor_.is_armed_) {
        // Do not arm on the same state-machine turn that connects the ports.
        // Require several completed realtime iterations to prove that both
        // electrical feedback outputs are being published continuously.
        constexpr uint32_t kRequiredReadyObservations = 3u;
        constexpr uint32_t kMaximumReadyObservations = 20u;
        uint32_t consecutive_ready = 0u;
        for (uint32_t i = 0u;
             i < kMaximumReadyObservations &&
             consecutive_ready < kRequiredReadyObservations;
             ++i) {
            wait_for_control_iteration();
            consecutive_ready = closed_loop_phase_feedback_ready_
                ? consecutive_ready + 1u
                : 0u;
        }
        if (consecutive_ready < kRequiredReadyObservations) {
            closed_loop_phase_feedback_ready_ = false;
            return false;
        }

        // Only now expose the connected feedback path to the controller. This
        // prevents the controller from treating the encoder's acquisition
        // window as an invalid runtime estimate.
        const uint32_t controller_ready_sequence_start =
            closed_loop_controller_ready_sequence_;
        controller_feedback_active_ = true;
        for (uint32_t i = 0u;
             i < kMaximumReadyObservations &&
             static_cast<uint32_t>(closed_loop_controller_ready_sequence_ -
                                   controller_ready_sequence_start) <
                 kRequiredReadyObservations;
             ++i) {
            wait_for_control_iteration();
            if (controller_.error_ != Controller::ERROR_NONE ||
                error_ != ERROR_NONE) {
                break;
            }
        }
        if (static_cast<uint32_t>(closed_loop_controller_ready_sequence_ -
                                  controller_ready_sequence_start) <
                kRequiredReadyObservations ||
            !closed_loop_controller_ready_ ||
            controller_.error_ != Controller::ERROR_NONE ||
            error_ != ERROR_NONE) {
            controller_feedback_active_ = false;
            closed_loop_phase_feedback_ready_ = false;
            closed_loop_controller_ready_ = false;
            return false;
        }
    }

    closed_loop_prepared_ = true;
    return true;
}

bool Axis::arm_closed_loop_control() {
    if (!prepare_closed_loop_control()) {
        return false;
    }
    if (!motor_.is_armed_ && !motor_.arm(&motor_.current_control_)) {
        return false;
    }
    if (motor_.is_armed_) {
        // Start each armed interval with a fresh watchdog grace period; idle
        // deadlines must never leak into a new control transaction.
        ControlTimeout::clear(*this);
        ControlTimeout::feed_command(*this);
    }
    return motor_.is_armed_ && check_for_errors();
}

bool Axis::start_closed_loop_control() {
    return prepare_closed_loop_control() && arm_closed_loop_control();
}

bool Axis::stop_closed_loop_control() {
    motor_.disarm();
    controller_feedback_active_ = false;
    closed_loop_phase_feedback_ready_ = false;
    closed_loop_controller_ready_ = false;
    closed_loop_prepared_ = false;
    ControlTimeout::clear_running(*this);
    ControlTimeout::clear(*this);
    return check_for_errors();
}

bool Axis::run_closed_loop_control_loop() {
    if (!start_closed_loop_control()) {
        return false;
    }
    while ((requested_state_ == AXIS_STATE_UNDEFINED) && motor_.is_armed_) {
        service_safety_supervisor();
        osDelay(1);
    }

    stop_closed_loop_control();

    return check_for_errors();
}


bool Axis::run_idle_loop() {
    last_drv_fault_ = motor_.gate_driver_.get_error();
    while (requested_state_ == AXIS_STATE_UNDEFINED) {
        service_safety_supervisor();
        if (requested_state_ != AXIS_STATE_UNDEFINED || motor_.is_armed_) {
            break;
        }
        motor_.setup();
        osDelay(1);
    }
    return check_for_errors();
}

// Infinite loop that does calibration and enters main control loop as appropriate
void Axis::run_state_machine_loop() {
    for (;;) {
        service_safety_supervisor();

        // Load the task chain if a specific request is pending
        if (requested_state_ != AXIS_STATE_UNDEFINED) {
            size_t pos = 0;
            if (requested_state_ == AXIS_STATE_STARTUP_SEQUENCE) {
                if (config_.startup_motor_calibration)
                    task_chain_[pos++] = AXIS_STATE_MOTOR_CALIBRATION;
                if (config_.startup_encoder_offset_calibration)
                    task_chain_[pos++] = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
                if (config_.startup_closed_loop_control)
                    task_chain_[pos++] = AXIS_STATE_CLOSED_LOOP_CONTROL;
                task_chain_[pos++] = AXIS_STATE_IDLE;
            } else if (requested_state_ == AXIS_STATE_FULL_CALIBRATION_SEQUENCE) {
                task_chain_[pos++] = AXIS_STATE_MOTOR_CALIBRATION;
                task_chain_[pos++] = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
                task_chain_[pos++] = AXIS_STATE_IDLE;
            } else if (requested_state_ != AXIS_STATE_UNDEFINED) {
                task_chain_[pos++] = requested_state_;
                task_chain_[pos++] = AXIS_STATE_IDLE;
            }
            task_chain_[pos++] = AXIS_STATE_UNDEFINED;  // TODO: bounds checking
            requested_state_ = AXIS_STATE_UNDEFINED;
        }

        // START only wakes this thread. All stage selection and later
        // transitions are firmware-owned from this point onward.
        service_calibration_session_start();

        // Note that current_state is a reference to task_chain_[0]

        // Run the specified state
        // Handlers should exit if requested_state != AXIS_STATE_UNDEFINED
        bool status;
        switch (current_state_) {
            case AXIS_STATE_MOTOR_CALIBRATION: {
                // These error checks are a hacky way to force legacy behavior
                // when an error is raised. TODO: remove this when we overhaul
                // the error architecture
                // (https://github.com/madcowswe/ODrive/issues/526).
                //if (odrv.any_error())
                //    goto invalid_state_label;
                const bool session_owned = calibration_session_.active() &&
                    calibration_session_.stage() ==
                        CalibrationSession::STAGE_ELECTRICAL_CAPTURE;
                const float active_resistance = motor_.config_.phase_resistance;
                const float active_inductance = motor_.config_.phase_inductance;
                const bool active_calibrated = motor_.is_calibrated_;
                if (session_owned) calibration_capture_enabled_ = true;
                status = motor_.run_calibration();
                if (session_owned) {
                    calibration_capture_enabled_ = false;
                    if (status) {
                        calibration_pending_result_.phase_resistance =
                            motor_.config_.phase_resistance;
                        calibration_pending_result_.phase_inductance =
                            motor_.config_.phase_inductance;
                        calibration_pending_result_.validity |=
                            CalibrationPendingResult::VALID_PHASE_RESISTANCE |
                            CalibrationPendingResult::VALID_PHASE_INDUCTANCE;
                    }

                    // Identification must not alter the active model before
                    // validation and commit.
                    motor_.config_.phase_resistance = active_resistance;
                    motor_.config_.phase_inductance = active_inductance;
                    motor_.is_calibrated_ = active_calibrated;
                    motor_.update_current_controller_gains();

                    if (calibration_session_.active()) {
                        if (!status) {
                            calibration_session_.fail(
                                CalibrationSession::FAILURE_MOTOR_CALIBRATION);
                        } else if (
                            calibration_session_.profile() ==
                                CalibrationSession::PROFILE_FULL ||
                            calibration_session_.profile() ==
                                CalibrationSession::PROFILE_ELECTRICAL) {
                            calibration_session_.set_stage(
                                CalibrationSession::STAGE_ENCODER_GEOMETRY);
                            calibration_session_.set_progress(200);
                            status = run_calibration_encoder_alignment();
                            if (!status && calibration_session_.active()) {
                                calibration_session_.fail(
                                    CalibrationSession::FAILURE_ENCODER_CALIBRATION);
                            }
                            if (status && calibration_session_.active()) {
                                if (kEnableReducerIdentification) {
                                    calibration_session_.set_progress(250);
                                    status = run_calibration_geometry_scan();
                                } else {
                                    calibration_session_.set_stage(
                                        CalibrationSession::
                                            STAGE_ELECTRICAL_CAPTURE);
                                    calibration_session_.set_progress(250);
                                    status = run_calibration_flux_scan();
                                }
                            }
                            if (kEnableReducerIdentification &&
                                status && calibration_session_.active()) {
                                calibration_session_.set_stage(
                                    CalibrationSession::STAGE_MECHANICAL_CAPTURE);
                                calibration_session_.set_progress(650);
                                status = run_calibration_mechanical_scan();
                                if (!status && calibration_session_.active()) {
                                    calibration_session_.fail(
                                        CalibrationSession::FAILURE_MECHANICAL_SCAN);
                                }
                            }
                            if (kEnableReducerIdentification &&
                                status && calibration_session_.active()) {
                                calibration_session_.set_stage(
                                    CalibrationSession::STAGE_ELECTRICAL_DELAY);
                                calibration_session_.set_progress(850);
                                status = run_calibration_delay_scan();
                                if (!status && calibration_session_.active()) {
                                    calibration_session_.fail(
                                        CalibrationSession::FAILURE_ELECTRICAL_DELAY);
                                }
                            }
                            if (calibration_session_.active()) {
                                if (status) {
                                    calibration_session_.transition(
                                        CalibrationSession::STATE_COLLECTED);
                                    calibration_session_.set_progress(940);
                                    calibration_session_.set_stage(
                                        CalibrationSession::STAGE_FITTING);
                                    calibration_session_.transition(
                                        CalibrationSession::STATE_FITTING);
                                    calibration_session_.transition(
                                        CalibrationSession::STATE_IDENTIFIED);
                                    calibration_session_.set_progress(950);
                                    calibration_session_.set_stage(
                                        CalibrationSession::STAGE_VALIDATION);
                                    calibration_session_.transition(
                                        CalibrationSession::STATE_VALIDATING);
                                    if (!validate_calibration_candidate()) {
                                        status = false;
                                        calibration_session_.fail(
                                            CalibrationSession::FAILURE_VALIDATION);
                                    } else {
                                        calibration_session_.transition(
                                            CalibrationSession::STATE_VALIDATED);
                                        calibration_session_.set_progress(975);
                                        calibration_session_.set_stage(
                                            CalibrationSession::STAGE_COMMIT);
                                        calibration_session_.transition(
                                            CalibrationSession::STATE_STAGED);
                                        if (commit_calibration_candidate()) {
                                            calibration_session_.set_progress(1000);
                                            calibration_session_.transition(
                                                CalibrationSession::STATE_COMMITTED);
                                        } else {
                                            status = false;
                                            calibration_session_.fail(
                                                CalibrationSession::FAILURE_STORAGE);
                                        }
                                    }
                                } else {
                                    calibration_session_.fail(
                                        CalibrationSession::FAILURE_GEOMETRY_SCAN);
                                }
                            }
                        } else {
                            calibration_session_.transition(
                                CalibrationSession::STATE_COLLECTED);
                            calibration_session_.set_progress(350);
                        }
                    }
                }
            } break;

            case AXIS_STATE_ENCODER_OFFSET_CALIBRATION: {
                //if (odrv.any_error())
                //    goto invalid_state_label;
                if (!motor_.is_calibrated_)
                    goto invalid_state_label;
                const bool session_owned = calibration_session_.active() &&
                    calibration_session_.stage() ==
                        CalibrationSession::STAGE_ENCODER_GEOMETRY;
                const int32_t active_direction = encoder_.config_.direction;
                const int32_t active_phase_offset = encoder_.config_.phase_offset;
                const float active_phase_offset_float =
                    encoder_.config_.phase_offset_float;
                const bool active_ready = encoder_.is_ready_;
                status = encoder_.run_offset_calibration();
                if (session_owned) {
                    if (status) {
                        calibration_pending_result_.encoder_direction =
                            encoder_.config_.direction;
                        calibration_pending_result_.phase_offset =
                            encoder_.config_.phase_offset;
                        calibration_pending_result_.phase_offset_float =
                            encoder_.config_.phase_offset_float;
                        calibration_pending_result_.validity |=
                            CalibrationPendingResult::VALID_ENCODER_DIRECTION |
                            CalibrationPendingResult::VALID_ELECTRICAL_OFFSET;
                    }
                    encoder_.config_.direction = active_direction;
                    encoder_.config_.phase_offset = active_phase_offset;
                    encoder_.config_.phase_offset_float =
                        active_phase_offset_float;
                    encoder_.is_ready_ = active_ready;

                    if (calibration_session_.active()) {
                        if (!status) {
                            calibration_session_.fail(
                                CalibrationSession::FAILURE_ENCODER_CALIBRATION);
                        } else {
                            calibration_session_.transition(
                                CalibrationSession::STATE_COLLECTED);
                            calibration_session_.set_progress(350);
                        }
                    }
                }
            } break;

            case AXIS_STATE_LOCKIN_SPIN: {
                //if (odrv.any_error())
                //    goto invalid_state_label;
                if (!motor_.is_calibrated_ || encoder_.config_.direction==0)
                    goto invalid_state_label;
                status = run_lockin_spin(config_.general_lockin, false);
            } break;

            case AXIS_STATE_CLOSED_LOOP_CONTROL: {
                //if (odrv.any_error())
                //    goto invalid_state_label;
                if (!motor_.is_calibrated_ || encoder_.config_.direction==0)
                    goto invalid_state_label;
                watchdog_feed();
                status = run_closed_loop_control_loop();
            } break;

            case AXIS_STATE_IDLE: {
                run_idle_loop();
                status = true;
            } break;

            default:
            invalid_state_label:
                {
                    odrive::fault::FaultRecord record;
                    record.control_sequence = odrv.n_evt_control_loop_;
                    record.timestamp_cycles = DWT->CYCCNT;
                    record.source = odrive::fault::FaultSource::AXIS;
                    record.code = odrive::fault::FaultCode::INVALID_STATE;
                    record.site = odrive::fault::FaultSite::STATE_TRANSITION;
                    record.severity = odrive::fault::FaultSeverity::LATCHED;
                    record.arg0 = static_cast<uint32_t>(current_state_);
                    raise_fault(record, ERROR_INVALID_STATE);
                }
                status = false;  // this will set the state to idle
                break;
        }

        // If the state failed, go to idle, else advance task chain
        if (!status) {
            std::fill(task_chain_.begin(), task_chain_.end(), AXIS_STATE_UNDEFINED);
            current_state_ = AXIS_STATE_IDLE;
        } else {
            std::rotate(task_chain_.begin(), task_chain_.begin() + 1, task_chain_.end());
            task_chain_.back() = AXIS_STATE_UNDEFINED;
        }
    }
}
