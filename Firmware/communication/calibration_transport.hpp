#ifndef __CALIBRATION_TRANSPORT_HPP
#define __CALIBRATION_TRANSPORT_HPP

#include <stdint.h>

struct CalibrationTransportStats {
    volatile uint32_t frames_sent = 0;
    volatile uint32_t queue_retries = 0;
    volatile uint32_t disconnect_waits = 0;
    volatile uint32_t records_discarded_disconnected = 0;
};

extern CalibrationTransportStats calibration_transport_stats;

void start_calibration_transport();

#endif // __CALIBRATION_TRANSPORT_HPP
