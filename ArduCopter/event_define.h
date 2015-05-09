// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *       This file contains functionality to send event messages to the GCS
 *
 *       To add a new event:
 *           1. use EVENT_ENUM() macro to add new event to the EventId enum
 *           2. use EVENT_DESC to defined the string to be sent to the GCS
 *           3. use EVENT_INFO to define the severity of the event to be sent to the GCS
 */

#ifndef _EVENTS_H
#define _EVENTS_H

// macro definitions
#define EVENT_ENUM(name) EVENTID_ ## name,
#define EVENT_DESC(name, desc) const char* EVENTID_ ## name ## _desc = desc;
#define EVENT_INFO(name, severity) {EVENTID_ ## name, severity, EVENTID_ ## name ## _desc},

#define EVENT_SET       1
#define EVENT_CLEARED   0

// add new events into this enum
enum EventId {
    // autotune events
    EVENTID_AUTOTUNE_STARTED = 0,                       // 0
    EVENT_ENUM(AUTOTUNE_RESTARTED)                      // 1
    EVENT_ENUM(AUTOTUNE_STOPPED)                        // 2
    EVENT_ENUM(AUTOTUNE_SUCCESS)                        // 3
    EVENT_ENUM(AUTOTUNE_FAILED)                         // 4
    EVENT_ENUM(AUTOTUNE_SAVED_GAINS)                    // 5
    EVENT_ENUM(AUTOTUNE_PILOTTESTING)                   // 6
    // baro
    EVENT_ENUM(BARO_CALIBRATING)                        // 7
    EVENT_ENUM(BARO_CALIBRATION_COMPLETE)               // 8
    // crash check events
    EVENT_ENUM(CRASHCHECK_DISARMED)                     // 9
    // dataflash
    EVENT_ENUM(DATAFLASH_ERASING_LOGS)                  // 10
    EVENT_ENUM(DATAFLASH_ERASE_COMPLETE)                // 11
    EVENT_ENUM(DATAFLASH_NO_DATAFLASH_INSERTED)         // 12
    // esc calibration
    EVENT_ENUM(ESC_CALIBRATION_RESTART_BOARD)           // 13
    EVENT_ENUM(ESC_CALIBRATION_PASSINGTHROUGH)          // 14
    EVENT_ENUM(ESC_CALIBRATION_AUTOCALIBRATION)         // 15
    EVENT_ENUM(ESC_CALIBRATION_PUSH_SAFETYSWITCH)       // 16
    // failsafe
    EVENT_ENUM(FAILSAFE_RADIO)                          // 17
    EVENT_ENUM(FAILSAFE_EKF_VARIANCE)                   // 18
    EVENT_ENUM(FAILSAFE_BATTERY_VOLTAGE_LOW)            // 19
    EVENT_ENUM(FAILSAFE_GCS)                            // 20
    // ins events
    EVENT_ENUM(INS_GYRO_CAL_FAILED)                     // 21
    // main autopilot events
    EVENT_ENUM(MAIN_INITIALISING)                       // 22
    EVENT_ENUM(MAIN_GROUND_START)                       // 23
    EVENT_ENUM(MAIN_WAITING_FOR_HIL)                    // 24
    EVENT_ENUM(MAIN_ARMING)                             // 25
    EVENT_ENUM(MAIN_DISARMING)                          // 26
    EVENT_ENUM(MAIN_LOCATE_COPTER_ALARM)                // 27
    EVENT_ENUM(MAIN_TRIM_SAVED)                         // 28
    // parachute
    EVENT_ENUM(PARACHUTE_RELEASED)                      // 29
    EVENT_ENUM(PARACHUTE_FAILED_TOO_LOW)                // 30

    // this must be the last item in the enum
    EVENTID_LAST_EVENT
};

struct EventInfo {
    enum EventId event_id;
    gcs_severity severity;
    const char* description;
};

// descriptions - add strings for the new events below
EVENT_DESC(AUTOTUNE_STARTED, "AutoTune: Started")
EVENT_DESC(AUTOTUNE_RESTARTED, "AutoTune: Re-Started")
EVENT_DESC(AUTOTUNE_STOPPED, "AutoTune: Stopped")
EVENT_DESC(AUTOTUNE_SUCCESS, "AutoTune: Success")
EVENT_DESC(AUTOTUNE_FAILED, "AutoTune: Failed")
EVENT_DESC(AUTOTUNE_SAVED_GAINS, "AutoTune: Saved Gains")
EVENT_DESC(AUTOTUNE_PILOTTESTING, "");  // never sent
EVENT_DESC(BARO_CALIBRATING, "Calibrating barometer")
EVENT_DESC(BARO_CALIBRATION_COMPLETE, "barometer calibration complete")
EVENT_DESC(CRASHCHECK_DISARMED, "Crash: Disarming")
EVENT_DESC(DATAFLASH_ERASING_LOGS, "Erasing logs")
EVENT_DESC(DATAFLASH_ERASE_COMPLETE, "Log erase complete")
EVENT_DESC(DATAFLASH_NO_DATAFLASH_INSERTED, "No dataflash inserted")
EVENT_DESC(ESC_CALIBRATION_RESTART_BOARD, "ESC Calibration: restart board")
EVENT_DESC(ESC_CALIBRATION_PASSINGTHROUGH, "Passing pilot throttle to ESCs")
EVENT_DESC(ESC_CALIBRATION_AUTOCALIBRATION, "Auto ESC calibration")
EVENT_DESC(ESC_CALIBRATION_PUSH_SAFETYSWITCH, "push safety switch")
EVENT_DESC(FAILSAFE_RADIO, "Radio Failsafe")
EVENT_DESC(FAILSAFE_EKF_VARIANCE, "EKF Variance")
EVENT_DESC(FAILSAFE_BATTERY_VOLTAGE_LOW, "Battery Low")
EVENT_DESC(FAILSAFE_GCS, "GCS Failsafe")
EVENT_DESC(INS_GYRO_CAL_FAILED, "Arm: Gyro calibration failed")
EVENT_DESC(MAIN_INITIALISING, "Initialising APM...")
EVENT_DESC(MAIN_GROUND_START, "GROUND START")
EVENT_DESC(MAIN_WAITING_FOR_HIL, "Waiting for first HIL_STATE msg")
EVENT_DESC(MAIN_ARMING, "ARMING MOTORS")
EVENT_DESC(MAIN_DISARMING, "DISARMING MOTORS")
EVENT_DESC(MAIN_LOCATE_COPTER_ALARM, "Locate Copter Alarm")
EVENT_DESC(MAIN_TRIM_SAVED, "Trim saved")
EVENT_DESC(PARACHUTE_RELEASED, "Parachute Released")
EVENT_DESC(PARACHUTE_FAILED_TOO_LOW, "Parachute: Alt Too Low")

// event descriptions - add new event and severity below
const EventInfo evtinfo[EVENTID_LAST_EVENT] = {
        EVENT_INFO(AUTOTUNE_STARTED, SEVERITY_HIGH)
        EVENT_INFO(AUTOTUNE_RESTARTED, SEVERITY_HIGH)
        EVENT_INFO(AUTOTUNE_STOPPED, SEVERITY_HIGH)
        EVENT_INFO(AUTOTUNE_SUCCESS, SEVERITY_HIGH)
        EVENT_INFO(AUTOTUNE_FAILED, SEVERITY_HIGH)
        EVENT_INFO(AUTOTUNE_SAVED_GAINS, SEVERITY_HIGH)
        EVENT_INFO(AUTOTUNE_PILOTTESTING, SEVERITY_LOW) // never sent
        EVENT_INFO(BARO_CALIBRATING, SEVERITY_LOW)
        EVENT_INFO(BARO_CALIBRATION_COMPLETE, SEVERITY_LOW)
        EVENT_INFO(CRASHCHECK_DISARMED, SEVERITY_HIGH)
        EVENT_INFO(DATAFLASH_ERASING_LOGS, SEVERITY_HIGH)
        EVENT_INFO(DATAFLASH_ERASE_COMPLETE, SEVERITY_HIGH)
        EVENT_INFO(DATAFLASH_NO_DATAFLASH_INSERTED, SEVERITY_HIGH)
        EVENT_INFO(ESC_CALIBRATION_RESTART_BOARD, SEVERITY_HIGH)
        EVENT_INFO(ESC_CALIBRATION_PASSINGTHROUGH, SEVERITY_HIGH)
        EVENT_INFO(ESC_CALIBRATION_AUTOCALIBRATION, SEVERITY_HIGH)
        EVENT_INFO(ESC_CALIBRATION_PUSH_SAFETYSWITCH, SEVERITY_HIGH)
        EVENT_INFO(FAILSAFE_RADIO, SEVERITY_HIGH)
        EVENT_INFO(FAILSAFE_EKF_VARIANCE, SEVERITY_HIGH)
        EVENT_INFO(FAILSAFE_BATTERY_VOLTAGE_LOW, SEVERITY_HIGH)
        EVENT_INFO(FAILSAFE_GCS, SEVERITY_HIGH)
        EVENT_INFO(INS_GYRO_CAL_FAILED, SEVERITY_HIGH)
        EVENT_INFO(MAIN_INITIALISING, SEVERITY_HIGH)
        EVENT_INFO(MAIN_GROUND_START, SEVERITY_LOW)
        EVENT_INFO(MAIN_WAITING_FOR_HIL, SEVERITY_LOW)
        EVENT_INFO(MAIN_ARMING, SEVERITY_HIGH)
        EVENT_INFO(MAIN_DISARMING, SEVERITY_HIGH)
        EVENT_INFO(MAIN_LOCATE_COPTER_ALARM, SEVERITY_HIGH)
        EVENT_INFO(MAIN_TRIM_SAVED, SEVERITY_HIGH)
        EVENT_INFO(PARACHUTE_RELEASED, SEVERITY_HIGH)
        EVENT_INFO(PARACHUTE_FAILED_TOO_LOW, SEVERITY_HIGH)
};

#endif
