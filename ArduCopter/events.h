// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *       This file contains functionality to send event messages to the GCS
 */

#ifndef _EVENTS_H
#define _EVENTS_H

enum EventId {
    // autotune events
    EVENTID_AUTOTUNE_STARTED,
    EVENTID_AUTOTUNE_RESTARTED,
    EVENTID_AUTOTUNE_STOPPED,
    EVENTID_AUTOTUNE_SUCCESS,
    EVENTID_AUTOTUNE_FAILED,
    EVENTID_AUTOTUNE_SAVED_GAINS,
    EVENTID_AUTOTUNE_PILOTTESTING,
    // baro
    EVENTID_BARO_CALIBRATING,
    EVENTID_BARO_CALIBRATION_COMPLETE,
    // crash check events
    EVENTID_CRASHCHECK_DISARMED,
    // dataflash
    EVENTID_DATAFLASH_ERASING_LOGS,
    EVENTID_DATAFLASH_ERASE_COMPLETE,
    EVENTID_DATAFLASH_NO_DATAFLASH_INSERTED,
    // esc calibration
    EVENTID_ESC_CALIBRATION_RESTART_BOARD,
    EVENTID_ESC_CALIBRATION_PASSINGTHROUGH,
    EVENTID_ESC_CALIBRATION_AUTOCALIBRATION,
    EVENTID_ESC_CALIBRATION_PUSH_SAFETYSWITCH,
    // failsafe
    EVENTID_FAILSAFE_RADIO,
    EVENTID_FAILSAFE_EKF_VARIANCE,
    EVENTID_FAILSAFE_BATTERY_VOLTAGE_LOW,
    EVENTID_FAILSAFE_GCS,
    // ins events
    EVENTID_INS_GYRO_CAL_FAILED,
    // main autopilot events
    EVENTID_MAIN_INITIALISING,
    EVENTID_MAIN_GROUND_START,
    EVENTID_MAIN_WAITING_FOR_HIL,
    EVENTID_MAIN_ARMING,
    EVENTID_MAIN_DISARMING,
    EVENTID_MAIN_LOCATE_COPTER_ALARM,
    EVENTID_MAIN_TRIM_SAVED,
    // parachute
    EVENTID_PARACHUTE_RELEASED,
    EVENTID_PARACHUTE_FAILED_TOO_LOW,

    // this must be the last item in the enum
    EVENTID_LAST_EVENT
};

#define EVENT_SET       1
#define EVENT_CLEARED   0

#define EVENT_NUM_ITEMS (EVENTID_LAST_EVENT)
#define EVENT_DESC_LEN 32

struct EventInfo {
    enum EventId event_id;
    gcs_severity severity;
    char description[EVENT_DESC_LEN];
};

// event descriptions
const EventInfo evtinfo[EVENT_NUM_ITEMS] = {
        // autotune events
        {EVENTID_AUTOTUNE_STARTED, SEVERITY_HIGH, "AutoTune: Started"},
        {EVENTID_AUTOTUNE_RESTARTED, SEVERITY_HIGH, "AutoTune: Re-Started"},
        {EVENTID_AUTOTUNE_STOPPED, SEVERITY_HIGH, "AutoTune: Stopped"},
        {EVENTID_AUTOTUNE_SUCCESS, SEVERITY_HIGH, "AutoTune: Success"},
        {EVENTID_AUTOTUNE_FAILED, SEVERITY_HIGH, "AutoTune: Failed"},
        {EVENTID_AUTOTUNE_SAVED_GAINS, SEVERITY_HIGH, "AutoTune: Saved Gains"},

        // baro
        {EVENTID_BARO_CALIBRATING, SEVERITY_LOW, "Calibrating barometer"},
        {EVENTID_BARO_CALIBRATION_COMPLETE, SEVERITY_LOW, "barometer calibration complete"},

        // crash check events
        {EVENTID_CRASHCHECK_DISARMED, SEVERITY_HIGH, "Crash: Disarming"},

        // dataflash
        {EVENTID_DATAFLASH_ERASING_LOGS, SEVERITY_HIGH, "Erasing logs"},
        {EVENTID_DATAFLASH_ERASE_COMPLETE, SEVERITY_HIGH, "Log erase complete"},
        {EVENTID_DATAFLASH_NO_DATAFLASH_INSERTED, SEVERITY_HIGH, "No dataflash inserted"},

        // esc calibration
        {EVENTID_ESC_CALIBRATION_RESTART_BOARD, SEVERITY_HIGH, "ESC Calibration: restart board"},
        {EVENTID_ESC_CALIBRATION_PASSINGTHROUGH, SEVERITY_HIGH, "Passing pilot throttle to ESCs"},
        {EVENTID_ESC_CALIBRATION_AUTOCALIBRATION, SEVERITY_HIGH, "Auto ESC calibration"},
        {EVENTID_ESC_CALIBRATION_PUSH_SAFETYSWITCH, SEVERITY_HIGH, "push safety switch"},

        // failsafe
        {EVENTID_FAILSAFE_RADIO, SEVERITY_HIGH, "Radio Failsafe"},
        {EVENTID_FAILSAFE_EKF_VARIANCE, SEVERITY_HIGH, "EKF Variance"},
        {EVENTID_FAILSAFE_BATTERY_VOLTAGE_LOW, SEVERITY_HIGH, "Battery Low"},
        {EVENTID_FAILSAFE_GCS, SEVERITY_HIGH, "GCS Failsafe"},

        // ins events
        {EVENTID_INS_GYRO_CAL_FAILED, SEVERITY_HIGH, "Gyro calibration failed"},

        // main autopilot events
        {EVENTID_MAIN_INITIALISING, SEVERITY_HIGH, "Initialising APM..."},
        {EVENTID_MAIN_GROUND_START, SEVERITY_LOW, "GROUND START"},
        {EVENTID_MAIN_WAITING_FOR_HIL, SEVERITY_LOW, "Waiting for first HIL_STATE msg"},
        {EVENTID_MAIN_ARMING, SEVERITY_HIGH, "ARMING MOTORS"},
        {EVENTID_MAIN_DISARMING, SEVERITY_HIGH, "DISARMING MOTORS"},
        {EVENTID_MAIN_LOCATE_COPTER_ALARM, SEVERITY_HIGH, "Locate Copter Alarm"},
        {EVENTID_MAIN_TRIM_SAVED, SEVERITY_HIGH, "Trim saved"},

        // parachute
        {EVENTID_PARACHUTE_RELEASED, SEVERITY_HIGH, "Parachute Released"},
        {EVENTID_PARACHUTE_FAILED_TOO_LOW, SEVERITY_HIGH, "Parachute: Alt Too Low"},
};

#endif
