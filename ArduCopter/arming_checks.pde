/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *       This file contains functionality to send arming_check_report messages to the GCS
 */

// run pre-arm and arm checks or somehow be passed the bits
//    checks will be run over and over
//    result of checks can be:
//          ArmingCheck_Skipped - check has been skipped
//          ArmingCheck_Failed - some user action is required to resolve the failue
//          ArmingCheck_Pending - waiting may resolve the issue (or it may not)
//          ArmingCheck_Passed - check passed (for now at least)
// prepare the present, passed, failed bits
// send message to GCS

// To-Do: add ARMING_CHECK_RANGEFINDER, ARMING_CHECK_OPTICAL_FLOW

// Problems: mode-is-armable requires knowing how we will arm..

// ARMING_CHECK_MASK_ALL | ARMING_CHECK_MASK_NONNAV
// ARMING_CHECK_MASK_AHRS | ARMING_CHECK_MASK_BARO | ARMING_CHECK_MASK_BOARD | ARMING_CHECK_MASK_COMPASS |
// ARMING_CHECK_MASK_FENCE | ARMING_CHECK_MASK_GPS | ARMING_CHECK_MASK_INS | ARMING_CHECK_MASK_PARAMS | ARMING_CHECK_MASK_RC | ARMING_CHECK_MASK_LOGGING | ARMING_CHECK_MASK_MAIN

// converts a ArmingCheckResult into the passed/failed bits - assumes relevant bit is zero before being passed in
void arming_check_bits_from_result(enum ArmingCheckResult res, enum ARMING_CHECK_MASK check, uint32_t& present, uint32_t& passed, uint32_t& failed)
{
    switch (res) {
    case ArmingCheck_Skipped:
        // do nothing to leave all bits as zero
        break;
    case ArmingCheck_Failed:
        present |= check;
        failed |= check;
        break;
    case ArmingCheck_Pending:
        present |= check;
        failed |= check;
        break;
    case ArmingCheck_Passed:
        present |= check;
        passed |= check;
        break;
    }
}

// update overall from individual result
void arming_check_update_overall(enum ArmingCheckResult res, enum ArmingCheckResult &overall)
{
    // a single failure downgrades the overall to failure
    if (res == ArmingCheck_Failed) {
        overall = ArmingCheck_Failed;
    }

    // a pending downgrades a passed to pending
    if (res == ArmingCheck_Pending && overall == ArmingCheck_Passed) {
        overall = ArmingCheck_Pending;
    }
}

// prepare and send arming_check_report to GCSs
void arming_checks_send_report(bool display_failure)
{
    // exit immediately if already armed
    if (motors.armed()) {
        return;
    }

    uint32_t present = ARMING_CHECK_MASK_ALL | ARMING_CHECK_MASK_NONNAV;
    uint32_t passed = 0;
    uint32_t failed = 0;
    enum ArmingCheckResult ret;
    enum ArmingCheckResult ret_all = ArmingCheck_Passed;

    // handle case where arming checks have been disabled?
    // handle case where motors are already armed?

    // ARMING_CHECK_AHRS
    ret = arming_check_ahrs(display_failure);
    arming_check_bits_from_result(ret, ARMING_CHECK_MASK_AHRS, present, passed, failed);
    arming_check_update_overall(ret, ret_all);

    // ARMING_CHECK_AIRSPEED -- not implemented for copters

    // ARMING_CHECK_BARO
    ret = arming_check_baro(display_failure);
    arming_check_bits_from_result(ret, ARMING_CHECK_MASK_BARO, present, passed, failed);
    arming_check_update_overall(ret, ret_all);

    // ARMING_CHECK_BOARD
    // ARMING_CHECK_COMPASS
    // ARMING_CHECK_FENCE
    // ARMING_CHECK_INS

    // ARMING_CHECK_PARAMS
    ret = arming_check_params(display_failure);
    arming_check_bits_from_result(ret, ARMING_CHECK_MASK_PARAMS, present, passed, failed);
    arming_check_update_overall(ret, ret_all);

    // ARMING_CHECK_RC
    ret = arming_checks_rc(display_failure);
    arming_check_bits_from_result(ret, ARMING_CHECK_MASK_RC, present, passed, failed);
    arming_check_update_overall(ret, ret_all);

    // ARMING_CHECK_LOGGING
    // ARMING_CHECK_MAIN
        // flight mode

    // ARMING_CHECK_NONNAV - everything up until now
    arming_check_bits_from_result(ret_all, ARMING_CHECK_MASK_NONNAV, present, passed, failed);

    // ARMING_CHECK_GPS
    arming_check_bits_from_result(ret, ARMING_CHECK_MASK_GPS, present, passed, failed);
    arming_check_update_overall(ret, ret_all);

    // ARMING_CHECK_ALL - everything including GPS
    arming_check_bits_from_result(ret_all, ARMING_CHECK_MASK_ALL, present, passed, failed);

    // send to all GCSs
    gcs_send_arming_check_report(present, passed, failed);
}

// perform arming checks of AHRS
enum ArmingCheckResult arming_check_ahrs(bool display_failure)
{
    // always check if inertial nav has started and is ready
    if(!ahrs.get_NavEKF().healthy()) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Waiting for Nav Checks"));
        }
        return ArmingCheck_Pending;
    }

    // return success
    return ArmingCheck_Passed;
}

// perform arming checks of Baro
enum ArmingCheckResult arming_check_baro(bool display_failure)
{
    // return skipped if checks are disabled
    if ((g.arming_check != ARMING_CHECK_ALL) && !(g.arming_check & ARMING_CHECK_BARO)) {
        return ArmingCheck_Skipped;
    }

    // barometer health check
    if(!barometer.all_healthy()) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Barometer not healthy"));
        }
        return ArmingCheck_Failed;
    }

    // Check baro & inav alt are within 1m if EKF is operating in an absolute position mode.
    // Do not check if intending to operate in a ground relative height mode as EKF will output a ground relative height
    // that may differ from the baro height due to baro drift.
    nav_filter_status filt_status = inertial_nav.get_filter_status();
    bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
    if (using_baro_ref) {
        if (fabs(inertial_nav.get_altitude() - baro_alt) > PREARM_MAX_ALT_DISPARITY_CM) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Altitude disparity"));
            }
            return ArmingCheck_Pending;
        }
    }

    // return success
    return ArmingCheck_Passed;
}

// perform arming checks of RC
enum ArmingCheckResult arming_checks_rc(bool display_failure)
{
    // set rc-checks to skipped if RC checks are disabled
    if ((g.arming_check != ARMING_CHECK_ALL) && !(g.arming_check & ARMING_CHECK_RC)) {
        return ArmingCheck_Skipped;
    }

    // check if radio has been calibrated
    if(!g.rc_3.radio_min.load() && !g.rc_3.radio_max.load()) {
        return ArmingCheck_Failed;
    }

    // check channels 1 & 2 have min <= 1300 and max >= 1700
    if (g.rc_1.radio_min > 1300 || g.rc_1.radio_max < 1700 || g.rc_2.radio_min > 1300 || g.rc_2.radio_max < 1700) {
        return ArmingCheck_Failed;
    }

    // check channels 3 & 4 have min <= 1300 and max >= 1700
    if (g.rc_3.radio_min > 1300 || g.rc_3.radio_max < 1700 || g.rc_4.radio_min > 1300 || g.rc_4.radio_max < 1700) {
        return ArmingCheck_Failed;
    }

    // check channels 1 & 2 have trim >= 1300 and <= 1700
    if (g.rc_1.radio_trim < 1300 || g.rc_1.radio_trim > 1700 || g.rc_2.radio_trim < 1300 || g.rc_2.radio_trim > 1700) {
        return ArmingCheck_Failed;
    }

    // check channel 4 has trim >= 1300 and <= 1700
    if (g.rc_4.radio_trim < 1300 || g.rc_4.radio_trim > 1700) {
        return ArmingCheck_Failed;
    }

    // if we've gotten this far rc is ok
    return ArmingCheck_Passed;
}

// perform arming checks of parameters
enum ArmingCheckResult arming_check_params(bool display_failure)
{
    return ArmingCheck_Passed;
}
