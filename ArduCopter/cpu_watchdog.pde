// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
//  cpu watchdog support
//  Andrew Tridgell, December 2011
//
//  our failsafe strategy is to detect main loop lockup and disarm the motors
//

static bool watchdog_enabled = false;
static uint16_t watchdog_last_mainLoop_count;
static uint32_t watchdog_last_timestamp;
static bool in_failsafe;

//
// watchdog_enable - enable failsafe
//
void watchdog_enable()
{
    watchdog_enabled = true;
    watchdog_last_timestamp = micros();
}

//
// watchdog_disable - used when we know we are going to delay the mainloop significantly
//
void watchdog_disable()
{
    watchdog_enabled = false;
}

//
//  watchdog_check - this function is called from the core timer interrupt at 1kHz.
//
void watchdog_check()
{
    uint32_t tnow = hal.scheduler->micros();

    if (mainLoop_count != watchdog_last_mainLoop_count) {
        // the main loop is running, all is OK
        watchdog_last_mainLoop_count = mainLoop_count;
        watchdog_last_timestamp = tnow;
        if (in_failsafe) {
            in_failsafe = false;
            Log_Write_Error(ERROR_SUBSYSTEM_CPU,ERROR_CODE_FAILSAFE_RESOLVED);
        }
        return;
    }

    if (!in_failsafe && watchdog_enabled && tnow - watchdog_last_timestamp > 2000000) {
        // motors are running but we have gone 2 second since the
        // main loop ran. That means we're in trouble and should
        // disarm the motors.
        in_failsafe = true;
        // reduce motors to minimum (we do not immediately disarm because we want to log the failure)
        if (motors.armed()) {
            motors.output_min();
        }
        // log an error
        Log_Write_Error(ERROR_SUBSYSTEM_CPU,ERROR_CODE_FAILSAFE_OCCURRED);
    }

    if (watchdog_enabled && in_failsafe && tnow - watchdog_last_timestamp > 1000000) {
        // disarm motors every second
        watchdog_last_timestamp = tnow;
        if(motors.armed()) {
            motors.armed(false);
            motors.output();
        }
    }
}
