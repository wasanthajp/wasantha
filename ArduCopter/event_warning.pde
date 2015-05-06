// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *       This file contains functionality to send event messages to the GCS
 */

#include "events.h"

// retrieve event description
static bool event_get_info(uint16_t event_id, struct EventInfo& ret)
{
    // sanity check event_id
    if (event_id >= EVENTID_LAST_EVENT) {
        return false;
    }

    // return event
    ret = evtinfo[event_id];
    return true;
}

// send event info to GCS and record in dataflash
static void event_send_and_record(uint8_t event_id, uint8_t set_or_cleared)
{
    // log event to dataflash
    event_record(event_id, set_or_cleared);

    // send event to GCS
    event_send(event_id, set_or_cleared);
}

// send event to GCS
static void event_send(uint8_t event_id, uint8_t set_or_cleared)
{
    // send event to GCS
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            // sent event message
            gcs[i].send_vehicle_event(event_id, set_or_cleared);
        }
    }
}

// record event in dataflash
static void event_record(uint8_t event_id, uint8_t set_or_cleared)
{
    // log event to dataflash
    Log_Write_Error(event_id, set_or_cleared);
}

// send event info to GCS
static bool event_handle_request_event_info(mavlink_channel_t chan, mavlink_message_t* msg)
{
    __mavlink_request_vehicle_event_info_t packet;
    mavlink_msg_request_vehicle_event_info_decode(msg, &packet);

    struct EventInfo evinf;

    // return immediately if we can't find the event info
    if (!event_get_info(packet.event_id, evinf)) {
        return false;
    }

    // check we have enough space
    if (comm_get_txspace(chan) >= MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_VEHICLE_EVENT_INFO_LEN) {
        return false;
    }

    // copy description to larger temp buffer
    char temp_desc[64];
    memset(temp_desc,0,sizeof(temp_desc));
    memcpy(evinf.description, temp_desc, EVENT_DESC_LEN);

    // send event info to GCS
    mavlink_msg_vehicle_event_info_send(chan, evinf.event_id, evinf.severity, evinf.description);
    return true;
}

