// MESSAGE VEHICLE_EVENT PACKING

#define MAVLINK_MSG_ID_VEHICLE_EVENT 226

typedef struct __mavlink_vehicle_event_t
{
 uint16_t event_id; ///< Event Id
 uint8_t flags; ///< Flags.  See enum VEHICLE_EVENT_FLAGS
} mavlink_vehicle_event_t;

#define MAVLINK_MSG_ID_VEHICLE_EVENT_LEN 3
#define MAVLINK_MSG_ID_226_LEN 3

#define MAVLINK_MSG_ID_VEHICLE_EVENT_CRC 229
#define MAVLINK_MSG_ID_226_CRC 229



#define MAVLINK_MESSAGE_INFO_VEHICLE_EVENT { \
	"VEHICLE_EVENT", \
	2, \
	{  { "event_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_vehicle_event_t, event_id) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_vehicle_event_t, flags) }, \
         } \
}


/**
 * @brief Pack a vehicle_event message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param event_id Event Id
 * @param flags Flags.  See enum VEHICLE_EVENT_FLAGS
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vehicle_event_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t event_id, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VEHICLE_EVENT_LEN];
	_mav_put_uint16_t(buf, 0, event_id);
	_mav_put_uint8_t(buf, 2, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VEHICLE_EVENT_LEN);
#else
	mavlink_vehicle_event_t packet;
	packet.event_id = event_id;
	packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VEHICLE_EVENT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VEHICLE_EVENT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VEHICLE_EVENT_LEN, MAVLINK_MSG_ID_VEHICLE_EVENT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VEHICLE_EVENT_LEN);
#endif
}

/**
 * @brief Pack a vehicle_event message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param event_id Event Id
 * @param flags Flags.  See enum VEHICLE_EVENT_FLAGS
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_vehicle_event_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t event_id,uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VEHICLE_EVENT_LEN];
	_mav_put_uint16_t(buf, 0, event_id);
	_mav_put_uint8_t(buf, 2, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_VEHICLE_EVENT_LEN);
#else
	mavlink_vehicle_event_t packet;
	packet.event_id = event_id;
	packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VEHICLE_EVENT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_VEHICLE_EVENT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VEHICLE_EVENT_LEN, MAVLINK_MSG_ID_VEHICLE_EVENT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_VEHICLE_EVENT_LEN);
#endif
}

/**
 * @brief Encode a vehicle_event struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param vehicle_event C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vehicle_event_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_vehicle_event_t* vehicle_event)
{
	return mavlink_msg_vehicle_event_pack(system_id, component_id, msg, vehicle_event->event_id, vehicle_event->flags);
}

/**
 * @brief Encode a vehicle_event struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vehicle_event C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_vehicle_event_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_vehicle_event_t* vehicle_event)
{
	return mavlink_msg_vehicle_event_pack_chan(system_id, component_id, chan, msg, vehicle_event->event_id, vehicle_event->flags);
}

/**
 * @brief Send a vehicle_event message
 * @param chan MAVLink channel to send the message
 *
 * @param event_id Event Id
 * @param flags Flags.  See enum VEHICLE_EVENT_FLAGS
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_vehicle_event_send(mavlink_channel_t chan, uint16_t event_id, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_VEHICLE_EVENT_LEN];
	_mav_put_uint16_t(buf, 0, event_id);
	_mav_put_uint8_t(buf, 2, flags);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_EVENT, buf, MAVLINK_MSG_ID_VEHICLE_EVENT_LEN, MAVLINK_MSG_ID_VEHICLE_EVENT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_EVENT, buf, MAVLINK_MSG_ID_VEHICLE_EVENT_LEN);
#endif
#else
	mavlink_vehicle_event_t packet;
	packet.event_id = event_id;
	packet.flags = flags;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_EVENT, (const char *)&packet, MAVLINK_MSG_ID_VEHICLE_EVENT_LEN, MAVLINK_MSG_ID_VEHICLE_EVENT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_EVENT, (const char *)&packet, MAVLINK_MSG_ID_VEHICLE_EVENT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_VEHICLE_EVENT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_vehicle_event_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t event_id, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, event_id);
	_mav_put_uint8_t(buf, 2, flags);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_EVENT, buf, MAVLINK_MSG_ID_VEHICLE_EVENT_LEN, MAVLINK_MSG_ID_VEHICLE_EVENT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_EVENT, buf, MAVLINK_MSG_ID_VEHICLE_EVENT_LEN);
#endif
#else
	mavlink_vehicle_event_t *packet = (mavlink_vehicle_event_t *)msgbuf;
	packet->event_id = event_id;
	packet->flags = flags;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_EVENT, (const char *)packet, MAVLINK_MSG_ID_VEHICLE_EVENT_LEN, MAVLINK_MSG_ID_VEHICLE_EVENT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_VEHICLE_EVENT, (const char *)packet, MAVLINK_MSG_ID_VEHICLE_EVENT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE VEHICLE_EVENT UNPACKING


/**
 * @brief Get field event_id from vehicle_event message
 *
 * @return Event Id
 */
static inline uint16_t mavlink_msg_vehicle_event_get_event_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field flags from vehicle_event message
 *
 * @return Flags.  See enum VEHICLE_EVENT_FLAGS
 */
static inline uint8_t mavlink_msg_vehicle_event_get_flags(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a vehicle_event message into a struct
 *
 * @param msg The message to decode
 * @param vehicle_event C-struct to decode the message contents into
 */
static inline void mavlink_msg_vehicle_event_decode(const mavlink_message_t* msg, mavlink_vehicle_event_t* vehicle_event)
{
#if MAVLINK_NEED_BYTE_SWAP
	vehicle_event->event_id = mavlink_msg_vehicle_event_get_event_id(msg);
	vehicle_event->flags = mavlink_msg_vehicle_event_get_flags(msg);
#else
	memcpy(vehicle_event, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_VEHICLE_EVENT_LEN);
#endif
}
