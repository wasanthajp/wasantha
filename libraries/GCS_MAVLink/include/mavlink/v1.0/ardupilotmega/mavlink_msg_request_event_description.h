// MESSAGE REQUEST_EVENT_DESCRIPTION PACKING

#define MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION 227

typedef struct __mavlink_request_event_description_t
{
 uint16_t eventId; ///< EventId
 uint8_t subsystem; ///< SubSystem (see EVENT_SUBSYSTEM enum)
} mavlink_request_event_description_t;

#define MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN 3
#define MAVLINK_MSG_ID_227_LEN 3

#define MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_CRC 1
#define MAVLINK_MSG_ID_227_CRC 1



#define MAVLINK_MESSAGE_INFO_REQUEST_EVENT_DESCRIPTION { \
	"REQUEST_EVENT_DESCRIPTION", \
	2, \
	{  { "eventId", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_request_event_description_t, eventId) }, \
         { "subsystem", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_request_event_description_t, subsystem) }, \
         } \
}


/**
 * @brief Pack a request_event_description message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param subsystem SubSystem (see EVENT_SUBSYSTEM enum)
 * @param eventId EventId
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_event_description_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t subsystem, uint16_t eventId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN];
	_mav_put_uint16_t(buf, 0, eventId);
	_mav_put_uint8_t(buf, 2, subsystem);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN);
#else
	mavlink_request_event_description_t packet;
	packet.eventId = eventId;
	packet.subsystem = subsystem;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN);
#endif
}

/**
 * @brief Pack a request_event_description message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param subsystem SubSystem (see EVENT_SUBSYSTEM enum)
 * @param eventId EventId
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_event_description_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t subsystem,uint16_t eventId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN];
	_mav_put_uint16_t(buf, 0, eventId);
	_mav_put_uint8_t(buf, 2, subsystem);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN);
#else
	mavlink_request_event_description_t packet;
	packet.eventId = eventId;
	packet.subsystem = subsystem;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN);
#endif
}

/**
 * @brief Encode a request_event_description struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param request_event_description C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_request_event_description_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_request_event_description_t* request_event_description)
{
	return mavlink_msg_request_event_description_pack(system_id, component_id, msg, request_event_description->subsystem, request_event_description->eventId);
}

/**
 * @brief Encode a request_event_description struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param request_event_description C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_request_event_description_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_request_event_description_t* request_event_description)
{
	return mavlink_msg_request_event_description_pack_chan(system_id, component_id, chan, msg, request_event_description->subsystem, request_event_description->eventId);
}

/**
 * @brief Send a request_event_description message
 * @param chan MAVLink channel to send the message
 *
 * @param subsystem SubSystem (see EVENT_SUBSYSTEM enum)
 * @param eventId EventId
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_request_event_description_send(mavlink_channel_t chan, uint8_t subsystem, uint16_t eventId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN];
	_mav_put_uint16_t(buf, 0, eventId);
	_mav_put_uint8_t(buf, 2, subsystem);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION, buf, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION, buf, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN);
#endif
#else
	mavlink_request_event_description_t packet;
	packet.eventId = eventId;
	packet.subsystem = subsystem;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION, (const char *)&packet, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION, (const char *)&packet, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_request_event_description_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t subsystem, uint16_t eventId)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, eventId);
	_mav_put_uint8_t(buf, 2, subsystem);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION, buf, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION, buf, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN);
#endif
#else
	mavlink_request_event_description_t *packet = (mavlink_request_event_description_t *)msgbuf;
	packet->eventId = eventId;
	packet->subsystem = subsystem;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION, (const char *)packet, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION, (const char *)packet, MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE REQUEST_EVENT_DESCRIPTION UNPACKING


/**
 * @brief Get field subsystem from request_event_description message
 *
 * @return SubSystem (see EVENT_SUBSYSTEM enum)
 */
static inline uint8_t mavlink_msg_request_event_description_get_subsystem(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field eventId from request_event_description message
 *
 * @return EventId
 */
static inline uint16_t mavlink_msg_request_event_description_get_eventId(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a request_event_description message into a struct
 *
 * @param msg The message to decode
 * @param request_event_description C-struct to decode the message contents into
 */
static inline void mavlink_msg_request_event_description_decode(const mavlink_message_t* msg, mavlink_request_event_description_t* request_event_description)
{
#if MAVLINK_NEED_BYTE_SWAP
	request_event_description->eventId = mavlink_msg_request_event_description_get_eventId(msg);
	request_event_description->subsystem = mavlink_msg_request_event_description_get_subsystem(msg);
#else
	memcpy(request_event_description, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_REQUEST_EVENT_DESCRIPTION_LEN);
#endif
}
