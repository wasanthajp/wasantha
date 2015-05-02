// MESSAGE REQUEST_VEHICLE_EVENT_INFO PACKING

#define MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO 227

typedef struct __mavlink_request_vehicle_event_info_t
{
 uint16_t event_id; ///< Event Id
} mavlink_request_vehicle_event_info_t;

#define MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN 2
#define MAVLINK_MSG_ID_227_LEN 2

#define MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_CRC 168
#define MAVLINK_MSG_ID_227_CRC 168



#define MAVLINK_MESSAGE_INFO_REQUEST_VEHICLE_EVENT_INFO { \
	"REQUEST_VEHICLE_EVENT_INFO", \
	1, \
	{  { "event_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_request_vehicle_event_info_t, event_id) }, \
         } \
}


/**
 * @brief Pack a request_vehicle_event_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param event_id Event Id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_vehicle_event_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t event_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN];
	_mav_put_uint16_t(buf, 0, event_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN);
#else
	mavlink_request_vehicle_event_info_t packet;
	packet.event_id = event_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN);
#endif
}

/**
 * @brief Pack a request_vehicle_event_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param event_id Event Id
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_vehicle_event_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t event_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN];
	_mav_put_uint16_t(buf, 0, event_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN);
#else
	mavlink_request_vehicle_event_info_t packet;
	packet.event_id = event_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN);
#endif
}

/**
 * @brief Encode a request_vehicle_event_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param request_vehicle_event_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_request_vehicle_event_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_request_vehicle_event_info_t* request_vehicle_event_info)
{
	return mavlink_msg_request_vehicle_event_info_pack(system_id, component_id, msg, request_vehicle_event_info->event_id);
}

/**
 * @brief Encode a request_vehicle_event_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param request_vehicle_event_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_request_vehicle_event_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_request_vehicle_event_info_t* request_vehicle_event_info)
{
	return mavlink_msg_request_vehicle_event_info_pack_chan(system_id, component_id, chan, msg, request_vehicle_event_info->event_id);
}

/**
 * @brief Send a request_vehicle_event_info message
 * @param chan MAVLink channel to send the message
 *
 * @param event_id Event Id
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_request_vehicle_event_info_send(mavlink_channel_t chan, uint16_t event_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN];
	_mav_put_uint16_t(buf, 0, event_id);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO, buf, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO, buf, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN);
#endif
#else
	mavlink_request_vehicle_event_info_t packet;
	packet.event_id = event_id;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO, (const char *)&packet, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO, (const char *)&packet, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_request_vehicle_event_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t event_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, event_id);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO, buf, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO, buf, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN);
#endif
#else
	mavlink_request_vehicle_event_info_t *packet = (mavlink_request_vehicle_event_info_t *)msgbuf;
	packet->event_id = event_id;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO, (const char *)packet, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO, (const char *)packet, MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE REQUEST_VEHICLE_EVENT_INFO UNPACKING


/**
 * @brief Get field event_id from request_vehicle_event_info message
 *
 * @return Event Id
 */
static inline uint16_t mavlink_msg_request_vehicle_event_info_get_event_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a request_vehicle_event_info message into a struct
 *
 * @param msg The message to decode
 * @param request_vehicle_event_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_request_vehicle_event_info_decode(const mavlink_message_t* msg, mavlink_request_vehicle_event_info_t* request_vehicle_event_info)
{
#if MAVLINK_NEED_BYTE_SWAP
	request_vehicle_event_info->event_id = mavlink_msg_request_vehicle_event_info_get_event_id(msg);
#else
	memcpy(request_vehicle_event_info, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_REQUEST_VEHICLE_EVENT_INFO_LEN);
#endif
}
