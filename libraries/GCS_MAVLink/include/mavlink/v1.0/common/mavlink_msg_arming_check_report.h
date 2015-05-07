// MESSAGE ARMING_CHECK_REPORT PACKING

#define MAVLINK_MSG_ID_ARMING_CHECK_REPORT 149

typedef struct __mavlink_arming_check_report_t
{
 uint64_t present_mask; ///< Present bitmask. 1 if this bit represents a check, 0 otherwise.  See ARMING_CHECK_STATUS enum for bit meanings
 uint64_t passed_mask; ///< Passed bitmask. 1 if this check has passed, 0 if pending or failed.  See ARMING_CHECK_STATUS enum for bit meanings
 uint64_t failed_mask; ///< Failed bitmask, 1 if this check has failed, 0 if pending or passed. See ARMING_CHECK_STATUS enum for bit meanings
} mavlink_arming_check_report_t;

#define MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN 24
#define MAVLINK_MSG_ID_149_LEN 24

#define MAVLINK_MSG_ID_ARMING_CHECK_REPORT_CRC 92
#define MAVLINK_MSG_ID_149_CRC 92



#define MAVLINK_MESSAGE_INFO_ARMING_CHECK_REPORT { \
	"ARMING_CHECK_REPORT", \
	3, \
	{  { "present_mask", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_arming_check_report_t, present_mask) }, \
         { "passed_mask", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_arming_check_report_t, passed_mask) }, \
         { "failed_mask", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_arming_check_report_t, failed_mask) }, \
         } \
}


/**
 * @brief Pack a arming_check_report message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param present_mask Present bitmask. 1 if this bit represents a check, 0 otherwise.  See ARMING_CHECK_STATUS enum for bit meanings
 * @param passed_mask Passed bitmask. 1 if this check has passed, 0 if pending or failed.  See ARMING_CHECK_STATUS enum for bit meanings
 * @param failed_mask Failed bitmask, 1 if this check has failed, 0 if pending or passed. See ARMING_CHECK_STATUS enum for bit meanings
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arming_check_report_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t present_mask, uint64_t passed_mask, uint64_t failed_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN];
	_mav_put_uint64_t(buf, 0, present_mask);
	_mav_put_uint64_t(buf, 8, passed_mask);
	_mav_put_uint64_t(buf, 16, failed_mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN);
#else
	mavlink_arming_check_report_t packet;
	packet.present_mask = present_mask;
	packet.passed_mask = passed_mask;
	packet.failed_mask = failed_mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARMING_CHECK_REPORT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN);
#endif
}

/**
 * @brief Pack a arming_check_report message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param present_mask Present bitmask. 1 if this bit represents a check, 0 otherwise.  See ARMING_CHECK_STATUS enum for bit meanings
 * @param passed_mask Passed bitmask. 1 if this check has passed, 0 if pending or failed.  See ARMING_CHECK_STATUS enum for bit meanings
 * @param failed_mask Failed bitmask, 1 if this check has failed, 0 if pending or passed. See ARMING_CHECK_STATUS enum for bit meanings
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_arming_check_report_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t present_mask,uint64_t passed_mask,uint64_t failed_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN];
	_mav_put_uint64_t(buf, 0, present_mask);
	_mav_put_uint64_t(buf, 8, passed_mask);
	_mav_put_uint64_t(buf, 16, failed_mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN);
#else
	mavlink_arming_check_report_t packet;
	packet.present_mask = present_mask;
	packet.passed_mask = passed_mask;
	packet.failed_mask = failed_mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ARMING_CHECK_REPORT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN);
#endif
}

/**
 * @brief Encode a arming_check_report struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param arming_check_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arming_check_report_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_arming_check_report_t* arming_check_report)
{
	return mavlink_msg_arming_check_report_pack(system_id, component_id, msg, arming_check_report->present_mask, arming_check_report->passed_mask, arming_check_report->failed_mask);
}

/**
 * @brief Encode a arming_check_report struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param arming_check_report C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_arming_check_report_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_arming_check_report_t* arming_check_report)
{
	return mavlink_msg_arming_check_report_pack_chan(system_id, component_id, chan, msg, arming_check_report->present_mask, arming_check_report->passed_mask, arming_check_report->failed_mask);
}

/**
 * @brief Send a arming_check_report message
 * @param chan MAVLink channel to send the message
 *
 * @param present_mask Present bitmask. 1 if this bit represents a check, 0 otherwise.  See ARMING_CHECK_STATUS enum for bit meanings
 * @param passed_mask Passed bitmask. 1 if this check has passed, 0 if pending or failed.  See ARMING_CHECK_STATUS enum for bit meanings
 * @param failed_mask Failed bitmask, 1 if this check has failed, 0 if pending or passed. See ARMING_CHECK_STATUS enum for bit meanings
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_arming_check_report_send(mavlink_channel_t chan, uint64_t present_mask, uint64_t passed_mask, uint64_t failed_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN];
	_mav_put_uint64_t(buf, 0, present_mask);
	_mav_put_uint64_t(buf, 8, passed_mask);
	_mav_put_uint64_t(buf, 16, failed_mask);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARMING_CHECK_REPORT, buf, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARMING_CHECK_REPORT, buf, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN);
#endif
#else
	mavlink_arming_check_report_t packet;
	packet.present_mask = present_mask;
	packet.passed_mask = passed_mask;
	packet.failed_mask = failed_mask;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARMING_CHECK_REPORT, (const char *)&packet, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARMING_CHECK_REPORT, (const char *)&packet, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_arming_check_report_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t present_mask, uint64_t passed_mask, uint64_t failed_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, present_mask);
	_mav_put_uint64_t(buf, 8, passed_mask);
	_mav_put_uint64_t(buf, 16, failed_mask);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARMING_CHECK_REPORT, buf, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARMING_CHECK_REPORT, buf, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN);
#endif
#else
	mavlink_arming_check_report_t *packet = (mavlink_arming_check_report_t *)msgbuf;
	packet->present_mask = present_mask;
	packet->passed_mask = passed_mask;
	packet->failed_mask = failed_mask;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARMING_CHECK_REPORT, (const char *)packet, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ARMING_CHECK_REPORT, (const char *)packet, MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ARMING_CHECK_REPORT UNPACKING


/**
 * @brief Get field present_mask from arming_check_report message
 *
 * @return Present bitmask. 1 if this bit represents a check, 0 otherwise.  See ARMING_CHECK_STATUS enum for bit meanings
 */
static inline uint64_t mavlink_msg_arming_check_report_get_present_mask(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field passed_mask from arming_check_report message
 *
 * @return Passed bitmask. 1 if this check has passed, 0 if pending or failed.  See ARMING_CHECK_STATUS enum for bit meanings
 */
static inline uint64_t mavlink_msg_arming_check_report_get_passed_mask(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field failed_mask from arming_check_report message
 *
 * @return Failed bitmask, 1 if this check has failed, 0 if pending or passed. See ARMING_CHECK_STATUS enum for bit meanings
 */
static inline uint64_t mavlink_msg_arming_check_report_get_failed_mask(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  16);
}

/**
 * @brief Decode a arming_check_report message into a struct
 *
 * @param msg The message to decode
 * @param arming_check_report C-struct to decode the message contents into
 */
static inline void mavlink_msg_arming_check_report_decode(const mavlink_message_t* msg, mavlink_arming_check_report_t* arming_check_report)
{
#if MAVLINK_NEED_BYTE_SWAP
	arming_check_report->present_mask = mavlink_msg_arming_check_report_get_present_mask(msg);
	arming_check_report->passed_mask = mavlink_msg_arming_check_report_get_passed_mask(msg);
	arming_check_report->failed_mask = mavlink_msg_arming_check_report_get_failed_mask(msg);
#else
	memcpy(arming_check_report, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ARMING_CHECK_REPORT_LEN);
#endif
}
